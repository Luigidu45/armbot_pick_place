#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import time
import numpy as np
import serial
import threading

class CameraGuidedArmController(Node):
    def __init__(self):
        super().__init__('camera_guided_arm_controller')
        
        # Publicador del brazo
        self.arm_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )
        
        # Suscriptor a la posición del objeto detectado por YOLO
        self.object_sub = self.create_subscription(
            Point,
            '/yolo/object_position',
            self.object_position_callback,
            10
        )
        
        # Configuración de comunicación serial con ESP32
        self.serial_connected = False
        self.serial_port = None
        
        # Intentar conectar con diferentes puertos comunes
        possible_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        
        for port in possible_ports:
            try:
                self.serial_port = serial.Serial(
                    port=port,
                    baudrate=115200,
                    timeout=0.1
                )
                self.serial_connected = True
                self.get_logger().info(f'ESP32 connected via {port}')
                time.sleep(2)  # Esperar a que el ESP32 se inicialice
                break
            except Exception as e:
                continue
        
        if not self.serial_connected:
            self.get_logger().warning('Could not connect to ESP32. LED control disabled.')
            self.get_logger().warning('Available ports to try: /dev/ttyUSB0, /dev/ttyACM0')
        
        # Resolución de la cámara
        self.camera_width = 320
        self.camera_height = 240
        
        # Posiciones límite del brazo
        # Cuando y=240 (adelante/lejos): brazo en (0, -1.55, -0.57) -> LED MÁS BRILLANTE
        # Cuando y=120 (atrás/cerca): brazo en (0, -1.085, -1.55) -> LED MENOS BRILLANTE
        self.position_front = (0.0, -1.55, -0.57)   # y=240 (LED brillante)
        self.position_back = (0.0, -1.085, -1.55)   # y=120 (LED tenue)
        
        # Zona central en x (tolerancia)
        self.center_x = 160
        self.x_tolerance = 30
        
        # Rango de y válido
        self.y_min = 0
        self.y_max = 240
        
        # Variables de estado
        self.last_object_x = None
        self.last_object_y = None
        self.tracking_enabled = False
        self.last_command_time = time.time()
        self.command_rate = 0.1
        
        # Variable para el brillo del LED
        self.current_led_brightness = 0
        
        # Timer para actualizar posición del brazo
        self.control_timer = self.create_timer(0.1, self.update_arm_position)
        
        self.get_logger().info('Camera Guided Arm Controller initialized')
        self.get_logger().info(f'Camera resolution: {self.camera_width}x{self.camera_height}')
        self.get_logger().info(f'Tracking zone: X={self.center_x}±{self.x_tolerance}, Y=[{self.y_min}, {self.y_max}]')
        self.get_logger().info(f'Front position (y=240): {self.position_front} -> LED BRIGHT')
        self.get_logger().info(f'Back position (y=120): {self.position_back} -> LED DIM')
        
        time.sleep(1.0)
    
    def object_position_callback(self, msg):
        """Callback que recibe la posición del objeto detectado"""
        self.last_object_x = msg.x
        self.last_object_y = msg.y
    
    def send_arm_command(self, base_to_base, base_to_arm, base_to_arm2):
        """Envía comando al brazo"""
        msg = Float64MultiArray()
        msg.data = [base_to_base, base_to_arm, base_to_arm2]
        self.arm_pub.publish(msg)
    
    def send_led_brightness(self, brightness):
        """
        Envía el valor de brillo al ESP32 via serial
        
        Args:
            brightness: valor entre 0 (apagado) y 255 (máximo brillo)
        """
        if not self.serial_connected or self.serial_port is None:
            return
        
        try:
            # Limitar el brillo entre 0 y 255
            brightness = max(0, min(255, int(brightness)))
            
            # Enviar comando al ESP32 en formato "B:XXX\n"
            command = f"B:{brightness}\n"
            self.serial_port.write(command.encode())
            
            # Guardar el último valor enviado
            self.current_led_brightness = brightness
            
        except Exception as e:
            self.get_logger().error(f'Error sending to ESP32: {e}')
    
    def calculate_led_brightness(self, y_pos):
        """
        Calcula el brillo del LED basado en la posición Y del objeto
        
        Args:
            y_pos: posición Y del objeto (0-240)
        
        Returns:
            int: brillo del LED (0-255)
        """
        # Limitar y_pos al rango válido
        y_pos = max(self.y_min, min(self.y_max, y_pos))
        
        # Normalizar y_pos de [0, 240] a [0, 1]
        # y=240 (adelante/lejos) -> brillo máximo (255)
        # y=0 (atrás/cerca) -> brillo mínimo (0)
        normalized = y_pos / self.y_max
        
        # Mapear a rango de brillo PWM (0-255)
        brightness = int(normalized * 255)
        
        return brightness
    
    def map_y_to_arm_position(self, y_pos):
        """
        Mapea la posición Y de la cámara a la posición del brazo
        
        Args:
            y_pos: posición Y del objeto en la imagen (0-240)
        
        Returns:
            tupla (base, arm, arm2) correspondiente
        """
        # Limitar y_pos al rango válido
        y_pos = max(self.y_min, min(self.y_max, y_pos))
        
        # Normalizar y_pos de [0, 240] a [0, 1]
        # y=240 (adelante) -> alpha=0
        # y=0 (atrás) -> alpha=1
        alpha = (self.y_max - y_pos) / self.y_max
        
        # Interpolar entre position_front y position_back
        base = self.position_front[0] + alpha * (self.position_back[0] - self.position_front[0])
        arm = self.position_front[1] + alpha * (self.position_back[1] - self.position_front[1])
        arm2 = self.position_front[2] + alpha * (self.position_back[2] - self.position_front[2])
        
        return (base, arm, arm2)
    
    def is_object_in_tracking_zone(self, x, y):
        """
        Verifica si el objeto está en la zona de seguimiento
        
        Args:
            x: posición X del objeto
            y: posición Y del objeto
        
        Returns:
            bool: True si está en la zona de tracking
        """
        x_in_range = (self.center_x - self.x_tolerance) <= x <= (self.center_x + self.x_tolerance)
        y_in_range = self.y_min <= y <= self.y_max
        
        return x_in_range and y_in_range
    
    def update_arm_position(self):
        """Actualiza la posición del brazo y el LED según la detección del objeto"""
        
        if self.last_object_x is None or self.last_object_y is None:
            # No hay detección reciente
            # Apagar LED si no hay detección
            if self.serial_connected and self.current_led_brightness != 0:
                self.send_led_brightness(0)
            return
        
        x = self.last_object_x
        y = self.last_object_y
        
        # Verificar si el objeto está en la zona de tracking
        if self.is_object_in_tracking_zone(x, y):
            
            if not self.tracking_enabled:
                self.get_logger().info('Object entered tracking zone - Starting tracking')
                self.tracking_enabled = True
            
            # Calcular posición del brazo basada en Y
            target_position = self.map_y_to_arm_position(y)
            
            # Enviar comando al brazo
            self.send_arm_command(*target_position)
            
            # Calcular y enviar brillo del LED
            led_brightness = self.calculate_led_brightness(y)
            self.send_led_brightness(led_brightness)
            
            # Log detallado cada segundo
            current_time = time.time()
            if current_time - self.last_command_time > 1.0:
                self.get_logger().info(
                    f'Tracking: Object(x={x:.1f}, y={y:.1f}) -> '
                    f'Arm({target_position[0]:.2f}, {target_position[1]:.2f}, {target_position[2]:.2f}) '
                    f'LED: {led_brightness}/255'
                )
                self.last_command_time = current_time
        
        else:
            if self.tracking_enabled:
                self.get_logger().info('Object left tracking zone - Stopping tracking')
                self.tracking_enabled = False
                # Apagar LED cuando sale de la zona
                self.send_led_brightness(0)
    
    def move_to_safe_position(self):
        """Mueve el brazo a una posición segura"""
        self.get_logger().info('Moving to safe position')
        self.send_arm_command(0.0, 0.0, 0.0)
        self.send_led_brightness(0)  # Apagar LED
        time.sleep(5.0)
    
    def test_sweep(self):
        """Prueba el rango completo de movimiento (adelante -> atrás) con LED"""
        self.get_logger().info('=== Testing full range of motion with LED ===')
        
        # Posición segura
        self.move_to_safe_position()
        
        # Adelante (LED brillante)
        self.get_logger().info('Moving to FRONT position (y=240) - LED BRIGHT')
        self.send_arm_command(*self.position_front)
        self.send_led_brightness(255)  # Máximo brillo
        time.sleep(3.0)
        
        # Interpolación suave de adelante hacia atrás
        self.get_logger().info('Sweeping FRONT -> BACK with LED fade')
        steps = 20
        for i in range(steps + 1):
            alpha = i / steps
            
            # Posición del brazo
            pos = tuple(
                self.position_front[j] + alpha * (self.position_back[j] - self.position_front[j])
                for j in range(3)
            )
            self.send_arm_command(*pos)
            
            # Brillo del LED (va de 255 a 0)
            brightness = int(255 * (1 - alpha))
            self.send_led_brightness(brightness)
            
            time.sleep(0.3)
        
        self.get_logger().info('Reached BACK position (y=120) - LED DIM')
        time.sleep(2.0)
        
        # Volver a seguro
        self.move_to_safe_position()
        
        self.get_logger().info('=== Test completed ===')
    
    def __del__(self):
        """Cleanup al destruir el nodo"""
        if self.serial_connected and self.serial_port:
            try:
                self.send_led_brightness(0)  # Apagar LED
                self.serial_port.close()
                self.get_logger().info('Serial port closed')
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    
    controller = CameraGuidedArmController()
    
    try:
        # OPCIÓN 1: Modo tracking (sigue el objeto continuamente con LED)
        controller.get_logger().info('=== Starting Camera-Guided Tracking Mode with LED ===')
        controller.get_logger().info('Move object in front of camera (x≈160, y=0-240)')
        controller.get_logger().info('LED brightness: Far (y→240) = BRIGHT, Near (y→0) = DIM')
        controller.move_to_safe_position()
        rclpy.spin(controller)
        
        # OPCIÓN 2: Prueba de rango con LED (descomentar para probar)
        # controller.test_sweep()
        # rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Tracking interrupted by user')
        controller.move_to_safe_position()
    
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()