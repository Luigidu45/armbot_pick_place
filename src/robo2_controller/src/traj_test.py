#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import time
import numpy as np

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
        
        # Resolución de la cámara
        self.camera_width = 320
        self.camera_height = 240
        
        # Posiciones límite del brazo
        # Cuando y=240 (adelante): brazo en (0, -1.55, -0.57)
        # Cuando y=120 (atrás): brazo en (0, -1.085, -1.55)
        self.position_front = (0.0, -1.55, -0.57)   # y=240
        self.position_back = (0.0, -1.085, -1.55)   # y=120
        
        # Zona central en x (tolerancia)
        self.center_x = 160
        self.x_tolerance = 30  # ±30 pixels de tolerancia
        
        # Rango de y válido
        self.y_min = 0
        self.y_max = 240
        
        # Variables de estado
        self.last_object_x = None
        self.last_object_y = None
        self.tracking_enabled = False
        self.last_command_time = time.time()
        self.command_rate = 0.1  # Enviar comandos cada 0.1 segundos
        
        # Timer para actualizar posición del brazo
        self.control_timer = self.create_timer(0.1, self.update_arm_position)
        
        self.get_logger().info('Camera Guided Arm Controller initialized')
        self.get_logger().info(f'Camera resolution: {self.camera_width}x{self.camera_height}')
        self.get_logger().info(f'Tracking zone: X={self.center_x}±{self.x_tolerance}, Y=[{self.y_min}, {self.y_max}]')
        self.get_logger().info(f'Front position (y=240): {self.position_front}')
        self.get_logger().info(f'Back position (y=120): {self.position_back}')
        
        time.sleep(1.0)
    
    def object_position_callback(self, msg):
        """Callback que recibe la posición del objeto detectado"""
        self.last_object_x = msg.x
        self.last_object_y = msg.y
        
        # Log periódico (cada 1 segundo aproximadamente)
        current_time = time.time()
        if current_time - self.last_command_time > 1.0:
            self.get_logger().info(f'Object detected at: x={msg.x:.1f}, y={msg.y:.1f}')
    
    def send_arm_command(self, base_to_base, base_to_arm, base_to_arm2):
        """Envía comando al brazo"""
        msg = Float64MultiArray()
        msg.data = [base_to_base, base_to_arm, base_to_arm2]
        self.arm_pub.publish(msg)
    
    def map_y_to_arm_position(self, y_pos):
        """
        Mapea la posición Y de la cámara a la posición del brazo
        
        Args:
            y_pos: posición Y del objeto en la imagen (120-240)
        
        Returns:
            tupla (base, arm, arm2) correspondiente
        """
        # Limitar y_pos al rango válido
        y_pos = max(self.y_min, min(self.y_max, y_pos))
        
        # Normalizar y_pos de [120, 240] a [0, 1]
        # y=240 (adelante) -> alpha=0
        # y=120 (atrás) -> alpha=1
        alpha = (self.y_max - y_pos) / (self.y_max - self.y_min)
        
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
        """Actualiza la posición del brazo según la detección del objeto"""
        
        if self.last_object_x is None or self.last_object_y is None:
            # No hay detección reciente
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
            
            # Log detallado cada segundo
            current_time = time.time()
            if current_time - self.last_command_time > 1.0:
                self.get_logger().info(
                    f'Tracking: Object(x={x:.1f}, y={y:.1f}) -> '
                    f'Arm({target_position[0]:.2f}, {target_position[1]:.2f}, {target_position[2]:.2f})'
                )
                self.last_command_time = current_time
        
        else:
            if self.tracking_enabled:
                self.get_logger().info('Object left tracking zone - Stopping tracking')
                self.tracking_enabled = False
    
    def move_to_safe_position(self):
        """Mueve el brazo a una posición segura"""
        self.get_logger().info('Moving to safe position')
        self.send_arm_command(0.0, 0.0, 0.0)
        time.sleep(5.0)
    
    def test_sweep(self):
        """Prueba el rango completo de movimiento (adelante -> atrás)"""
        self.get_logger().info('=== Testing full range of motion ===')
        
        # Posición segura
        self.move_to_safe_position()
        
        # Adelante
        self.get_logger().info('Moving to FRONT position (y=240)')
        self.send_arm_command(*self.position_front)
        time.sleep(13.0)
        
        # Interpolación suave de adelante hacia atrás
        self.get_logger().info('Sweeping FRONT -> BACK')
        steps = 20
        for i in range(steps + 1):
            alpha = i / steps
            pos = tuple(
                self.position_front[j] + alpha * (self.position_back[j] - self.position_front[j])
                for j in range(3)
            )
            self.send_arm_command(*pos)
            time.sleep(0.2)
        
        self.get_logger().info('Reached BACK position (y=120)')
        time.sleep(5.0)
        
        # Volver a seguro
        self.move_to_safe_position()
        
        self.get_logger().info('=== Test completed ===')

def main(args=None):
    rclpy.init(args=args)
    
    controller = CameraGuidedArmController()
    
    try:
        # OPCIÓN 1: Modo tracking (sigue el objeto continuamente)
        controller.get_logger().info('=== Starting Camera-Guided Tracking Mode ===')
        controller.get_logger().info('Move object in front of camera (x≈160, y=120-240)')
        controller.move_to_safe_position()
        rclpy.spin(controller)
        
        # OPCIÓN 2: Prueba de rango (descomentar para probar)
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