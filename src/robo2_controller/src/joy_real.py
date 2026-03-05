#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import serial
import threading
import time

class PS4RobotControllerDual(Node):
    def __init__(self):
        super().__init__('ps4_robot_controller_dual')
        
        # ═══════════════════════════════════════════════════════
        # PUBLICADORES ROS2 (GAZEBO)
        # ═══════════════════════════════════════════════════════
        self.arm_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )
        
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )
        
        # ═══════════════════════════════════════════════════════
        # COMUNICACIÓN SERIAL CON ARDUINO
        # ═══════════════════════════════════════════════════════
        self.arduino_connected = False
        self.arduino_port = None
        
        # Intentar conectar con el Arduino
        possible_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        
        for port in possible_ports:
            try:
                self.arduino_port = serial.Serial(
                    port=port,
                    baudrate=115200,
                    timeout=0.1,
                    write_timeout=1.0
                )
                self.arduino_connected = True
                self.get_logger().info(f'✓ Arduino connected via {port}')
                time.sleep(2)  # Esperar que Arduino se inicialice
                
                # Configurar Arduino en modo ABSOLUTO
                self.arduino_port.write(b'A\n')
                time.sleep(0.1)
                
                break
            except Exception as e:
                continue
        
        if not self.arduino_connected:
            self.get_logger().warning('⚠ Arduino not connected. Only Gazebo control active.')
        
        # ═══════════════════════════════════════════════════════
        # SUSCRIPTOR AL JOYSTICK
        # ═══════════════════════════════════════════════════════
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # ═══════════════════════════════════════════════════════
        # POSICIONES ACTUALES - GAZEBO (RADIANES)
        # ═══════════════════════════════════════════════════════
        self.base_to_base_rad = 0.0
        self.base_to_arm_rad = 0.0
        self.base_to_arm2_rad = 0.0
        self.gripper = 0.0 # Valor para Gazebo (0.0 a 0.19)
        
        # ═══════════════════════════════════════════════════════
        # POSICIONES ACTUALES - ARDUINO (GRADOS)
        # ═══════════════════════════════════════════════════════
        self.M1_deg = 0.0  # base_to_base
        self.M2_deg = 0.0  # base_to_arm
        self.M3_deg = 0.0  # base_to_arm2
        
        # Valor del Gripper para Arduino (Predeterminado 90/Centro)
        self.arduino_gripper_val = 90 
        
        # ═══════════════════════════════════════════════════════
        # VELOCIDADES OBJETIVO (PARA SUAVIZADO)
        # ═══════════════════════════════════════════════════════
        self.target_base_velocity = 0.0
        self.target_arm_velocity = 0.0
        
        self.current_base_velocity = 0.0
        self.current_arm_velocity = 0.0
        
        # ═══════════════════════════════════════════════════════
        # LÍMITES Y PARÁMETROS
        # ═══════════════════════════════════════════════════════
        self.limits_rad = {
            'base_to_base': (-1.57, 1.57),
            'base_to_arm': (-1.57, 1.57),
            'base_to_arm2': (-1.57, 1.57),
            'gripper': (0.0, 0.19)
        }
        
        self.limits_deg = {
            'M1': (-90.0, 90.0),    
            'M2': (-90.0, 90.0),    
            'M3': (-90.0, 90.0)     
        }
        
        self.joystick_max_speed_rad = 0.015      
        self.joystick_max_speed_deg = 0.86       
        self.button_speed_rad = 0.005            
        self.button_speed_deg = 0.29             
        self.gripper_speed = 0.005 # Velocidad visual para gazebo
        
        self.smoothing_factor = 0.15
        self.deadzone = 0.15
        
        self.prev_buttons = [0] * 20
        
        # ═══════════════════════════════════════════════════════
        # TIMERS
        # ═══════════════════════════════════════════════════════
        self.timer = self.create_timer(0.033, self.update_and_publish)
        self.arduino_timer = self.create_timer(0.033, self.send_to_arduino)
        self.arduino_command_pending = False
        
        self.get_logger().info('🎮 PS4 Robot Controller DUAL MODE initialized')

    
    def apply_deadzone(self, value):
        if abs(value) < self.deadzone: return 0.0
        sign = 1 if value > 0 else -1
        normalized = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * normalized
    
    def clamp(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))
    
    def smooth_velocity(self, current, target, factor):
        return current + (target - current) * factor
    
    def rad_to_deg(self, rad):
        return rad * 180.0 / 3.14159265359
    
    def joy_callback(self, msg):
        """Procesa los comandos del mando PS4"""
        
        # --- JOYSTICKS (Movimiento Motores) ---
        left_stick_y = self.apply_deadzone(msg.axes[1])
        self.target_arm_velocity = -left_stick_y * self.joystick_max_speed_rad
        
        right_stick_x = self.apply_deadzone(msg.axes[3])
        self.target_base_velocity = -right_stick_x * self.joystick_max_speed_rad
        
        # --- BOTONES MOTORES (M3) ---
        # Triángulo: M3 UP
        if msg.buttons[2] == 1:
            self.base_to_arm2_rad += self.button_speed_rad
            self.base_to_arm2_rad = self.clamp(self.base_to_arm2_rad, *self.limits_rad['base_to_arm2'])
            self.M3_deg += self.button_speed_deg
            self.M3_deg = self.clamp(self.M3_deg, *self.limits_deg['M3'])
            self.arduino_command_pending = True
        
        # X: M3 DOWN
        if msg.buttons[0] == 1:
            self.base_to_arm2_rad -= self.button_speed_rad
            self.base_to_arm2_rad = self.clamp(self.base_to_arm2_rad, *self.limits_rad['base_to_arm2'])
            self.M3_deg -= self.button_speed_deg
            self.M3_deg = self.clamp(self.M3_deg, *self.limits_deg['M3'])
            self.arduino_command_pending = True
        
        # --- BOTONES GRIPPER (Lógica Arduino + Gazebo) ---
        
        # CUADRADO (Button 3) -> CERRAR GRIPPER
        # Envía 110 al Arduino inmediatamente.
        if msg.buttons[3] == 1 and self.prev_buttons[3] == 0:
            self.get_logger().info("✊ Closing Gripper (110)")
            self.arduino_gripper_val = 110  # Valor para cerrar físico
            self.gripper = 0.0              # Valor cerrado en Gazebo
            self.arduino_command_pending = True

        # CÍRCULO (Button 1) -> ABRIR GRIPPER CON SECUENCIA
        # Secuencia: 80 -> espera 1s -> 90.
        if msg.buttons[1] == 1 and self.prev_buttons[1] == 0:
            self.perform_open_sequence()

        self.prev_buttons = list(msg.buttons)

    def perform_open_sequence(self):
        """Ejecuta la secuencia de abrir en un hilo separado para no bloquear el control"""
        def sequence():
            self.get_logger().info("✋ Opening Gripper Sequence: 80 -> Wait 1s -> 90")
            
            # 1. Enviar comando abrir (80)
            self.arduino_gripper_val = 80
            self.gripper = 0.19 # Abierto en Gazebo
            self.send_to_arduino() # Forzar envío inmediato
            
            # 2. Esperar 1 segundo (sin bloquear el resto del robot)
            time.sleep(1.0)
            
            # 3. Enviar comando relax (90)
            self.arduino_gripper_val = 90
            self.send_to_arduino() # Forzar envío
            self.get_logger().info("✋ Gripper Relaxed (90)")

        # Iniciar el hilo
        threading.Thread(target=sequence, daemon=True).start()
    
    def update_and_publish(self):
        """Actualiza posiciones y publica a Gazebo"""
        
        # Suavizado de velocidades
        self.current_base_velocity = self.smooth_velocity(self.current_base_velocity, self.target_base_velocity, self.smoothing_factor)
        self.current_arm_velocity = self.smooth_velocity(self.current_arm_velocity, self.target_arm_velocity, self.smoothing_factor)
        
        # Actualizar GAZEBO
        if abs(self.current_base_velocity) > 0.001:
            self.base_to_base_rad += self.current_base_velocity
            self.base_to_base_rad = self.clamp(self.base_to_base_rad, *self.limits_rad['base_to_base'])
        
        if abs(self.current_arm_velocity) > 0.001:
            self.base_to_arm_rad += self.current_arm_velocity
            self.base_to_arm_rad = self.clamp(self.base_to_arm_rad, *self.limits_rad['base_to_arm'])
        
        # Actualizar ARDUINO
        if abs(self.current_base_velocity) > 0.001:
            vel_deg = self.rad_to_deg(self.current_base_velocity)
            self.M1_deg += vel_deg
            self.M1_deg = self.clamp(self.M1_deg, *self.limits_deg['M1'])
            self.arduino_command_pending = True
        
        if abs(self.current_arm_velocity) > 0.001:
            vel_deg = self.rad_to_deg(self.current_arm_velocity)
            self.M2_deg += vel_deg
            self.M2_deg = self.clamp(self.M2_deg, *self.limits_deg['M2'])
            self.arduino_command_pending = True
        
        self.publish_gazebo_commands()
    
    def publish_gazebo_commands(self):
        arm_msg = Float64MultiArray()
        arm_msg.data = [self.base_to_base_rad, self.base_to_arm_rad, self.base_to_arm2_rad]
        self.arm_pub.publish(arm_msg)
        
        gripper_msg = Float64MultiArray()
        gripper_msg.data = [self.gripper]
        self.gripper_pub.publish(gripper_msg)
    
    def send_to_arduino(self):
        """Envía comandos al Arduino (4 VALORES)"""
        
        if not self.arduino_connected:
            return
            
        # Enviamos siempre que estemos conectados para mantener sincronía
        try:
            # FORMATO NUEVO: M1 M2 M3 GRIPPER
            # Usamos las posiciones actuales de los motores para no resetearlos a 0
            command = f"{self.M1_deg:.2f} {self.M2_deg:.2f} {self.M3_deg:.2f} {self.arduino_gripper_val}\n"
            self.arduino_port.write(command.encode())
            
            if self.arduino_port.in_waiting > 0:
                self.arduino_port.read(self.arduino_port.in_waiting)
            
        except Exception as e:
            self.get_logger().error(f'Error sending to Arduino: {e}')

    def reset_all(self):
        """Resetea a 0"""
        self.get_logger().info('🔄 Resetting all joints to zero...')
        self.base_to_base_rad = 0.0
        self.base_to_arm_rad = 0.0
        self.base_to_arm2_rad = 0.0
        self.gripper = 0.0
        
        self.M1_deg = 0.0
        self.M2_deg = 0.0
        self.M3_deg = 0.0
        self.arduino_gripper_val = 90
        
        self.publish_gazebo_commands()
        
        if self.arduino_connected:
            try:
                # Enviamos 0.01 para forzar movimiento físico a home, no calibración
                cmd = f"0.01 0.01 0.01 90\n"
                self.arduino_port.write(cmd.encode())
            except Exception as e:
                self.get_logger().error(f'Error resetting Arduino: {e}')
        
        self.get_logger().info('✓ Reset complete')
    
    def __del__(self):
        if self.arduino_connected and self.arduino_port:
            try:
                self.arduino_port.write(b'H\n')
                time.sleep(0.5)
                self.arduino_port.close()
                self.get_logger().info('✓ Arduino connection closed')
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    controller = PS4RobotControllerDual()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('🎮 PS4 controller shutting down')
        controller.reset_all()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()