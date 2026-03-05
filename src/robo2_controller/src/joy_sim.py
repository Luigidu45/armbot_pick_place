#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class PS4RobotController(Node):
    def __init__(self):
        super().__init__('ps4_robot_controller')
        
        # Publicadores
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
        
        # Suscriptor al joystick
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Posiciones actuales de las juntas
        self.base_to_base = 0.0
        self.base_to_arm = 0.0
        self.base_to_arm2 = 0.0
        self.gripper = 0.0
        
        # Velocidades objetivo (para suavizado)
        self.target_base_velocity = 0.0
        self.target_arm_velocity = 0.0
        
        # Velocidades actuales (interpoladas)
        self.current_base_velocity = 0.0
        self.current_arm_velocity = 0.0
        
        # Límites de las juntas
        self.limits = {
            'base_to_base': (-1.57, 1.57),
            'base_to_arm': (-1.57, 1.57),
            'base_to_arm2': (-1.57, 1.57),
            'gripper': (0.0, 0.19)  # 0.0 = abierto, 0.19 = cerrado
        }
        
        # Velocidades de control incremental (AJUSTADAS)
        self.joystick_max_speed = 0.015  # Velocidad máxima más lenta
        self.button_speed = 0.005         # Velocidad de botones
        self.gripper_speed = 0.005        # Velocidad del gripper
        
        # Factor de suavizado (0.0 a 1.0, menor = más suave)
        self.smoothing_factor = 0.15  # Suaviza la transición de velocidad
        
        # Estados anteriores de botones
        self.prev_buttons = [0] * 20
        
        # Deadzone para los joysticks (AJUSTADO)
        self.deadzone = 0.15  # Un poco más alto para ignorar drift
        
        # Timer para publicar comandos continuamente
        self.timer = self.create_timer(0.05, self.update_and_publish)  # 20 Hz
        
        self.get_logger().info('🎮 PS4 Robot Controller initialized (SMOOTH MODE)')
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick ⬆️⬇️  : base_to_arm (↑ increase, ↓ decrease)')
        self.get_logger().info('  Right Stick ⬅️➡️ : base_to_base (→ increase, ← decrease)')
        self.get_logger().info('  Triangle 🔺     : base_to_arm2 increase')
        self.get_logger().info('  X ❌           : base_to_arm2 decrease')
        self.get_logger().info('  Square 🟦      : Close Gripper (0.19)')
        self.get_logger().info('  Circle ⭕      : Open Gripper (0.0)')
        self.get_logger().info('═══════════════════════════════════════')
    
    def apply_deadzone(self, value):
        """Aplica zona muerta a los valores del joystick"""
        if abs(value) < self.deadzone:
            return 0.0
        
        # Normalizar el valor fuera de la deadzone
        # Esto hace que la transición sea más suave
        sign = 1 if value > 0 else -1
        normalized = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * normalized
    
    def clamp(self, value, min_val, max_val):
        """Limita un valor entre min y max"""
        return max(min_val, min(max_val, value))
    
    def smooth_velocity(self, current, target, factor):
        """Interpola suavemente entre velocidad actual y objetivo"""
        return current + (target - current) * factor
    
    def joy_callback(self, msg):
        """Callback que procesa los comandos del mando PS4"""
        
        # ═══════════════════════════════════════════════════════
        # JOYSTICKS - Control con suavizado
        # ═══════════════════════════════════════════════════════
        
        # JOYSTICK IZQUIERDO VERTICAL (axes[1]) -> base_to_arm
        # axes[1] = 1 (adelante) → aumenta
        # axes[1] = -1 (atrás) → disminuye
        left_stick_y = self.apply_deadzone(msg.axes[1])
        self.target_arm_velocity = -left_stick_y * self.joystick_max_speed
        
        # JOYSTICK DERECHO HORIZONTAL (axes[3]) -> base_to_base
        # axes[3] = -1 (derecha) → aumenta
        # axes[3] = 1 (izquierda) → disminuye
        right_stick_x = self.apply_deadzone(msg.axes[3])
        self.target_base_velocity = -right_stick_x * self.joystick_max_speed  # Invertido
        
        # ═══════════════════════════════════════════════════════
        # BOTONES - Control incremental
        # ═══════════════════════════════════════════════════════
        
        # TRIÁNGULO (buttons[2]) -> base_to_arm2 AUMENTA
        if msg.buttons[2] == 1:
            self.base_to_arm2 += self.button_speed
            self.base_to_arm2 = self.clamp(
                self.base_to_arm2,
                *self.limits['base_to_arm2']
            )
            if self.prev_buttons[2] == 0:
                self.get_logger().info(f'🔺 base_to_arm2: {self.base_to_arm2:.2f}')
        
        # X (buttons[0]) -> base_to_arm2 DISMINUYE
        if msg.buttons[0] == 1:
            self.base_to_arm2 -= self.button_speed
            self.base_to_arm2 = self.clamp(
                self.base_to_arm2,
                *self.limits['base_to_arm2']
            )
            if self.prev_buttons[0] == 0:
                self.get_logger().info(f'❌ base_to_arm2: {self.base_to_arm2:.2f}')
        
        # CUADRADO (buttons[3]) -> CERRAR GRIPPER
        if msg.buttons[3] == 1:
            self.gripper += self.gripper_speed
            self.gripper = self.clamp(
                self.gripper,
                *self.limits['gripper']
            )
            if self.prev_buttons[3] == 0:
                self.get_logger().info(f'🟦 Closing gripper: {self.gripper:.2f}')
        
        # CÍRCULO (buttons[1]) -> ABRIR GRIPPER
        if msg.buttons[1] == 1:
            self.gripper -= self.gripper_speed
            self.gripper = self.clamp(
                self.gripper,
                *self.limits['gripper']
            )
            if self.prev_buttons[1] == 0:
                self.get_logger().info(f'⭕ Opening gripper: {self.gripper:.2f}')
        
        # Guardar estado de botones
        self.prev_buttons = list(msg.buttons)
    
    def update_and_publish(self):
        """Actualiza las posiciones con suavizado y publica comandos"""
        
        # Suavizar las velocidades (interpolación exponencial)
        self.current_base_velocity = self.smooth_velocity(
            self.current_base_velocity,
            self.target_base_velocity,
            self.smoothing_factor
        )
        
        self.current_arm_velocity = self.smooth_velocity(
            self.current_arm_velocity,
            self.target_arm_velocity,
            self.smoothing_factor
        )
        
        # Aplicar velocidades a las posiciones
        if abs(self.current_base_velocity) > 0.001:  # Umbral mínimo
            self.base_to_base += self.current_base_velocity
            self.base_to_base = self.clamp(
                self.base_to_base,
                *self.limits['base_to_base']
            )
        
        if abs(self.current_arm_velocity) > 0.001:  # Umbral mínimo
            self.base_to_arm += self.current_arm_velocity
            self.base_to_arm = self.clamp(
                self.base_to_arm,
                *self.limits['base_to_arm']
            )
        
        # Publicar comandos
        self.publish_commands()
    
    def publish_commands(self):
        """Publica los comandos del brazo y gripper"""
        
        # Publicar comando del brazo
        arm_msg = Float64MultiArray()
        arm_msg.data = [self.base_to_base, self.base_to_arm, self.base_to_arm2]
        self.arm_pub.publish(arm_msg)
        
        # Publicar comando del gripper
        gripper_msg = Float64MultiArray()
        gripper_msg.data = [self.gripper]
        self.gripper_pub.publish(gripper_msg)


def main(args=None):
    rclpy.init(args=args)
    
    controller = PS4RobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('🎮 PS4 controller shutting down')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()