#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class PickAndPlaceController(Node):
    def __init__(self):
        super().__init__('pick_and_place_controller')
        
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
        
        self.get_logger().info('Pick and Place Controller initialized')
        
        # Esperar a que los publicadores estén listos
        time.sleep(1.0)
        
    def send_arm_command(self, base_to_base, base_to_arm, base_to_arm2):
        """Envía comando al brazo [base_to_base, base_to_arm, base_to_arm2]"""
        msg = Float64MultiArray()
        msg.data = [base_to_base, base_to_arm, base_to_arm2]
        self.arm_pub.publish(msg)
        self.get_logger().info(f'Arm command: [{base_to_base:.2f}, {base_to_arm:.2f}, {base_to_arm2:.2f}]')
    
    def send_gripper_command(self, position):
        """Envía comando al gripper (0.0 = cerrado, 0.1-0.195 = abierto)"""
        msg = Float64MultiArray()
        msg.data = [position]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'Gripper command: {position:.3f}')
    
    def pick_and_place_sequence(self):
        """Ejecuta la secuencia completa de pick and place"""
        
        self.get_logger().info('=== Starting Pick and Place Sequence ===')
        
        # Posición inicial: brazo arriba, gripper abierto
        self.get_logger().info('Step 1: Moving to initial position (up, gripper open)')
        self.send_arm_command(0.0, 0.0, 0.0)  # Brazo en posición neutral
        self.send_gripper_command(0.0)  # Gripper abierto
        time.sleep(3.0)
        
        # Paso 1: Agacharse para agarrar el objeto
        self.get_logger().info('Step 2: Moving down to pick object')
        self.send_arm_command(0.0, -0.8, -0.720)  # Agacharse
        time.sleep(3.0)
        
        # Paso 2: Cerrar el gripper (agarrar objeto)
        self.get_logger().info('Step 3: Closing gripper (grabbing object)')
        self.send_gripper_command(0.16)  # Cerrar gripper
        time.sleep(2.0)
        
        
        
        # Paso 4: Rotar la base 90 grados (1.57 radianes)
        self.get_logger().info('Step 5: Rotating base 90 degrees')
        self.send_arm_command(1.57, 0.0, 0.0)  # Rotar base
        time.sleep(5.0)
        
        # Paso 5: Bajar un poco para colocar el objeto
        self.get_logger().info('Step 6: Moving down to place object')
        self.send_arm_command(1.57, -0.8, -0.720)  # Bajar un poco
        time.sleep(3.0)
        
        # Paso 6: Abrir el gripper (soltar objeto)
        self.get_logger().info('Step 7: Opening gripper (releasing object)')
        self.send_gripper_command(0.0)  # Abrir gripper
        time.sleep(2.0)

        # Movimiento gradual en 3 pasos
        self.send_arm_command(1.05, 0.0, 0.0)  # 60 grados
        time.sleep(1.5)

        self.send_arm_command(0.52, 0.0, 0.0)  # 30 grados
        time.sleep(1.5)

        self.send_arm_command(0.2, 0.0, 0.0)  # 30 grados
        time.sleep(1.5)

        self.send_arm_command(0.0, 0.0, 0.0)  # 0 grados
        time.sleep(2.0)
        
        
        self.get_logger().info('=== Pick and Place Sequence Completed ===')

def main(args=None):
    rclpy.init(args=args)
    
    controller = PickAndPlaceController()
    
    try:
        # Ejecutar la secuencia
        controller.pick_and_place_sequence()
        
    except KeyboardInterrupt:
        controller.get_logger().info('Pick and Place interrupted by user')
    
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()