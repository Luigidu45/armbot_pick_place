#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import time
import threading

class AutoTrajectoryNode(Node):
    def __init__(self):
        super().__init__('auto_trajectory_node')
        
        # ════════ PUBLICADORES ROS2 ════════
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
        
        # ════════ CONEXIÓN SERIAL ARDUINO ════════
        self.arduino_connected = False
        self.arduino_port = None
        self.connect_arduino()
        
        # ════════ ESTADO INTERNO ════════
        # Posiciones (Radianes para Gazebo, Grados para Arduino)
        self.q1_rad = 0.0
        self.q2_rad = 0.0
        self.q3_rad = 0.0
        
        self.m1_deg = 0.0
        self.m2_deg = 0.0
        self.m3_deg = 0.0
        self.gripper_val = 90 # 90 = Relax/Centro
        self.gazebo_gripper = 0.0

        self.get_logger().info('🚀 Nodo de Trayectoria Automática Iniciado')

    def connect_arduino(self):
        possible_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        for port in possible_ports:
            try:
                self.arduino_port = serial.Serial(port, 115200, timeout=0.1)
                self.arduino_connected = True
                self.get_logger().info(f'✓ Arduino conectado en {port}')
                time.sleep(2) # Esperar reinicio
                self.arduino_port.write(b'A\n') # Modo Absoluto
                time.sleep(0.1)
                return
            except: continue
        self.get_logger().warning('⚠ Arduino NO conectado (Solo Simulación)')

    def rad_to_deg(self, rad):
        return rad * 180.0 / 3.14159265359

    def send_commands(self):
        """Envía los comandos actuales a Gazebo y Arduino"""
        
        # 1. Enviar a Gazebo
        msg_arm = Float64MultiArray()
        msg_arm.data = [self.q1_rad, self.q2_rad, self.q3_rad]
        self.arm_pub.publish(msg_arm)
        
        msg_grip = Float64MultiArray()
        msg_grip.data = [self.gazebo_gripper]
        self.gripper_pub.publish(msg_grip)
        
        # 2. Enviar a Arduino
        if self.arduino_connected:
            try:
                # Formato: M1 M2 M3 Gripper
                cmd = f"{self.m1_deg:.2f} {self.m2_deg:.2f} {self.m3_deg:.2f} {self.gripper_val}\n"
                self.arduino_port.write(cmd.encode())
                # Limpiar buffer de entrada para no acumular basura
                if self.arduino_port.in_waiting:
                    self.arduino_port.read(self.arduino_port.in_waiting)
            except Exception as e:
                self.get_logger().error(f"Error serial: {e}")

    # ════════ FUNCIONES DE MOVIMIENTO (ALTO NIVEL) ════════
    
    def mover_brazo(self, j1, j2, j3, tiempo_espera=3.0):
        """Mueve el brazo a la posición (radianes) y espera"""
        self.get_logger().info(f"📍 Moviendo a: [{j1}, {j2}, {j3}]")
        
        # Actualizar variables
        self.q1_rad = float(j1)
        self.q2_rad = float(j2)
        self.q3_rad = float(j3)
        
        # Convertir a grados para Arduino
        self.m1_deg = self.rad_to_deg(self.q1_rad)
        self.m2_deg = self.rad_to_deg(self.q2_rad)
        self.m3_deg = self.rad_to_deg(self.q3_rad)
        
        # Ejecutar
        self.send_commands()
        time.sleep(tiempo_espera) # Esperar a que llegue

    def cerrar_gripper(self):
        """Cierra el gripper (Envía 110)"""
        self.get_logger().info("✊ Cerrando Gripper...")
        self.gripper_val = 110
        self.gazebo_gripper = 0.0 # Cerrado visual
        self.send_commands()
        time.sleep(1.0) # Tiempo para asegurar agarre

    def abrir_gripper(self):
        """Secuencia de apertura: 80 -> 1s -> 90"""
        self.get_logger().info("✋ Abriendo Gripper (Secuencia)...")
        
        # Paso 1: Abrir (80)
        self.gripper_val = 80
        self.gazebo_gripper = 0.19 # Abierto visual
        self.send_commands()
        time.sleep(1.0)
        
        # Paso 2: Relajar (90)
        self.get_logger().info("✋ Gripper Relajado (90)")
        self.gripper_val = 90
        self.send_commands()
        time.sleep(0.5)

    def ir_a_home(self):
        """Vuelve a 0 0 0"""
        self.get_logger().info("🏠 Yendo a HOME...")
        # Usamos 0.01 para forzar movimiento físico en Arduino (evitar bug de reset)
        self.q1_rad = 0.0
        self.q2_rad = 0.0
        self.q3_rad = 0.0
        self.m1_deg = 0.01
        self.m2_deg = 0.01
        self.m3_deg = 0.01
        self.send_commands()
        time.sleep(4.0)

# ════════ EJECUCIÓN PRINCIPAL ════════

def main(args=None):
    rclpy.init(args=args)
    robot = AutoTrajectoryNode()
    
    # Hilo para mantener ROS vivo (opcional para scripts secuenciales simples, 
    # pero buena práctica si necesitas recibir datos de sensores)
    spinner = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    spinner.start()

    try:
        print("\n--- INICIANDO SECUENCIA AUTOMÁTICA ---\n")
        
        # 1. Posición Inicial (Pick)
        # "0 -1.57 0.60"
        # robot.mover_brazo(0.0, -1.50, 0.6, tiempo_espera=2.5)
        robot.mover_brazo(0.3, -1.4, 0.5, tiempo_espera=2.5)

        # 2. Cerrar Gripper
        # robot.cerrar_gripper()
        
        # 3. Posición Final (Place)
        # "1.0 0 0"
        # robot.mover_brazo(1.40, 0.0, 0.0, tiempo_espera=3.0)
        
        # 4. Abrir Gripper
        # robot.abrir_gripper()
        
        # 5. Volver a Home (Opcional)
        robot.ir_a_home()
        
        print("\n--- SECUENCIA COMPLETADA CON ÉXITO ---\n")

    except KeyboardInterrupt:
        print("Cancelado por usuario.")
    finally:
        # Apagar suavemente
        if robot.arduino_connected:
            # Opcional: Relajar gripper al salir
            robot.arduino_port.write(b'0 0 0 90\n')
            robot.arduino_port.close()
        
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()