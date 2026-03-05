#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point # Asumiendo que /yolo/object_position usa Point
import serial
import time
import threading

# TIPO DE MENSAJE: 
# Si tu tópico '/yolo/object_position' usa otro tipo de mensaje (ej. Pose, Vector3),
# cámbialo aquí. Por el formato "x: ... y: ... z: ..." suele ser geometry_msgs/Point.
from geometry_msgs.msg import Point 

class YoloSmartPickNode(Node):
    def __init__(self):
        super().__init__('yolo_smart_pick_node')
        
        # ════════ CONFIGURACIÓN DE ROS2 ════════
        self.arm_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        
        # Suscriptor a YOLO
        self.yolo_sub = self.create_subscription(
            Point, 
            '/yolo/object_position', 
            self.yolo_callback, 
            10
        )
        
        # ════════ CONEXIÓN ARDUINO ════════
        self.arduino_connected = False
        self.arduino_port = None
        self.connect_arduino()
        
        # ════════ VARIABLES DE ESTADO ════════
        self.current_q1 = 0.0
        self.current_q2 = 0.0
        self.current_q3 = 0.0
        self.gripper_val = 90
        self.gazebo_gripper = 0.0
        
        # Variables de Visión
        self.object_detected = False
        self.target_y = 0.0
        self.target_x = 0.0
        
        # ════════ CALIBRACIÓN DE CÁMARA (TUS DATOS) ════════
        # Rango de Píxeles en Y
        self.CAM_Y_MIN = 70.4   # Posición más baja en imagen
        self.CAM_Y_MAX = 145.0  # Posición más alta en imagen
        
        # Rango de Motores correspondiente (Radianes)
        # Para Y = 70.4
        self.M2_MIN = -0.90
        self.M3_MIN = 1.70
        
        # Para Y = 145.0
        self.M2_MAX = -1.50
        self.M3_MAX = 0.60

        self.get_logger().info('👁️ Robot con Visión Artificial Iniciado. Esperando objeto...')

    def connect_arduino(self):
        possible_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        for port in possible_ports:
            try:
                self.arduino_port = serial.Serial(port, 115200, timeout=0.01, write_timeout=0)
                self.arduino_connected = True
                self.get_logger().info(f'✓ Arduino conectado en {port}')
                time.sleep(2)
                self.arduino_port.write(b'A\n')
                time.sleep(0.1)
                return
            except: continue
        self.get_logger().warning('⚠ Arduino NO conectado')

    def rad_to_deg(self, rad):
        return rad * 180.0 / 3.14159265359

    # ════════ CALLBACK DE YOLO ════════
    def yolo_callback(self, msg):
        # Guardamos la última posición vista
        self.target_x = msg.x
        self.target_y = msg.y
        self.object_detected = True
        # Nota: No ejecutamos el movimiento aquí para no saturar. 
        # Lo haremos en el loop principal.

    # ════════ MATEMÁTICA DE INTERPOLACIÓN (MAPEO) ════════
    def map_value(self, x, in_min, in_max, out_min, out_max):
        """Convierte un valor de un rango a otro (Regla de 3 inteligente)"""
        # 1. Limitar la entrada para no salirnos de rango
        if x < in_min: x = in_min
        if x > in_max: x = in_max
        
        # 2. Calcular proporción
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def calcular_pose_desde_y(self, y_pixel):
        """Calcula q2 y q3 basándose en el pixel Y"""
        
        # Interpolamos M2
        q2_target = self.map_value(y_pixel, self.CAM_Y_MIN, self.CAM_Y_MAX, self.M2_MIN, self.M2_MAX)
        
        # Interpolamos M3
        q3_target = self.map_value(y_pixel, self.CAM_Y_MIN, self.CAM_Y_MAX, self.M3_MIN, self.M3_MAX)
        
        self.get_logger().info(f"📐 Calculando para Y={y_pixel:.1f}: q2={q2_target:.2f}, q3={q3_target:.2f}")
        return q2_target, q3_target

    # ════════ COMUNICACIÓN Y MOVIMIENTO ════════
    def enviar_comando_rapido(self, q1, q2, q3, grip_val):
        # Gazebo
        msg_arm = Float64MultiArray()
        msg_arm.data = [q1, q2, q3]
        self.arm_pub.publish(msg_arm)
        msg_grip = Float64MultiArray()
        msg_grip.data = [self.gazebo_gripper]
        self.gripper_pub.publish(msg_grip)
        
        # Arduino
        if self.arduino_connected:
            try:
                m1 = self.rad_to_deg(q1)
                m2 = self.rad_to_deg(q2)
                m3 = self.rad_to_deg(q3)
                cmd = f"{m1:.2f} {m2:.2f} {m3:.2f} {int(grip_val)}\n"
                self.arduino_port.write(cmd.encode())
            except: pass

    def mover_brazo_suave(self, target_q1, target_q2, target_q3, duracion=3.0):
        self.get_logger().info(f"📍 Moviendo a objetivo...")
        diff_q1 = target_q1 - self.current_q1
        diff_q2 = target_q2 - self.current_q2
        diff_q3 = target_q3 - self.current_q3
        
        frecuencia = 30.0 
        pasos_totales = int(duracion * frecuencia)
        if pasos_totales < 1: pasos_totales = 1
        tiempo_por_paso = 1.0 / frecuencia
        
        start_time = time.perf_counter()
        
        for i in range(1, pasos_totales + 1):
            progreso = i / pasos_totales
            nq1 = self.current_q1 + (diff_q1 * progreso)
            nq2 = self.current_q2 + (diff_q2 * progreso)
            nq3 = self.current_q3 + (diff_q3 * progreso)
            
            self.enviar_comando_rapido(nq1, nq2, nq3, self.gripper_val)
            
            target_time = start_time + (i * tiempo_por_paso)
            error = target_time - time.perf_counter()
            if error > 0: time.sleep(error)
            
        self.current_q1 = target_q1
        self.current_q2 = target_q2
        self.current_q3 = target_q3
        self.enviar_comando_rapido(target_q1, target_q2, target_q3, self.gripper_val)
        time.sleep(0.5) # Estabilizar

    # ════════ ACCIONES GRIPPER ════════
    def cerrar_gripper(self):
        self.get_logger().info("✊ Cerrando Gripper")
        self.gripper_val = 110
        self.gazebo_gripper = 0.0
        self.enviar_comando_rapido(self.current_q1, self.current_q2, self.current_q3, self.gripper_val)
        time.sleep(1.0)

    def abrir_gripper(self):
        self.get_logger().info("✋ Abriendo Gripper")
        self.gripper_val = 80
        self.gazebo_gripper = 0.19
        self.enviar_comando_rapido(self.current_q1, self.current_q2, self.current_q3, self.gripper_val)
        time.sleep(1.0)
        self.gripper_val = 90
        self.enviar_comando_rapido(self.current_q1, self.current_q2, self.current_q3, self.gripper_val)
        time.sleep(0.5)

    def ir_a_home(self):
        self.get_logger().info("🏠 Yendo a HOME")
        self.mover_brazo_suave(0.0, 0.0, 0.0, duracion=3.0)

    # ════════ LÓGICA DE DECISIÓN (CEREBRO) ════════
    def ejecutar_ciclo_autonomo(self):
        # 1. Asegurar inicio
        # self.abrir_gripper()
        
        while rclpy.ok():
            # Esperar a ver un objeto
            if not self.object_detected:
                self.get_logger().info("🔎 Buscando objeto...", once=True)
                time.sleep(0.5)
                continue
            
            # Objeto detectado. Leemos su Y actual.
            y_detectado = self.target_y
            
            # Filtro básico: ¿Es un Y válido para nosotros?
            if y_detectado < 50 or y_detectado > 160:
                self.get_logger().warning(f"Objeto fuera de rango alcanzable (Y={y_detectado:.1f})")
                self.object_detected = False
                continue
            
            self.get_logger().info(f"✅ ¡OBJETO FIJADO EN Y={y_detectado:.1f}!")
            
            # 2. CALCULAR CINEMÁTICA INVERSA BASADA EN VISIÓN
            target_q2, target_q3 = self.calcular_pose_desde_y(y_detectado)
            
            # 3. IR A BUSCARLO (Movimiento suave)
            # Mantenemos q1 en 0.0 como pediste
            self.mover_brazo_suave(0.0, target_q2, target_q3, duracion=3.5)
            
            # 4. AGARRAR
            self.cerrar_gripper()
            
            # 5. LLEVAR A DESTINO (1.0, 0, 0)
            self.mover_brazo_suave(1.0, 0.0, 0.0, duracion=4.0)
            
            # 6. SOLTAR
            self.abrir_gripper()
            
            # 7. VOLVER A HOME Y REINICIAR
            self.ir_a_home()
            
            # Resetear detección para esperar el siguiente
            self.object_detected = False
            self.get_logger().info("--- CICLO TERMINADO, LISTO PARA EL SIGUIENTE ---")
            time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    robot = YoloSmartPickNode()
    
    # Hilo para escuchar a YOLO en segundo plano
    spinner = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    spinner.start()

    try:
        robot.ejecutar_ciclo_autonomo()
    except KeyboardInterrupt:
        print("Apagando...")
    finally:
        if robot.arduino_connected: robot.arduino_port.close()
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()