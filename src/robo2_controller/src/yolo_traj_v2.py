#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String  # <--- AGREGADO STRING
from geometry_msgs.msg import Point 
import serial
import time
import threading

class YoloSmartPickNode(Node):
    def __init__(self):
        super().__init__('yolo_smart_pick_node')
        
        # ════════ CONFIGURACIÓN DE ROS2 ════════
        self.arm_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        
        # Suscriptor a POSICIÓN YOLO
        self.yolo_sub = self.create_subscription(
            Point, 
            '/yolo/object_position', 
            self.yolo_callback, 
            10
        )

        # Suscriptor a CLASE YOLO (NOMBRE DEL OBJETO) <--- NUEVO
        self.class_sub = self.create_subscription(
            String,
            '/yolo/object_class',
            self.class_callback,
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
        self.target_class = "desconocido"  # <--- Variable para guardar el nombre
        
        # ════════ CALIBRACIÓN DE CÁMARA (EJE Y - Profundidad) ════════
        self.CAM_Y_MIN = 158.8   # Cerca
        self.CAM_Y_MAX = 124.8  # Lejos
        
        # Rangos Motores Y (M2, M3)
        self.M2_MIN_Y = -0.70   # Para Y=70.4
        self.M3_MIN_Y = 1.50
        
        self.M2_MAX_Y = -1.0   # Para Y=145.0
        self.M3_MAX_Y = 0.60

        # ════════ CALIBRACIÓN DE CÁMARA (EJE X - Base M1) ════════
        self.CAM_X_MIN = 104.0  # Izquierda
        self.CAM_X_MAX = 92.0  # Derecha
        
        # Rangos Motor X (M1)
        self.M1_LEFT = -0.43    # Para X=139.0
        self.M1_RIGHT = 0.43    # Para X=261.6

        self.get_logger().info('👁️ Robot Clasificador Vision Pick & Place Iniciado...')

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

    # ════════ CALLBACKS DE YOLO ════════
    def yolo_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.object_detected = True

    def class_callback(self, msg):
        """Actualiza la clase detectada en tiempo real"""
        self.target_class = msg.data
        # Opcional: imprimir qué ve
        # self.get_logger().info(f"Veo un: {self.target_class}")

    # ════════ MATEMÁTICA DE INTERPOLACIÓN (MAPEO) ════════
    def map_value(self, x, in_min, in_max, out_min, out_max):
        if x < in_min: x = in_min
        if x > in_max: x = in_max
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def calcular_cinematica_vision(self, x_pixel, y_pixel):
        # 1. Calcular Base (M1) usando X
        q1_target = self.map_value(x_pixel, self.CAM_X_MIN, self.CAM_X_MAX, self.M1_LEFT, self.M1_RIGHT)
        
        # 2. Calcular Brazo (M2, M3) usando Y
        q2_target = self.map_value(y_pixel, self.CAM_Y_MIN, self.CAM_Y_MAX, self.M2_MIN_Y, self.M2_MAX_Y)
        q3_target = self.map_value(y_pixel, self.CAM_Y_MIN, self.CAM_Y_MAX, self.M3_MIN_Y, self.M3_MAX_Y)
        
        return q1_target, q2_target, q3_target

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
        time.sleep(0.5)

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
        # self.abrir_gripper() # Opcional al inicio
        
        while rclpy.ok():
            if not self.object_detected:
                self.get_logger().info("🔎 Buscando objeto...", once=True)
                time.sleep(0.5)
                continue
            
            # --- FOTO INSTANTÁNEA ---
            x_detectado = self.target_x
            y_detectado = self.target_y
            
            # Guardamos la clase detectada en ese momento (convertimos a minúscula por si acaso)
            clase_objeto = str(self.target_class).lower() 
            
            if y_detectado < 50 or y_detectado > 160:
                self.object_detected = False
                continue

            self.get_logger().info(f"✅ DETECTADO: {clase_objeto.upper()} en X={x_detectado:.1f}")
            
            # 2. CALCULAR CINEMÁTICA
            q1_target, q2_target, q3_target = self.calcular_cinematica_vision(x_detectado, y_detectado)
            
            # 3.1. ALINEAR BASE
            self.mover_brazo_suave(q1_target, 0.0, 0.0, duracion=1.5)
            
            # 3.2. BAJAR
            self.mover_brazo_suave(q1_target, q2_target, q3_target, duracion=2.5)
            
            # 4. AGARRAR
            self.cerrar_gripper()
            
            # 5. SUBIR (Seguridad)
            self.get_logger().info("⬆️ Subiendo carga...")
            self.mover_brazo_suave(q1_target, 0.0, 0.0, duracion=1.5)

            # ─────────────────────────────────────────────────────────
            # 6. CLASIFICACIÓN (LLEVAR A DESTINO SEGÚN CLASE)
            # ─────────────────────────────────────────────────────────
            destino_q1 = 1.0 # Valor por defecto (para placas u otros)

            # Nota: usa 'in' para que detecte "bateria", "Bateria", "una bateria", etc.
            if "bater" in clase_objeto: 
                self.get_logger().info(f"⚡ Es una BATERÍA -> Izquierda (1.57 rad)")
                destino_q1 = 1.57
            elif "motor" in clase_objeto:
                self.get_logger().info(f"⚙️ Es un MOTOR -> Derecha (-1.57 rad)")
                destino_q1 = -1.57
            else:
                self.get_logger().info(f"📦 Objeto '{clase_objeto}' -> Destino Estándar (1.0 rad)")
            
            self.get_logger().info("➡️ Llevando a zona de clasificación...")
            self.mover_brazo_suave(destino_q1, 0.0, 0.0, duracion=2.5)
            
            # 7. SOLTAR
            self.abrir_gripper()
            
            # 8. HOME
            self.ir_a_home()
            
            self.object_detected = False
            self.get_logger().info("--- CICLO TERMINADO ---")
            time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    robot = YoloSmartPickNode()
    
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