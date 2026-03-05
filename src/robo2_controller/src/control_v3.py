#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Point # <--- TIPO DE MENSAJE CORRECTO
import tkinter as tk
from tkinter import ttk
import threading
import serial
import time
import math
import numpy as np
from collections import deque

# ═══════════════════════════════════════════════════════
# 1. MATEMÁTICA Y FÍSICA (CALIBRADO)
# ═══════════════════════════════════════════════════════
class DynamicsModel:
    def __init__(self):
        self.SCALE = 0.0000015 

    def compute_torque(self, q, dq, q_d, dq_d, ddq_d, Kp, Ki, Kd, e_integral):
        q1, q2, q3 = q
        dq1, dq2, dq3 = dq
        c2 = np.cos(q2); s2 = np.sin(q2); c3 = np.cos(q3); s3 = np.sin(q3)
        c2q2 = np.cos(2*q2); s2q2 = np.sin(2*q2); c2q3 = np.cos(2*q3); s2q3 = np.sin(2*q3)
        c23 = np.cos(q2 - q3); s23 = np.sin(q2 - q3)
        
        d11 = 6.96*s2q2 + 0.131*s2q3 - 289.0*c2*c3 + 121.0*c2q2 + 71.0*c2q3 + 652.0
        D = np.array([[d11, 183.0*c2, 183.0*c2], [183.0*c2, 766.0, -144.0*c23], [183.0*c2, -144.0*c23, 200.0]]) * self.SCALE

        c11 = dq3*(144.0*c2*s3 - 71.0*c3*s3 + 0.262*c2q3 - 0.131) - dq2*(121.0*c2*s2 - 144.0*c3*s2 - 13.9*c2q2 + 6.96)
        c12 = 183.0*dq2*c2 - 91.3*dq3*c2 - dq1*(121.0*c2*s2 - 144.0*c3*s2 - 13.9*c2q2 + 6.96)
        c13 = -91.3*dq2*c2 + dq1*(144.0*c2*s3 - 71.0*c3*s3 + 0.262*c2q3 - 0.131)
        c21 = 91.3*dq3*c2 + dq1*(121.0*c2*s2 - 144.0*c3*s2 - 13.9*c2q2 + 6.96)
        c23 = 91.3*dq1*c2 - 144.0*dq3*s23
        c31 = -91.3*dq2*c2 - dq1*(144.0*c2*s3 - 71.0*c3*s3 + 0.262*c2q3 - 0.131)
        c32 = 144.0*dq2*s23 - 91.3*dq1*c2
        C = np.array([[c11, c12, c13], [c21, 0, c23], [c31, c32, 0]]) * self.SCALE

        g1 = 0; g2 = 6050*c2 + 1110*s2
        g3 = -(268*s3 - 8320*c3) 
        G = np.array([g1, g2, g3]) * self.SCALE

        e = q_d - q; de = dq_d - dq
        aux = ddq_d + (Kd @ de) + (Kp @ e) + (Ki @ e_integral)
        return D @ aux + C @ dq + G

class TrajectoryGen:
    def __init__(self, omega_n=3.0, zeta=1.0):
        self.omega_n = omega_n; self.zeta = zeta; self.dt = 0.02
        self.q = np.zeros(3); self.dq = np.zeros(3); self.ddq = np.zeros(3)
    def update(self, target):
        error = target - self.q
        self.ddq = (self.omega_n**2 * error) - (2*self.zeta*self.omega_n*self.dq)
        self.dq += self.ddq * self.dt; self.q += self.dq * self.dt
        return self.q, self.dq, self.ddq

# ═══════════════════════════════════════════════════════
# 2. CONTROLADOR LÓGICO (YOLO + ARDUINO)
# ═══════════════════════════════════════════════════════
class YoloRobotController(Node):
    def __init__(self):
        super().__init__('yolo_robot_controller')
        
        # --- SUSCRIPCIONES CORREGIDAS ---
        # Ahora usamos Point para la posición
        self.sub_pos = self.create_subscription(Point, '/yolo/object_position', self.yolo_pos_callback, 10)
        self.sub_class = self.create_subscription(String, '/yolo/object_class', self.yolo_class_callback, 10)
        
        self.arm_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        
        self.arduino_connected = False; self.arduino_port = None
        self.connect_arduino()
        
        self.dynamics = DynamicsModel()
        self.traj_gen = TrajectoryGen()
        
        # PARÁMETROS CALIBRADOS
        self.Kp = np.diag([350.0, 300.0, 450.0]) 
        self.Kd = np.diag([10.0, 10.0, 10.0]) 
        self.Ki = np.diag([25.0, 115.0, 90.0]) 
        self.e_integral = np.zeros(3)
        
        self.q_curr = np.zeros(3); self.dq_curr = np.zeros(3)
        self.q_target = np.zeros(3)
        self.gripper_val = 90
        
        self.initialized_pose = False
        
        # Lógica Automata
        self.state = "IDLE"
        self.state_timer = 0
        self.detected_obj_pos = [0.0, 0.0]
        self.detected_class = ""
        self.target_bin_angle = 0.0 
        
        self.create_timer(0.02, self.control_loop) # 50Hz Control
        self.create_timer(0.1, self.logic_loop)    # 10Hz Lógica
        
        if self.arduino_connected: 
            threading.Thread(target=self.serial_read_loop, daemon=True).start()

    def connect_arduino(self):
        ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', 'COM3']
        for p in ports:
            try:
                self.arduino_port = serial.Serial(p, 115200, timeout=0.05)
                time.sleep(2.5)
                self.arduino_connected = True
                print(f"✅ Conectado en {p}")
                return
            except: pass

    def serial_read_loop(self):
        while self.arduino_connected:
            try:
                if self.arduino_port.in_waiting:
                    line = self.arduino_port.readline().decode().strip()
                    if line.startswith("FB:"):
                        parts = line[3:].split(',')
                        if len(parts)>=3:
                            deg = [float(p) for p in parts]
                            self.q_curr = np.deg2rad(np.array(deg))
                            if not self.initialized_pose:
                                self.q_target = self.q_curr.copy()
                                self.traj_gen.q = self.q_curr.copy()
                                self.initialized_pose = True
            except: pass
            time.sleep(0.002)

    # --- CALLBACKS YOLO CORREGIDOS ---
    def yolo_pos_callback(self, msg):
        # msg es geometry_msgs/msg/Point (tiene x, y, z)
        if self.state == "IDLE":
            self.detected_obj_pos = [msg.x, msg.y]
            self.state = "DETECTED" # Iniciar secuencia

    def yolo_class_callback(self, msg):
        # msg es std_msgs/msg/String (tiene data)
        if self.state == "IDLE" or self.state == "DETECTED":
            self.detected_class = msg.data
            # Clasificación: Izquierda o Derecha
            if "Bateria" in self.detected_class or "Placa" in self.detected_class:
                self.target_bin_angle = np.deg2rad(-90)
            else:
                self.target_bin_angle = np.deg2rad(90)

    # --- KINEMATICS (TUS PUNTOS CALIBRADOS) ---
    def get_target_from_vision(self, x_pixel, y_pixel):
        # 1. Base (M1) usando Y Pixel
        # Y: 16.7 (Izquierda, -30.3) -> 223.3 (Derecha, 27.3)
        m1_deg = np.interp(y_pixel, [16.7, 223.3], [-30.3, 27.3])
        
        # 2. Brazo (M2, M3) usando X Pixel (Distancia)
        # Interpolación con tus 5 puntos de calibración
        pixel_refs = np.linspace(35.1, 161.35, 5) 
        
        # Ángulos correspondientes (De Cerca a Lejos)
        m2_refs = np.array([-57.0, -71.7, -74.0, -78.8, -90.0])
        m3_refs = np.array([90.0, 56.7, 52.4, 50.0, 25.0])
        
        m2_deg = np.interp(x_pixel, pixel_refs, m2_refs)
        m3_deg = np.interp(x_pixel, pixel_refs, m3_refs)
        
        return np.deg2rad([m1_deg, m2_deg, m3_deg])

    # --- MÁQUINA DE ESTADOS ---
    def logic_loop(self):
        if not self.initialized_pose: return

        error_pos = np.linalg.norm(self.q_target - self.q_curr)
        
        if self.state == "DETECTED":
            print(f"🎯 Objeto: {self.detected_class} en ({self.detected_obj_pos[0]:.1f}, {self.detected_obj_pos[1]:.1f})")
            # Calcular target para tocar el suelo
            self.q_target = self.get_target_from_vision(self.detected_obj_pos[0], self.detected_obj_pos[1])
            self.state = "APPROACH"
            
        elif self.state == "APPROACH":
            if error_pos < 0.08: # Umbral de llegada (rad)
                print("🔒 Agarrando...")
                self.gripper_val = 120 # Cerrar
                self.state = "GRIPPING"
                self.state_timer = time.time()
                
        elif self.state == "GRIPPING":
            if time.time() - self.state_timer > 1.2:
                print("⬆️ Levantando...")
                # Posición segura arriba
                self.q_target[1] = np.deg2rad(-45)
                self.q_target[2] = np.deg2rad(45)
                self.state = "LIFTING"
                
        elif self.state == "LIFTING":
            if error_pos < 0.08:
                print(f"🔄 Clasificando hacia {np.rad2deg(self.target_bin_angle):.1f}°...")
                self.q_target[0] = self.target_bin_angle
                self.state = "MOVING_BIN"
                
        elif self.state == "MOVING_BIN":
            if error_pos < 0.08:
                print("🔓 Soltando...")
                self.gripper_val = 70 # Abrir
                self.state = "DROPPING"
                self.state_timer = time.time()
                
        elif self.state == "DROPPING":
            if time.time() - self.state_timer > 1.0:
                print("🏠 Volviendo a casa...")
                # Home
                self.q_target = np.deg2rad([0, -20, 20])
                self.state = "HOMING"
                
        elif self.state == "HOMING":
            if error_pos < 0.08:
                self.state = "IDLE"
                print("👀 Esperando...")

    # --- CONTROL DE BAJO NIVEL ---
    def control_loop(self):
        if not self.arduino_connected or not self.initialized_pose: return
        
        # Trayectoria suave hacia el objetivo dictado por la lógica
        q_d, dq_d, ddq_d = self.traj_gen.update(self.q_target)
        
        dt = 0.02
        self.e_integral += (q_d - self.q_curr) * dt
        self.e_integral = np.clip(self.e_integral, -1.0, 1.0)
        
        tau = self.dynamics.compute_torque(self.q_curr, self.dq_curr, q_d, dq_d, ddq_d, self.Kp, self.Ki, self.Kd, self.e_integral)
        
        # Fricción personalizada (M3 reforzado)
        u_static = np.array([6.0, 6.0, 6.3]) * np.sign(dq_d)
        u_visc = 5.0 * dq_d
        
        u = (tau * 80.0) + u_static + u_visc
        u = np.clip(u, -12.0, 12.0)
        
        try: 
            self.arduino_port.write(f"CMD:{u[0]:.2f},{u[1]:.2f},{u[2]:.2f},{int(self.gripper_val)}\n".encode())
        except: pass

        msg = Float64MultiArray(); msg.data = [q_d[0], q_d[1], q_d[2]]
        self.arm_pub.publish(msg)

# ═══════════════════════════════════════════════════════
# MONITOR SIMPLE
# ═══════════════════════════════════════════════════════
class RobotMonitor:
    def __init__(self, controller):
        self.c = controller
        self.root = tk.Tk()
        self.root.title("YOLO PICK & PLACE")
        self.root.geometry("400x250")
        self.root.config(bg='#13141f')
        
        tk.Label(self.root, text="ESTADO ROBOT", fg='white', bg='#13141f', font=("Arial", 14, "bold")).pack(pady=15)
        self.lbl_state = tk.Label(self.root, text="--", fg='#7aa2f7', bg='#1f202e', font=("Arial", 18), width=20)
        self.lbl_state.pack(pady=10)
        
        self.lbl_info = tk.Label(self.root, text="...", fg='gray', bg='#13141f', font=("Consolas", 10))
        self.lbl_info.pack(pady=5)
        
        self.update()
        
    def update(self):
        self.lbl_state.config(text=self.c.state)
        if self.c.state != "IDLE":
            self.lbl_info.config(text=f"Obj: {self.c.detected_class}\nPos: {self.c.detected_obj_pos}")
        self.root.after(100, self.update)
    
    def run(self): self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    controller = YoloRobotController()
    t = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    t.start()
    gui = RobotMonitor(controller)
    try: gui.run()
    except KeyboardInterrupt: pass
    finally:
        if controller.arduino_connected: 
            controller.arduino_port.write(b"CMD:0,0,0,90\n")
            controller.arduino_port.close()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()