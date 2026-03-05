#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import serial
import time
import math
import numpy as np
from collections import deque
from enum import Enum

# ═══════════════════════════════════════════════════════
# ESTADOS PARA PICK & PLACE
# ═══════════════════════════════════════════════════════
class PickPlaceState(Enum):
    DISABLED = 0
    IDLE = 1
    MOVING_TO_OBJECT = 2
    LOWERING = 3
    CLOSING_GRIPPER = 4
    LIFTING = 5
    ROTATING_TO_BIN = 6
    OPENING_GRIPPER = 7
    RETURNING_HOME = 8

# ═══════════════════════════════════════════════════════
# TEMAS (DARK & LIGHT)
# ═══════════════════════════════════════════════════════
THEMES = {
    'dark': {
        'bg_main': '#13141f',
        'bg_panel': '#1f202e',
        'fg_text': '#a9b1d6',
        'accent_1': '#7aa2f7',
        'accent_2': '#bb9af7',
        'target_color': '#f7768e',
        'real_color': '#9ece6a',
        'error_color': '#ff9e64',
        'grid_color': '#2f334d',
        'grid_sub': '#24283b',
        'btn_close': '#f7768e',
        'btn_open': '#9ece6a',
        'btn_reset': '#e0af68',
        'btn_test': '#7dcfff',
        'btn_power': '#9ece6a',
        'btn_pickplace': '#bb9af7',
        'ps4_active': '#9ece6a',
        'ps4_inactive': '#3b4261',
        'fg_entry': '#7aa2f7',
        'bg_entry': '#13141f'
    },
    'light': {
        'bg_main': '#f2f4f8',
        'bg_panel': '#ffffff',
        'fg_text': '#2c3e50',
        'accent_1': '#2980b9',
        'accent_2': '#8e44ad',
        'target_color': '#e74c3c',
        'real_color': '#27ae60',
        'error_color': '#f39c12',
        'grid_color': '#dcdde1',
        'grid_sub': '#f1f2f6',
        'btn_close': '#ff6b6b',
        'btn_open': '#51cf66',
        'btn_reset': '#fcc419',
        'btn_test': '#339af0',
        'btn_power': '#51cf66',
        'btn_pickplace': '#8e44ad',
        'ps4_active': '#2ecc71',
        'ps4_inactive': '#bdc3c7',
        'fg_entry': '#2980b9',
        'bg_entry': '#ecf0f1'
    }
}

# ═══════════════════════════════════════════════════════
# GRÁFICAS
# ═══════════════════════════════════════════════════════
class GraphWidget(tk.Canvas):
    def __init__(self, parent, title, width=280, height=250, theme=THEMES['dark']):
        super().__init__(parent, width=width, height=height, highlightthickness=1)
        self.title_text = title
        self.theme = theme
        
        self.max_points = 100
        self.target_data = deque([0.0]*self.max_points, maxlen=self.max_points)
        self.real_data = deque([0.0]*self.max_points, maxlen=self.max_points)
        
        self.min_y = -95
        self.max_y = 95
        
        self.bind("<Configure>", self.on_resize)
        self.apply_theme_colors(theme)

    def apply_theme_colors(self, theme):
        self.theme = theme
        self.config(bg=theme['bg_panel'], highlightbackground=theme['grid_color'])
        self.draw_graph()

    def on_resize(self, event):
        self.width = event.width
        self.height = event.height
        self.draw_graph()
        
    def add_point(self, target, real):
        self.target_data.append(target)
        self.real_data.append(real)
        self.draw_graph()
        
    def draw_graph(self):
        self.delete("all")
        
        w = self.winfo_width()
        h = self.winfo_height()
        if w < 10: w = int(self['width'])
        if h < 10: h = int(self['height'])

        margin_left = 40
        margin_right = 10
        margin_top = 25
        margin_bottom = 20
        
        graph_w = w - margin_left - margin_right
        graph_h = h - margin_top - margin_bottom
        
        if graph_w <= 0 or graph_h <= 0: return

        num_v_lines = 10
        step_v = graph_w / num_v_lines
        for i in range(1, num_v_lines):
            x = margin_left + i * step_v
            self.create_line(x, margin_top, x, h-margin_bottom, fill=self.theme['grid_sub'], width=1)

        y_zero = margin_top + (graph_h / 2)
        y_max = margin_top
        y_min = h - margin_bottom
        
        y_mid_upper = margin_top + (graph_h * 0.25)
        y_mid_lower = margin_top + (graph_h * 0.75)
        
        self.create_line(margin_left, y_mid_upper, w-margin_right, y_mid_upper, fill=self.theme['grid_sub'], dash=(2, 4))
        self.create_line(margin_left, y_mid_lower, w-margin_right, y_mid_lower, fill=self.theme['grid_sub'], dash=(2, 4))
        self.create_line(margin_left, y_zero, w-margin_right, y_zero, fill=self.theme['grid_color'], width=1)
        self.create_line(margin_left, y_max, w-margin_right, y_max, fill=self.theme['grid_color'], width=1)
        self.create_line(margin_left, y_min, w-margin_right, y_min, fill=self.theme['grid_color'], width=1)
        
        font_axis = ("Consolas", 8)
        self.create_text(margin_left-5, y_max, text="90°", fill=self.theme['fg_text'], anchor="e", font=font_axis)
        self.create_text(margin_left-5, y_mid_upper, text="45°", fill=self.theme['fg_text'], anchor="e", font=font_axis)
        self.create_text(margin_left-5, y_zero, text="0°", fill=self.theme['fg_text'], anchor="e", font=font_axis)
        self.create_text(margin_left-5, y_mid_lower, text="-45°", fill=self.theme['fg_text'], anchor="e", font=font_axis)
        self.create_text(margin_left-5, y_min, text="-90°", fill=self.theme['fg_text'], anchor="e", font=font_axis)
        
        self.create_text(10, 5, text=self.title_text, fill=self.theme['accent_1'], anchor="nw", font=("Arial", 10, "bold"))
        
        self.draw_polyline(self.target_data, self.theme['target_color'], graph_w, graph_h, margin_left, margin_top)
        self.draw_polyline(self.real_data, self.theme['real_color'], graph_w, graph_h, margin_left, margin_top)

    def draw_polyline(self, data, color, gw, gh, ml, mt):
        points = []
        range_y = self.max_y - self.min_y
        step_x = gw / (self.max_points - 1)
        
        for i, val in enumerate(data):
            x = ml + (i * step_x)
            normalized_y = 1.0 - ((val - self.min_y) / range_y)
            y = mt + (normalized_y * gh)
            y = max(mt, min(mt + gh, y))
            points.append(x)
            points.append(y)
            
        if len(points) >= 4:
            self.create_line(points, fill=color, width=2, smooth=True)

# ═══════════════════════════════════════════════════════
# MODELO DINÁMICO
# ═══════════════════════════════════════════════════════
class DynamicsModel:
    def __init__(self):
        self.SCALE = 0.0000015 # Tu calibración validada

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
        g3 = -(268*s3 - 8320*c3) # Signo corregido
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
# CONTROLADOR CON PICK & PLACE SUAVE (S-CURVE)
# ═══════════════════════════════════════════════════════
class RobotGUIControllerDual(Node):
    def __init__(self):
        super().__init__('robot_gui_controller_dual')
        self.arm_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # SUSCRIPCIONES YOLO
        self.yolo_position_sub = self.create_subscription(Point, '/yolo/object_position', self.yolo_position_callback, 10)
        self.yolo_class_sub = self.create_subscription(String, '/yolo/object_class', self.yolo_class_callback, 10)
        
        # Variables YOLO
        self.object_x = None; self.object_y = None; self.object_class = None
        self.pickplace_state = PickPlaceState.DISABLED
        
        # --- VARIABLES PARA MOVIMIENTO SUAVE (INTERPOLACIÓN) ---
        self.move_start_time = 0
        self.move_duration = 0
        self.pose_start = np.zeros(3)
        self.pose_end = np.zeros(3)
        self.is_moving_smoothly = False
        
        # Calibración YOLO
        self.yolo_x_far = 167.6; self.yolo_x_near = 53.4
        self.yolo_y_left = 24.8; self.yolo_y_right = 227.1
        self.m1_left_limit = -30.3; self.m1_right_limit = 27.3
        
        # Puntos de suelo calibrados
        self.ground_points = np.array([[-90.0, 25.0], [-78.8, 50.0], [-74.0, 52.4], [-71.7, 56.7], [-57.0, 90.0]])
        self.ground_distances = np.linspace(1.0, 0.0, len(self.ground_points))
        
        self.bin_positions = {'Batería': 90.0, 'Motor': -90.0, 'Placa': 90.0}
        self.target_angles_pp = np.zeros(3)
        
        self.arduino_connected = False; self.arduino_port = None
        self.connect_arduino()
        
        self.dynamics = DynamicsModel()
        self.traj_gen = TrajectoryGen()
        
        # TUNING
        self.Kp = np.diag([350.0, 300.0, 450.0]) 
        self.Kd = np.diag([10.0, 10.0, 10.0]) 
        self.Ki = np.diag([45.0, 115.0, 120.0]) 
        self.e_integral = np.zeros(3)
        
        self.q_curr = np.zeros(3); self.dq_curr = np.zeros(3)
        self.q_target_raw = np.zeros(3) # Objetivo instantáneo (el que sigue la curva suave)
        self.q_target_final = np.zeros(3) # Objetivo final real
        self.gripper_val = 90
        
        self.motors_enabled = False; self.initialized_pose = False
        self.test_mode = False; self.test_start_time = 0; self.test_freq = 0.25

        self.ps4_enabled = False; self.joy_vel = np.zeros(3); self.prev_buttons = [0]*20
        self.base_to_base=0.0; self.base_to_arm=0.0; self.base_to_arm2=0.0
        self.M1_deg = 0.0; self.M2_deg = 0.0; self.M3_deg = 0.0; self.real_M1 = 0.0; self.real_M2 = 0.0; self.real_M3 = 0.0
        
        self.create_timer(0.05, self.ps4_update_loop)
        self.create_timer(0.02, self.control_loop)
        self.create_timer(0.1, self.pickplace_logic) # Lógica separada
        
        if self.arduino_connected: threading.Thread(target=self.serial_read_loop, daemon=True).start()

    # ═══════════════════════════════════════════════════════
    # FUNCIÓN CLAVE: START SMOOTH MOVE
    # ═══════════════════════════════════════════════════════
    def start_smooth_move(self, target_rads, duration):
        """Inicia una interpolación cosenoidal desde la posición actual hasta el objetivo"""
        self.pose_start = self.q_target_raw.copy() # Desde donde estoy ahora (setpoint)
        self.pose_end = np.array(target_rads)      # A donde voy
        self.move_start_time = time.time()
        self.move_duration = duration
        self.is_moving_smoothly = True

    # ═══════════════════════════════════════════════════════
    # CONTROL LOOP (50Hz) - GENERACIÓN DE PERFIL S
    # ═══════════════════════════════════════════════════════
    def control_loop(self):
        if not self.arduino_connected or not self.initialized_pose: return
        
        # 1. ACTUALIZAR OBJETIVO INSTANTÁNEO SI HAY MOVIMIENTO SUAVE
        if self.is_moving_smoothly:
            t = time.time() - self.move_start_time
            if t <= self.move_duration:
                # Curva Cosenoidal (S-Curve): Velocidad 0 al inicio y al final
                # k va de 0.0 a 1.0 suavemente
                k = (1 - math.cos(math.pi * t / self.move_duration)) / 2
                self.q_target_raw = self.pose_start + (self.pose_end - self.pose_start) * k
            else:
                self.q_target_raw = self.pose_end # Asegurar llegada
                self.is_moving_smoothly = False   # Terminar movimiento

        # Modo Test Senoidal
        if self.test_mode:
            t_sine = time.time() - self.test_start_time
            self.q_target_raw[0] = 0.4 * np.sin(2 * np.pi * self.test_freq * t_sine)
            self.q_target_raw[1] = 0.4 * np.sin(2 * np.pi * self.test_freq * t_sine + np.pi/2)
            self.q_target_raw[2] = 0.4 * np.sin(2 * np.pi * self.test_freq * t_sine)

        # 2. Control Dinámico (Tu código validado)
        q_d, dq_d, ddq_d = self.traj_gen.update(self.q_target_raw)
        
        # Actualizar variables para GUI
        self.base_to_base, self.base_to_arm, self.base_to_arm2 = q_d
        self.M1_deg = np.rad2deg(q_d[0]); self.M2_deg = np.rad2deg(q_d[1]); self.M3_deg = np.rad2deg(q_d[2])

        if self.motors_enabled:
            dt = 0.02
            self.e_integral += (q_d - self.q_curr) * dt
            self.e_integral = np.clip(self.e_integral, -1.0, 1.0)
            
            tau = self.dynamics.compute_torque(self.q_curr, self.dq_curr, q_d, dq_d, ddq_d, self.Kp, self.Ki, self.Kd, self.e_integral)
            
            u_static = np.array([6.0, 6.0, 6.3]) * np.sign(dq_d)
            u_visc = 5.0 * dq_d
            u = (tau * 80.0) + u_static + u_visc
            u = np.clip(u, -12.0, 12.0)
            
            try: self.arduino_port.write(f"CMD:{u[0]:.2f},{u[1]:.2f},{u[2]:.2f},{int(self.gripper_val)}\n".encode())
            except: pass
        elif not self.motors_enabled:
            try: self.arduino_port.write(f"CMD:0,0,0,{int(self.gripper_val)}\n".encode())
            except: pass
        
        msg = Float64MultiArray(); msg.data = [q_d[0], q_d[1], q_d[2]]; self.arm_pub.publish(msg)

    # ═══════════════════════════════════════════════════════
    # LÓGICA DE ESTADOS PICK & PLACE
    # ═══════════════════════════════════════════════════════
    def yolo_position_callback(self, msg):
        self.object_x = msg.x; self.object_y = msg.y
    
    def yolo_class_callback(self, msg):
        self.object_class = msg.data
        if (self.pickplace_state == PickPlaceState.IDLE and self.object_class in self.bin_positions and
            self.object_x is not None and self.object_y is not None):
            
            self.get_logger().info(f'✅ Detectado: {self.object_class}. Iniciando...')
            
            # Calcular ángulos objetivo
            self.target_angles_pp = self.yolo_to_robot_angles(self.object_x, self.object_y)
            
            # Cambiar a estado MOVIMIENTO
            self.pickplace_state = PickPlaceState.MOVING_TO_OBJECT
            
            # PASO 1: Girar M1 (2 segundos)
            target_1 = self.q_target_raw.copy()
            target_1[0] = self.target_angles_pp[0]
            self.start_smooth_move(target_1, 2.0)

    def yolo_to_robot_angles(self, x_yolo, y_yolo):
        # Interpolación Base M1
        m1 = np.interp(y_yolo, [self.yolo_y_left, self.yolo_y_right], [self.m1_left_limit, self.m1_right_limit])
        
        # Interpolación Brazo M2, M3
        dist_norm = np.interp(x_yolo, [self.yolo_x_near, self.yolo_x_far], [1.0, 0.0])
        m2 = np.interp(dist_norm, self.ground_distances, self.ground_points[:, 0])
        m3 = np.interp(dist_norm, self.ground_distances, self.ground_points[:, 1])
        
        # APLICAR LÍMITES DE SEGURIDAD (Tus datos)
        m2 = np.clip(m2, -90.0, -57.0) # No bajar más de -90, no subir más de -57 (cerca)
        m3 = np.clip(m3, 25.0, 90.0)   # Rango seguro M3
        
        return np.deg2rad([m1, m2, m3])

    def pickplace_logic(self):
        if self.pickplace_state == PickPlaceState.DISABLED or self.pickplace_state == PickPlaceState.IDLE: return
        
        # ESPERAR A QUE TERMINE EL MOVIMIENTO SUAVE
        if self.is_moving_smoothly: return 

        # LÓGICA SECUENCIAL (Solo avanza cuando terminó el movimiento anterior)
        if self.pickplace_state == PickPlaceState.MOVING_TO_OBJECT:
            # Ya giró M1, ahora BAJAR SUAVE (3.0 segundos)
            self.get_logger().info('⬇️ Bajando suavemente (3s)...')
            # PASO 2: Bajar M2 y M3 suavemente en 3 segundos
            target_down = np.array([self.q_target_raw[0], self.target_angles_pp[1], self.target_angles_pp[2]])
            self.start_smooth_move(target_down, 3.0) 
            self.pickplace_state = PickPlaceState.LOWERING
            
        elif self.pickplace_state == PickPlaceState.LOWERING:
            # Ya bajó, cerrar gripper
            self.get_logger().info('✊ Cerrando gripper...')
            self.gripper_val = 120
            time.sleep(3.0)
            self.pickplace_state = PickPlaceState.CLOSING_GRIPPER
            
        elif self.pickplace_state == PickPlaceState.CLOSING_GRIPPER:
            self.get_logger().info('⬆️ Levantando...')
            target_up = self.q_target_raw.copy()
            target_up[1] = np.deg2rad(-20.0); target_up[2] = np.deg2rad(10.0)
            self.start_smooth_move(target_up, 2.0)
            self.pickplace_state = PickPlaceState.LIFTING
            
        elif self.pickplace_state == PickPlaceState.LIFTING:
            bin_angle = self.bin_positions.get(self.object_class, 90.0)
            self.get_logger().info(f'🔄 Rotando a {bin_angle}°...')
            target_bin = self.q_target_raw.copy()
            target_bin[0] = np.deg2rad(bin_angle)
            self.start_smooth_move(target_bin, 2.0)
            self.pickplace_state = PickPlaceState.ROTATING_TO_BIN
            
        elif self.pickplace_state == PickPlaceState.ROTATING_TO_BIN:
            self.get_logger().info('✋ Soltando...')
            self.gripper_val = 70
            time.sleep(0.9)
            self.gripper_val = 90
            self.pickplace_state = PickPlaceState.OPENING_GRIPPER
            
        elif self.pickplace_state == PickPlaceState.OPENING_GRIPPER:
            self.get_logger().info('🏠 Home...')
            self.start_smooth_move(np.zeros(3), 3.0)
            self.pickplace_state = PickPlaceState.RETURNING_HOME
            
        elif self.pickplace_state == PickPlaceState.RETURNING_HOME:
            self.gripper_val = 90
            self.pickplace_state = PickPlaceState.IDLE
            self.get_logger().info('✅ Listo.')

    # ═══════════════════════════════════════════════════════
    # UTILS
    # ═══════════════════════════════════════════════════════
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
                            self.real_M1 = deg[0]; self.real_M2 = deg[1]; self.real_M3 = deg[2]
                            if not self.initialized_pose:
                                self.q_target_raw = self.q_curr.copy()
                                self.traj_gen.q = self.q_curr.copy()
                                self.initialized_pose = True
            except: pass
            time.sleep(0.002)

    def toggle_pickplace(self):
        if self.pickplace_state == PickPlaceState.DISABLED:
            self.pickplace_state = PickPlaceState.IDLE
            self.motors_enabled = True; return True
        else:
            self.pickplace_state = PickPlaceState.DISABLED; return False

    def toggle_power(self):
        self.motors_enabled = not self.motors_enabled
        if self.motors_enabled: self.q_target_raw = self.q_curr.copy(); self.traj_gen.q = self.q_curr.copy(); self.e_integral = np.zeros(3)
        return self.motors_enabled

    def toggle_test(self):
        self.test_mode = not self.test_mode
        if self.test_mode: self.test_start_time = time.time(); self.motors_enabled = True
        else: self.q_target_raw = np.zeros(3)
        return self.test_mode

    def update_target_deg(self, idx, deg): 
        if not self.test_mode and self.pickplace_state == PickPlaceState.DISABLED: 
            self.q_target_raw[idx] = np.deg2rad(deg)

    def update_from_input_deg(self, joint, deg_value):
        if self.test_mode or self.pickplace_state != PickPlaceState.DISABLED: return
        idx = {"base_to_base": 0, "base_to_arm": 1, "base_to_arm2": 2}[joint]
        self.q_target_raw[idx] = np.deg2rad(deg_value)

    def update_from_slider(self, joint, value):
        if self.test_mode or self.pickplace_state != PickPlaceState.DISABLED: return
        idx = {"base_to_base": 0, "base_to_arm": 1, "base_to_arm2": 2}[joint]
        self.q_target_raw[idx] = float(value)

    # ═══════════════════════════════════════════════════════
    #  JOYSTICK MODIFICADO (Control M3 y Gripper)
    # ═══════════════════════════════════════════════════════
    def joy_callback(self, msg):
        if self.ps4_enabled and not self.test_mode and self.pickplace_state == PickPlaceState.DISABLED:
            # Control M1 y M2 (Joysticks)
            self.joy_vel[0] = -msg.axes[3]*0.05
            self.joy_vel[1] = -msg.axes[1]*0.05
            
            # Control M3 (Botones Triangle/X) - INCREMENTAL
            step_m3 = 0.02
            if msg.buttons[2] == 1: # Triangle: Bajar
                self.q_target_raw[2] -= step_m3
            if msg.buttons[0] == 1: # X: Subir
                self.q_target_raw[2] += step_m3
            
            # Control Gripper (Botones Square/Circle)
            if msg.buttons[3] == 1 and self.prev_buttons[3] == 0: # Square: Cerrar
                self.gripper_val = 120
            if msg.buttons[1] == 1 and self.prev_buttons[1] == 0: # Circle: Abrir
                def sequence():
                    self.gripper_val = 70
                    time.sleep(0.7)
                    self.gripper_val = 90
                threading.Thread(target=sequence, daemon=True).start()
            
            self.prev_buttons = list(msg.buttons)
            
    def ps4_update_loop(self):
        if self.ps4_enabled and not self.test_mode and self.pickplace_state == PickPlaceState.DISABLED:
            # Integra velocidades de joy_vel y los cambios directos a q_target_raw
            self.q_target_raw = np.clip(self.q_target_raw + self.joy_vel, -1.57, 1.57)

    def reset_all(self):
        self.test_mode = False; self.q_target_raw = np.zeros(3); self.e_integral = np.zeros(3)
        self.pickplace_state = PickPlaceState.DISABLED
        if self.arduino_connected: self.arduino_port.write(b"RESET\n")

    def command_close_gripper(self): self.gripper_val = 120
    def command_open_gripper(self): 
        def seq(): self.gripper_val=70; time.sleep(0.6); self.gripper_val=90
        threading.Thread(target=seq, daemon=True).start()

# ═══════════════════════════════════════════════════════
# INTERFAZ GRÁFICA (TU CÓDIGO ORIGINAL)
# ═══════════════════════════════════════════════════════
THEMES = {
    'dark': { 'bg_main': '#13141f', 'bg_panel': '#1f202e', 'fg_text': '#a9b1d6', 'accent_1': '#7aa2f7', 'accent_2': '#bb9af7', 'target_color': '#f7768e', 'real_color': '#9ece6a', 'grid_color': '#2f334d', 'grid_sub': '#24283b', 'btn_close': '#f7768e', 'btn_open': '#9ece6a', 'btn_reset': '#e0af68', 'btn_test': '#7dcfff', 'btn_power': '#9ece6a', 'btn_pickplace': '#bb9af7', 'ps4_active': '#9ece6a', 'ps4_inactive': '#3b4261', 'fg_entry': '#7aa2f7', 'bg_entry': '#13141f' },
    'light': { 'bg_main': '#f2f4f8', 'bg_panel': '#ffffff', 'fg_text': '#2c3e50', 'accent_1': '#2980b9', 'accent_2': '#8e44ad', 'target_color': '#e74c3c', 'real_color': '#27ae60', 'grid_color': '#dcdde1', 'grid_sub': '#f1f2f6', 'btn_close': '#ff6b6b', 'btn_open': '#51cf66', 'btn_reset': '#fcc419', 'btn_test': '#339af0', 'btn_power': '#51cf66', 'btn_pickplace': '#8e44ad', 'ps4_active': '#2ecc71', 'ps4_inactive': '#bdc3c7', 'fg_entry': '#2980b9', 'bg_entry': '#ecf0f1' }
}

class RobotGUI:
    def __init__(self, controller):
        self.controller = controller
        self.root = tk.Tk()
        self.root.title("🤖 Robot Master Controller + Pick&Place")
        self.root.geometry("1000x900")
        self.current_theme_key = 'dark'; self.current_theme = THEMES['dark']
        self.is_user_interacting = False; self.updating_from_entry = False
        self.frames = []; self.labels = []; self.sliders = {}; self.entries = {}; self.graphs = {}
        self.setup_ui(); self.apply_theme(self.current_theme_key)
        self.sync_gui_with_ps4(); self.update_graphs_loop()

    def setup_ui(self):
        self.main_frame = tk.Frame(self.root); self.main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20); self.frames.append(self.main_frame)
        header_frame = tk.Frame(self.main_frame, height=50); header_frame.pack(fill=tk.X, pady=(0, 10)); self.frames.append(header_frame)
        self.title_lbl = tk.Label(header_frame, text="ROBOT MASTER CONTROL", font=('Segoe UI', 24, 'bold')); self.title_lbl.pack(side=tk.TOP, pady=5); self.labels.append(self.title_lbl)
        self.btn_theme = tk.Button(header_frame, text="🌗", font=('Arial', 14), command=self.toggle_theme, relief=tk.FLAT, cursor='hand2'); self.btn_theme.place(relx=1.0, y=0, anchor='ne')
        status_text = "🟢 CONECTADO: Gazebo + Arduino" if self.controller.arduino_connected else "🟡 MODO SIMULACIÓN"
        self.status_lbl = tk.Label(self.main_frame, text=status_text, font=('Segoe UI', 11)); self.status_lbl.pack(pady=(5, 10)); self.labels.append(self.status_lbl)
        
        ctrl_frame = tk.Frame(self.main_frame); ctrl_frame.pack(fill=tk.X, pady=10); self.frames.append(ctrl_frame)
        self.btn_power = tk.Button(ctrl_frame, text="⚡ ACTIVAR MOTORES (OFF)", command=self.toggle_power, font=('Segoe UI', 11, 'bold'), relief=tk.FLAT, width=22, height=2); self.btn_power.pack(side=tk.LEFT, padx=5)
        self.btn_test = tk.Button(ctrl_frame, text="🌊 TEST SENOIDAL", command=self.toggle_test, font=('Segoe UI', 11, 'bold'), relief=tk.FLAT, width=18, height=2); self.btn_test.pack(side=tk.LEFT, padx=5)
        self.btn_pickplace = tk.Button(ctrl_frame, text="🤖 PICK & PLACE", command=self.toggle_pickplace, font=('Segoe UI', 11, 'bold'), relief=tk.FLAT, width=18, height=2); self.btn_pickplace.pack(side=tk.LEFT, padx=5)
        
        self.ps4_frame = tk.Frame(self.main_frame, relief=tk.FLAT, bd=0); self.ps4_frame.pack(fill=tk.X, pady=10, ipady=10, ipadx=10); self.frames.append(self.ps4_frame)
        self.btn_ps4 = tk.Button(self.ps4_frame, text="🎮 CONTROL PS4: DESACTIVADO", command=self.toggle_ps4, font=('Segoe UI', 12, 'bold'), relief=tk.FLAT, width=30, height=2, cursor='hand2'); self.btn_ps4.pack(pady=5)

        self.sliders_container = tk.Frame(self.main_frame); self.sliders_container.pack(fill=tk.X, pady=10); self.frames.append(self.sliders_container)
        joints = [("Base (M1)", "base_to_base", -1.57, 1.57), ("Arm 1 (M2)", "base_to_arm", -1.57, 1.57), ("Arm 2 (M3)", "base_to_arm2", -1.57, 1.57)]
        for label, name, min_v, max_v in joints: self.create_control_row(self.sliders_container, label, name, min_v, max_v)

        self.btn_container = tk.Frame(self.main_frame); self.btn_container.pack(pady=15); self.frames.append(self.btn_container)
        self.btn_close = tk.Button(self.btn_container, text="✊ CERRAR GRIPPER", command=self.controller.command_close_gripper, font=('Segoe UI', 10, 'bold'), relief=tk.FLAT, width=18, height=2); self.btn_close.pack(side=tk.LEFT, padx=10)
        self.btn_open = tk.Button(self.btn_container, text="✋ ABRIR GRIPPER", command=self.controller.command_open_gripper, font=('Segoe UI', 10, 'bold'), relief=tk.FLAT, width=18, height=2); self.btn_open.pack(side=tk.LEFT, padx=10)
        self.btn_reset = tk.Button(self.btn_container, text="🔄 RESET HOME", command=self.reset_gui, font=('Segoe UI', 10, 'bold'), relief=tk.FLAT, width=18, height=2); self.btn_reset.pack(side=tk.LEFT, padx=10)

        self.graphs_frame = tk.Frame(self.main_frame); self.graphs_frame.pack(fill=tk.BOTH, expand=True, pady=10); self.frames.append(self.graphs_frame)
        self.g1 = GraphWidget(self.graphs_frame, "M1: Base", width=10, height=300); self.g1.pack(side=tk.LEFT, padx=5, fill=tk.BOTH, expand=True); self.graphs["M1"] = self.g1
        self.g2 = GraphWidget(self.graphs_frame, "M2: Arm 1", width=10, height=300); self.g2.pack(side=tk.LEFT, padx=5, fill=tk.BOTH, expand=True); self.graphs["M2"] = self.g2
        self.g3 = GraphWidget(self.graphs_frame, "M3: Arm 2", width=10, height=300); self.g3.pack(side=tk.LEFT, padx=5, fill=tk.BOTH, expand=True); self.graphs["M3"] = self.g3
        
        # (CÓDIGO CORREGIDO)
        self.legend_frame = tk.Frame(self.main_frame); self.legend_frame.pack(pady=5); self.frames.append(self.legend_frame)
        self.lbl_leg_target = tk.Label(self.legend_frame, text="⬤ Target", font=('Segoe UI', 10, 'bold'))
        self.lbl_leg_target.pack(side=tk.LEFT, padx=10)
        self.lbl_leg_real = tk.Label(self.legend_frame, text="⬤ Real", font=('Segoe UI', 10, 'bold'))
        self.lbl_leg_real.pack(side=tk.LEFT, padx=10)

    def create_control_row(self, parent, label, name, min_v, max_v):
        f = tk.Frame(parent, height=50); f.pack(fill=tk.X, pady=5, ipady=5); f.pack_propagate(False); self.frames.append(f)
        tk.Label(f, text=label, width=20, font=('Segoe UI', 12, 'bold')).pack(side=tk.LEFT, padx=20)
        s = tk.Scale(f, from_=min_v, to=max_v, resolution=0.001, orient=tk.HORIZONTAL, highlightthickness=0, showvalue=0, length=300, command=lambda v: self.on_slider_input(name, v))
        s.set(0.0); s.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=20)
        var = tk.StringVar(value="0.0"); e = tk.Entry(f, textvariable=var, width=6, font=('Consolas', 12, 'bold'), justify='center', relief=tk.FLAT)
        e.pack(side=tk.RIGHT, padx=20); e.bind('<Return>', lambda ev: self.on_entry_input(name, var))
        tk.Label(f, text="deg", font=('Segoe UI', 9)).pack(side=tk.RIGHT)
        self.sliders[name] = s; self.entries[name] = (e, var)
        s.bind("<Button-1>", lambda e: self.set_interacting(True)); s.bind("<ButtonRelease-1>", lambda e: self.set_interacting(False))

    def toggle_test(self):
        st = self.controller.toggle_test(); t = self.current_theme
        self.btn_test.config(text="⏹️ DETENER TEST" if st else "🌊 TEST SENOIDAL", bg=t['btn_reset'] if st else t['btn_test'])
        if st: self.btn_power.config(text="⚡ MOTORES: ON", bg=t['btn_power'])
    def toggle_pickplace(self):
        st = self.controller.toggle_pickplace(); t = self.current_theme
        self.btn_pickplace.config(text="🤖 PICK&PLACE: ON" if st else "🤖 PICK & PLACE", bg=t['btn_pickplace'] if st else t['ps4_inactive'])
        if st: self.btn_power.config(text="⚡ MOTORES: ON", bg=t['btn_power'])
    def toggle_power(self):
        st = self.controller.toggle_power(); t = self.current_theme
        self.btn_power.config(text="⚡ MOTORES: ON" if st else "⚡ ACTIVAR MOTORES (OFF)", bg=t['btn_power'] if st else t['ps4_inactive'])
    def toggle_theme(self):
        self.current_theme_key = 'light' if self.current_theme_key == 'dark' else 'dark'; self.apply_theme(self.current_theme_key)
    def apply_theme(self, theme_key):
        self.current_theme_key = theme_key; t = THEMES[theme_key]; self.current_theme = t
        self.root.configure(bg=t['bg_main'])
        for f in self.frames: f.configure(bg=t['bg_main'] if f != self.ps4_frame else t['bg_panel'])
        self.sliders_container.configure(bg=t['bg_main']); self.ps4_frame.configure(bg=t['bg_panel'])
        for child in self.sliders_container.winfo_children(): child.configure(bg=t['bg_panel'])
        for lbl in self.labels: lbl.configure(bg=t['bg_panel'] if lbl.master in self.sliders_container.winfo_children() else t['bg_main'], fg=t['fg_text'])
        self.title_lbl.configure(bg=t['bg_main'], fg=t['accent_1']); self.status_lbl.configure(bg=t['bg_main'], fg=t['accent_2'])
        self.lbl_leg_target.configure(bg=t['bg_main'], fg=t['target_color']); self.lbl_leg_real.configure(bg=t['bg_main'], fg=t['real_color'])
        for name, s in self.sliders.items(): s.configure(bg=t['bg_panel'], fg=t['fg_text'], troughcolor=t['bg_main'], activebackground=t['accent_1'])
        for name, (e, var) in self.entries.items(): e.configure(bg=t['bg_entry'], fg=t['fg_entry'], insertbackground=t['fg_text'])
        self.btn_theme.configure(bg=t['bg_main'], fg=t['fg_text'], activebackground=t['bg_panel'])
        self.btn_close.configure(bg=t['btn_close'], activebackground=t['btn_close']); self.btn_open.configure(bg=t['btn_open'], activebackground=t['btn_open']); self.btn_reset.configure(bg=t['btn_reset'], activebackground=t['btn_reset'])
        if self.controller.motors_enabled: self.btn_power.configure(bg=t['btn_power'])
        else: self.btn_power.configure(bg=t['ps4_inactive'])
        if self.controller.test_mode: self.btn_test.configure(bg=t['btn_reset'])
        else: self.btn_test.configure(bg=t['btn_test'])
        if self.controller.pickplace_state != PickPlaceState.DISABLED: self.btn_pickplace.configure(bg=t['btn_pickplace'])
        else: self.btn_pickplace.configure(bg=t['ps4_inactive'])
        self.update_ps4_btn_color()
        for g in self.graphs.values(): g.apply_theme_colors(t)
    def update_ps4_btn_color(self):
        t = self.current_theme
        if self.controller.ps4_enabled: self.btn_ps4.config(text="🎮 CONTROL PS4: ACTIVADO", bg=t['ps4_active'], fg='#1a1b26')
        else: self.btn_ps4.config(text="🎮 CONTROL PS4: DESACTIVADO", bg=t['ps4_inactive'], fg='#ffffff' if self.current_theme_key == 'dark' else '#2c3e50')
    def toggle_ps4(self):
        if self.controller.test_mode: messagebox.showwarning("Advertencia", "Detén el test primero"); return
        self.controller.ps4_enabled = not self.controller.ps4_enabled; self.update_ps4_btn_color()
    def on_slider_input(self, name, value):
        if self.updating_from_entry: return
        val = float(value); deg = val * 180.0 / math.pi
        if not self.is_user_interacting: self.entries[name][1].set(f"{deg:.1f}")
        if self.is_user_interacting: self.controller.update_from_slider(name, val)
    def on_entry_input(self, name, str_var):
        try:
            self.updating_from_entry = True; deg_val = float(str_var.get()); deg_val = max(-90, min(90, deg_val))
            self.controller.update_from_input_deg(name, deg_val); rad_val = deg_val * math.pi / 180.0
            self.sliders[name].set(rad_val); self.root.focus()
        except: pass
        self.root.after(100, self.reset_entry_flag)
    def reset_entry_flag(self): self.updating_from_entry = False
    def set_interacting(self, status): self.is_user_interacting = status
    def reset_gui(self):
        self.controller.reset_all()
        for name in self.sliders: self.sliders[name].set(0.0); self.entries[name][1].set("0.0")
        t = self.current_theme
        self.btn_power.config(text="⚡ ACTIVAR MOTORES (OFF)", bg=t['ps4_inactive']); self.btn_test.config(text="🌊 TEST SENOIDAL", bg=t['btn_test']); self.btn_pickplace.config(text="🤖 PICK & PLACE", bg=t['ps4_inactive'])
    def sync_gui_with_ps4(self):
        if not self.is_user_interacting:
            targets_rad = [self.controller.q_target_raw[0], self.controller.q_target_raw[1], self.controller.q_target_raw[2]]
            names = ["base_to_base", "base_to_arm", "base_to_arm2"]
            for i, name in enumerate(names):
                rad_val = targets_rad[i]; deg_val = rad_val * 180.0 / math.pi
                self.sliders[name].set(rad_val); self.entries[name][1].set(f"{deg_val:.1f}")
        self.root.after(50, self.sync_gui_with_ps4)
    def update_graphs_loop(self):
        self.graphs["M1"].add_point(self.controller.M1_deg, self.controller.real_M1)
        self.graphs["M2"].add_point(self.controller.M2_deg, self.controller.real_M2)
        self.graphs["M3"].add_point(self.controller.M3_deg, self.controller.real_M3)
        self.root.after(50, self.update_graphs_loop)
    def run(self): self.root.mainloop()

def ros_spin(node): rclpy.spin(node)
def main(args=None):
    rclpy.init(args=args); controller = RobotGUIControllerDual()
    t = threading.Thread(target=ros_spin, args=(controller,), daemon=True); t.start()
    gui = RobotGUI(controller)
    try: gui.run()
    except KeyboardInterrupt: pass
    finally:
        if controller.arduino_connected: controller.reset_all(); controller.arduino_port.close()
        controller.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()