#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import serial
import time
import math
import numpy as np
from collections import deque

# ═══════════════════════════════════════════════════════
# TEMAS (DARK & LIGHT) - DEL CÓDIGO ANTIGUO
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
        'ps4_active': '#2ecc71',
        'ps4_inactive': '#bdc3c7',
        'fg_entry': '#2980b9',
        'bg_entry': '#ecf0f1'
    }
}

# ═══════════════════════════════════════════════════════
# GRÁFICAS - DEL CÓDIGO ANTIGUO
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

        # Grilla vertical
        num_v_lines = 10
        step_v = graph_w / num_v_lines
        for i in range(1, num_v_lines):
            x = margin_left + i * step_v
            self.create_line(x, margin_top, x, h-margin_bottom, fill=self.theme['grid_sub'], width=1)

        # Líneas horizontales
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
# MODELO DINÁMICO (TU CÓDIGO ACTUAL - SIN TOCAR)
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

        g1 = 0; g2 = 6050*c2 + 1110*s2; g3 = 268*s3 - 8320*c3
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
# CONTROLADOR (TU BACKEND ACTUAL - SIN TOCAR)
# ═══════════════════════════════════════════════════════
class RobotGUIControllerDual(Node):
    def __init__(self):
        super().__init__('robot_gui_controller_dual')
        self.arm_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.arduino_connected = False; self.arduino_port = None
        self.connect_arduino()
        
        self.dynamics = DynamicsModel()
        self.traj_gen = TrajectoryGen()
        
        # TUS PARÁMETROS CALIBRADOS - SIN TOCAR
        self.Kp = np.diag([350.0, 300.0, 450.0]) 
        self.Kd = np.diag([10.0, 10.0, 10.0]) 
        self.Ki = np.diag([25.0, 115.0, 90.0]) 
        self.e_integral = np.zeros(3)
        
        self.q_curr = np.zeros(3); self.dq_curr = np.zeros(3)
        self.q_target_raw = np.zeros(3)
        self.gripper_val = 90
        
        self.motors_enabled = False; self.initialized_pose = False
        self.test_mode = False; self.test_start_time = 0; self.test_freq = 0.25

        # Variables PS4 / GUI
        self.ps4_enabled = False; self.joy_vel = np.zeros(3); self.prev_buttons = [0]*20
        self.base_to_base=0.0; self.base_to_arm=0.0; self.base_to_arm2=0.0
        
        # Para GUI (grados)
        self.M1_deg = 0.0; self.M2_deg = 0.0; self.M3_deg = 0.0
        self.real_M1 = 0.0; self.real_M2 = 0.0; self.real_M3 = 0.0
        
        self.create_timer(0.05, self.ps4_update_loop)
        self.create_timer(0.02, self.control_loop)
        
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
                            
                            # Para GUI
                            self.real_M1 = deg[0]
                            self.real_M2 = deg[1]
                            self.real_M3 = deg[2]
                            
                            if not self.initialized_pose:
                                self.q_target_raw = self.q_curr.copy()
                                self.traj_gen.q = self.q_curr.copy()
                                self.initialized_pose = True
            except: pass
            time.sleep(0.002)

    def control_loop(self):
        if not self.arduino_connected or not self.initialized_pose: return
        
        target = self.q_target_raw.copy()
        if self.test_mode:
            t = time.time() - self.test_start_time
            target[0] = 0.4 * np.sin(2 * np.pi * self.test_freq * t)
            target[1] = 0.4 * np.sin(2 * np.pi * self.test_freq * t + np.pi/2)
            target[2] = 0.4 * np.sin(2 * np.pi * self.test_freq * t)
            self.q_target_raw = target

        q_d, dq_d, ddq_d = self.traj_gen.update(target)
        self.base_to_base, self.base_to_arm, self.base_to_arm2 = q_d
        
        # Para GUI (grados)
        self.M1_deg = np.rad2deg(q_d[0])
        self.M2_deg = np.rad2deg(q_d[1])
        self.M3_deg = np.rad2deg(q_d[2])

        if self.motors_enabled:
            dt = 0.02
            self.e_integral += (q_d - self.q_curr) * dt
            self.e_integral = np.clip(self.e_integral, -1.0, 1.0)
            
            tau = self.dynamics.compute_torque(self.q_curr, self.dq_curr, q_d, dq_d, ddq_d, self.Kp, self.Ki, self.Kd, self.e_integral)
            
            u_static = np.array([6.0, 6.0, 6.3]) * np.sign(dq_d)
            u_visc = 5.0 * dq_d
            
            u = (tau * 80.0) + u_static + u_visc
            u = np.clip(u, -12.0, 12.0)
            
            try: 
                self.arduino_port.write(f"CMD:{u[0]:.2f},{u[1]:.2f},{u[2]:.2f},{int(self.gripper_val)}\n".encode())
            except: pass
        elif not self.motors_enabled:
            try: 
                self.arduino_port.write(f"CMD:0,0,0,{int(self.gripper_val)}\n".encode())
            except: pass

        msg = Float64MultiArray(); msg.data = [q_d[0], q_d[1], q_d[2]]
        self.arm_pub.publish(msg)

    def toggle_power(self):
        self.motors_enabled = not self.motors_enabled
        if self.motors_enabled: 
            self.q_target_raw = self.q_curr.copy()
            self.traj_gen.q = self.q_curr.copy()
            self.e_integral = np.zeros(3)
        return self.motors_enabled
    
    def toggle_test(self):
        self.test_mode = not self.test_mode
        if self.test_mode: 
            self.test_start_time = time.time()
            self.motors_enabled = True
        else:
            self.q_target_raw = np.zeros(3)
        return self.test_mode

    def update_target_deg(self, idx, deg): 
        if not self.test_mode: self.q_target_raw[idx] = np.deg2rad(deg)
    
    def update_from_input_deg(self, joint, deg_value):
        if self.test_mode: return
        idx = {"base_to_base": 0, "base_to_arm": 1, "base_to_arm2": 2}[joint]
        self.q_target_raw[idx] = np.deg2rad(deg_value)
    
    def update_from_slider(self, joint, value):
        if self.test_mode: return
        idx = {"base_to_base": 0, "base_to_arm": 1, "base_to_arm2": 2}[joint]
        self.q_target_raw[idx] = float(value)
    
    def joy_callback(self, msg):
        if self.ps4_enabled and not self.test_mode:
            self.joy_vel[0] = -msg.axes[3]*0.05
            self.joy_vel[1] = msg.axes[1]*0.05
            if msg.buttons[3]==1 and self.prev_buttons[3]==0: self.gripper_val=110
            if msg.buttons[1]==1 and self.prev_buttons[1]==0: self.gripper_val=80
            self.prev_buttons = list(msg.buttons)
            
    def ps4_update_loop(self):
        if self.ps4_enabled and not self.test_mode:
            self.q_target_raw = np.clip(self.q_target_raw + self.joy_vel, -1.57, 1.57)

    def reset_all(self):
        self.test_mode = False; self.q_target_raw = np.zeros(3); self.e_integral = np.zeros(3)
        if self.arduino_connected: self.arduino_port.write(b"RESET\n")
    
    def command_close_gripper(self):
        self.gripper_val = 120
    
    def command_open_gripper(self):
        def sequence():
            self.gripper_val = 70
            time.sleep(0.7)
            self.gripper_val = 90
        threading.Thread(target=sequence, daemon=True).start()

# ═══════════════════════════════════════════════════════
# INTERFAZ GRÁFICA COMPLETA (DEL CÓDIGO ANTIGUO)
# ═══════════════════════════════════════════════════════
class RobotGUI:
    def __init__(self, controller):
        self.controller = controller
        self.root = tk.Tk()
        self.root.title("🤖 Robot Master Controller + Graphs")
        self.root.geometry("1000x900")
        
        self.current_theme_key = 'dark'
        self.current_theme = THEMES['dark']
        self.is_user_interacting = False
        self.updating_from_entry = False
        
        self.frames = []
        self.labels = []
        self.sliders = {}
        self.entries = {}
        self.graphs = {}
        
        self.setup_ui()
        self.apply_theme(self.current_theme_key)
        
        self.sync_gui_with_ps4()
        self.update_graphs_loop()

    def setup_ui(self):
        # Frame Principal
        self.main_frame = tk.Frame(self.root)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        self.frames.append(self.main_frame)
        
        # Header
        header_frame = tk.Frame(self.main_frame, height=50)
        header_frame.pack(fill=tk.X, pady=(0, 10))
        self.frames.append(header_frame)
        
        self.title_lbl = tk.Label(header_frame, text="ROBOT MASTER CONTROL", font=('Segoe UI', 24, 'bold'))
        self.title_lbl.pack(side=tk.TOP, pady=5)
        self.labels.append(self.title_lbl)
        
        self.btn_theme = tk.Button(header_frame, text="🌗", font=('Arial', 14), 
                                   command=self.toggle_theme, relief=tk.FLAT, cursor='hand2')
        self.btn_theme.place(relx=1.0, y=0, anchor='ne')
        
        # Estado Conexión
        status_text = "🟢 CONECTADO: Gazebo + Arduino" if self.controller.arduino_connected else "🟡 MODO SIMULACIÓN: Gazebo Only"
        self.status_lbl = tk.Label(self.main_frame, text=status_text, font=('Segoe UI', 11))
        self.status_lbl.pack(pady=(5, 10))
        self.labels.append(self.status_lbl)
        
        # --- BOTONES DE CONTROL (POWER Y TEST) ---
        ctrl_frame = tk.Frame(self.main_frame)
        ctrl_frame.pack(fill=tk.X, pady=10)
        self.frames.append(ctrl_frame)
        
        self.btn_power = tk.Button(ctrl_frame, text="⚡ ACTIVAR MOTORES (OFF)", command=self.toggle_power, 
                                   font=('Segoe UI', 11, 'bold'), relief=tk.FLAT, width=25, height=2, cursor='hand2')
        self.btn_power.pack(side=tk.LEFT, padx=10)
        
        self.btn_test = tk.Button(ctrl_frame, text="🌊 TEST SENOIDAL", command=self.toggle_test, 
                                  font=('Segoe UI', 11, 'bold'), relief=tk.FLAT, width=20, height=2, cursor='hand2')
        self.btn_test.pack(side=tk.LEFT, padx=10)
        
        # --- PS4 PANEL ---
        self.ps4_frame = tk.Frame(self.main_frame, relief=tk.FLAT, bd=0)
        self.ps4_frame.pack(fill=tk.X, pady=10, ipady=10, ipadx=10)
        self.frames.append(self.ps4_frame)
        
        self.btn_ps4 = tk.Button(self.ps4_frame, text="🎮 CONTROL PS4: DESACTIVADO", command=self.toggle_ps4,
                                 font=('Segoe UI', 12, 'bold'), relief=tk.FLAT, width=30, height=2, cursor='hand2')
        self.btn_ps4.pack(pady=5)

        # --- SLIDERS PANEL ---
        self.sliders_container = tk.Frame(self.main_frame)
        self.sliders_container.pack(fill=tk.X, pady=10) 
        self.frames.append(self.sliders_container)
        
        joints = [
            ("Base Rotation (M1)", "base_to_base", -1.57, 1.57),
            ("Arm Joint 1 (M2)", "base_to_arm", -1.57, 1.57),
            ("Arm Joint 2 (M3)", "base_to_arm2", -1.57, 1.57),
        ]
        
        for label, name, min_v, max_v in joints:
            self.create_control_row(self.sliders_container, label, name, min_v, max_v)

        # --- BOTONES DE ACCIÓN ---
        self.btn_container = tk.Frame(self.main_frame)
        self.btn_container.pack(pady=15)
        self.frames.append(self.btn_container)
        
        def mk_btn(txt, cmd):
            return tk.Button(self.btn_container, text=txt, command=cmd, font=('Segoe UI', 10, 'bold'), 
                             relief=tk.FLAT, width=18, height=2, cursor='hand2')

        self.btn_close = mk_btn("✊ CERRAR GRIPPER", self.controller.command_close_gripper)
        self.btn_close.pack(side=tk.LEFT, padx=10)
        
        self.btn_open = mk_btn("✋ ABRIR GRIPPER", self.controller.command_open_gripper)
        self.btn_open.pack(side=tk.LEFT, padx=10)
        
        self.btn_reset = mk_btn("🔄 RESET HOME", self.reset_gui)
        self.btn_reset.pack(side=tk.LEFT, padx=10)

        # --- GRÁFICAS PANEL ---
        self.graphs_frame = tk.Frame(self.main_frame)
        self.graphs_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        self.frames.append(self.graphs_frame)
        
        self.g1 = GraphWidget(self.graphs_frame, "M1: Base", width=10, height=300) 
        self.g1.pack(side=tk.LEFT, padx=5, fill=tk.BOTH, expand=True)
        self.graphs["M1"] = self.g1
        
        self.g2 = GraphWidget(self.graphs_frame, "M2: Arm 1", width=10, height=300)
        self.g2.pack(side=tk.LEFT, padx=5, fill=tk.BOTH, expand=True)
        self.graphs["M2"] = self.g2

        self.g3 = GraphWidget(self.graphs_frame, "M3: Arm 2", width=10, height=300)
        self.g3.pack(side=tk.LEFT, padx=5, fill=tk.BOTH, expand=True)
        self.graphs["M3"] = self.g3
        
        # Leyenda
        self.legend_frame = tk.Frame(self.main_frame)
        self.legend_frame.pack(pady=5)
        self.frames.append(self.legend_frame)
        
        self.lbl_leg_target = tk.Label(self.legend_frame, text="⬤ Target (Deseado)", font=('Segoe UI', 10, 'bold'))
        self.lbl_leg_target.pack(side=tk.LEFT, padx=10)
        self.lbl_leg_real = tk.Label(self.legend_frame, text="⬤ Real (Encoder)", font=('Segoe UI', 10, 'bold'))
        self.lbl_leg_real.pack(side=tk.LEFT, padx=10)

    def create_control_row(self, parent, label, name, min_v, max_v):
        f = tk.Frame(parent, height=50)
        f.pack(fill=tk.X, pady=5, ipady=5)
        f.pack_propagate(False)
        self.frames.append(f)
        
        lbl = tk.Label(f, text=label, width=20, font=('Segoe UI', 12, 'bold'))
        lbl.pack(side=tk.LEFT, padx=20)
        self.labels.append(lbl)
        
        s = tk.Scale(f, from_=min_v, to=max_v, resolution=0.001, orient=tk.HORIZONTAL,
                     highlightthickness=0, showvalue=0, length=300,
                     command=lambda v: self.on_slider_input(name, v))
        s.set(0.0)
        s.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=20)
        
        entry_var = tk.StringVar(value="0.0")
        e = tk.Entry(f, textvariable=entry_var, width=6, 
                     font=('Consolas', 12, 'bold'), justify='center',
                     relief=tk.FLAT)
        e.pack(side=tk.RIGHT, padx=20)
        e.bind('<Return>', lambda event: self.on_entry_input(name, entry_var))
        
        unit_lbl = tk.Label(f, text="deg", font=('Segoe UI', 9))
        unit_lbl.pack(side=tk.RIGHT)
        self.labels.append(unit_lbl)
        
        self.sliders[name] = s
        self.entries[name] = (e, entry_var)
        s.bind("<Button-1>", lambda e: self.set_interacting(True))
        s.bind("<ButtonRelease-1>", lambda e: self.set_interacting(False))

    def toggle_power(self):
        st = self.controller.toggle_power()
        t = self.current_theme
        self.btn_power.config(
            text="⚡ MOTORES: ON" if st else "⚡ ACTIVAR MOTORES (OFF)", 
            bg=t['btn_power'] if st else t['ps4_inactive']
        )

    def toggle_test(self):
        st = self.controller.toggle_test()
        t = self.current_theme
        self.btn_test.config(
            text="⏹️ DETENER TEST" if st else "🌊 TEST SENOIDAL", 
            bg=t['btn_reset'] if st else t['btn_test']
        )
        if st:
            self.btn_power.config(text="⚡ MOTORES: ON", bg=t['btn_power'])

    def toggle_theme(self):
        new_theme = 'light' if self.current_theme_key == 'dark' else 'dark'
        self.apply_theme(new_theme)

    def apply_theme(self, theme_key):
        self.current_theme_key = theme_key
        t = THEMES[theme_key]
        self.current_theme = t
        
        self.root.configure(bg=t['bg_main'])
        for f in self.frames:
            f.configure(bg=t['bg_main'] if f != self.ps4_frame else t['bg_panel'])
        
        self.sliders_container.configure(bg=t['bg_main'])
        self.ps4_frame.configure(bg=t['bg_panel'])
        for child in self.sliders_container.winfo_children():
            child.configure(bg=t['bg_panel'])

        for lbl in self.labels:
            lbl.configure(bg=t['bg_panel'] if lbl.master in self.sliders_container.winfo_children() else t['bg_main'], 
                          fg=t['fg_text'])
        
        self.title_lbl.configure(bg=t['bg_main'], fg=t['accent_1'])
        self.status_lbl.configure(bg=t['bg_main'], fg=t['accent_2'])
        self.lbl_leg_target.configure(bg=t['bg_main'], fg=t['target_color'])
        self.lbl_leg_real.configure(bg=t['bg_main'], fg=t['real_color'])

        for name, s in self.sliders.items():
            s.configure(bg=t['bg_panel'], fg=t['fg_text'], troughcolor=t['bg_main'], activebackground=t['accent_1'])
        
        for name, (e, var) in self.entries.items():
            e.configure(bg=t['bg_entry'], fg=t['fg_entry'], insertbackground=t['fg_text'])

        self.btn_theme.configure(bg=t['bg_main'], fg=t['fg_text'], activebackground=t['bg_panel'])
        self.btn_close.configure(bg=t['btn_close'], activebackground=t['btn_close'])
        self.btn_open.configure(bg=t['btn_open'], activebackground=t['btn_open'])
        self.btn_reset.configure(bg=t['btn_reset'], activebackground=t['btn_reset'])
        
        # Actualizar botones power y test
        if self.controller.motors_enabled:
            self.btn_power.configure(bg=t['btn_power'])
        else:
            self.btn_power.configure(bg=t['ps4_inactive'])
        
        if self.controller.test_mode:
            self.btn_test.configure(bg=t['btn_reset'])
        else:
            self.btn_test.configure(bg=t['btn_test'])
        
        self.update_ps4_btn_color()

        for g in self.graphs.values():
            g.apply_theme_colors(t)

    def update_ps4_btn_color(self):
        t = self.current_theme
        if self.controller.ps4_enabled:
            self.btn_ps4.config(text="🎮 CONTROL PS4: ACTIVADO", bg=t['ps4_active'], fg='#1a1b26')
        else:
            self.btn_ps4.config(text="🎮 CONTROL PS4: DESACTIVADO", bg=t['ps4_inactive'], 
                               fg='#ffffff' if self.current_theme_key == 'dark' else '#2c3e50')

    def toggle_ps4(self):
        if self.controller.test_mode:
            messagebox.showwarning("Advertencia", "Detén el test primero")
            return
        self.controller.ps4_enabled = not self.controller.ps4_enabled
        self.update_ps4_btn_color()

    def on_slider_input(self, name, value):
        if self.updating_from_entry:
            return

        val = float(value)
        deg = val * 180.0 / math.pi
        
        if not self.is_user_interacting: 
             self.entries[name][1].set(f"{deg:.1f}")
             
        if self.is_user_interacting:
            self.controller.update_from_slider(name, val)

    def on_entry_input(self, name, str_var):
        try:
            self.updating_from_entry = True
            
            deg_val = float(str_var.get())
            deg_val = max(-90, min(90, deg_val))
            
            self.controller.update_from_input_deg(name, deg_val)
            
            rad_val = deg_val * math.pi / 180.0
            self.sliders[name].set(rad_val)
            
            self.root.focus()
        except: 
            pass
        
        self.root.after(100, self.reset_entry_flag)

    def reset_entry_flag(self):
        self.updating_from_entry = False

    def set_interacting(self, status): 
        self.is_user_interacting = status
    
    def reset_gui(self):
        self.controller.reset_all()
        for name in self.sliders: 
            self.sliders[name].set(0.0)
            self.entries[name][1].set("0.0")
        
        t = self.current_theme
        self.btn_power.config(text="⚡ ACTIVAR MOTORES (OFF)", bg=t['ps4_inactive'])
        self.btn_test.config(text="🌊 TEST SENOIDAL", bg=t['btn_test'])

    def sync_gui_with_ps4(self):
        # Actualizar sliders con los valores del controlador
        if not self.is_user_interacting:
            targets_rad = [self.controller.q_target_raw[0], self.controller.q_target_raw[1], self.controller.q_target_raw[2]]
            names = ["base_to_base", "base_to_arm", "base_to_arm2"]
            
            for i, name in enumerate(names):
                rad_val = targets_rad[i]
                deg_val = rad_val * 180.0 / math.pi
                self.sliders[name].set(rad_val)
                self.entries[name][1].set(f"{deg_val:.1f}")
        
        self.root.after(50, self.sync_gui_with_ps4)

    def update_graphs_loop(self):
        self.graphs["M1"].add_point(self.controller.M1_deg, self.controller.real_M1)
        self.graphs["M2"].add_point(self.controller.M2_deg, self.controller.real_M2)
        self.graphs["M3"].add_point(self.controller.M3_deg, self.controller.real_M3)
        self.root.after(50, self.update_graphs_loop)

    def run(self): 
        self.root.mainloop()

def ros_spin(node): 
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotGUIControllerDual()
    t = threading.Thread(target=ros_spin, args=(controller,), daemon=True)
    t.start()
    gui = RobotGUI(controller)
    try: 
        gui.run()
    except KeyboardInterrupt: 
        pass
    finally:
        if controller.arduino_connected: 
            controller.reset_all()
            controller.arduino_port.close()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()