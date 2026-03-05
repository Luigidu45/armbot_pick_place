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
# 1. LÓGICA MATEMÁTICA (LA QUE FUNCIONA AL 100%)
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
        
        # Matrices (Tu modelo validado)
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
        g3 = 268*s3 - 8320*c3 # Signo corregido
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
# 2. CONTROLADOR (BACKEND)
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
        
        # --- TU CALIBRACIÓN "PERFECTA" ---
        self.Kp = np.diag([350.0, 400.0, 450.0]) 
        self.Kd = np.diag([10.0, 10.0, 10.0]) 
        self.Ki = np.diag([25.0, 115.0, 90.0]) 
        self.e_integral = np.zeros(3)
        
        self.q_curr = np.zeros(3); self.dq_curr = np.zeros(3)
        self.q_target_raw = np.zeros(3) # Target del usuario (rad)
        self.gripper_val = 90
        
        self.motors_enabled = False; self.initialized_pose = False
        self.test_mode = False; self.test_start_time = 0; self.test_freq = 0.25

        # Variables PS4 / GUI
        self.ps4_enabled = False; self.joy_vel = np.zeros(3); self.prev_buttons = [0]*20
        self.base_to_base=0.0; self.base_to_arm=0.0; self.base_to_arm2=0.0 # Para GUI
        
        self.create_timer(0.05, self.ps4_update_loop)
        self.create_timer(0.02, self.control_loop)
        
        if self.arduino_connected: threading.Thread(target=self.serial_read_loop, daemon=True).start()

    def connect_arduino(self):
        ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', 'COM3']
        for p in ports:
            try:
                self.arduino_port = serial.Serial(p, 115200, timeout=0.05)
                time.sleep(2.5) # Esperar reset
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
            self.q_target_raw = target # Actualizar para que la GUI lo vea

        q_d, dq_d, ddq_d = self.traj_gen.update(target)
        self.base_to_base, self.base_to_arm, self.base_to_arm2 = q_d # Update GUI vars

        if self.motors_enabled:
            dt = 0.02
            self.e_integral += (q_d - self.q_curr) * dt
            self.e_integral = np.clip(self.e_integral, -1.0, 1.0)
            
            tau = self.dynamics.compute_torque(self.q_curr, self.dq_curr, q_d, dq_d, ddq_d, self.Kp, self.Ki, self.Kd, self.e_integral)
            
            # Fricción personalizada (M3 reforzado)
            u_static = np.array([6.0, 6.5, 6.8]) * np.sign(dq_d)
            u_visc = 5.0 * dq_d
            
            u = (tau * 80.0) + u_static + u_visc
            u = np.clip(u, -12.0, 12.0)
            
            try: self.arduino_port.write(f"CMD:{u[0]:.2f},{u[1]:.2f},{u[2]:.2f},{int(self.gripper_val)}\n".encode())
            except: pass
        
        # Enviar 0 si motores apagados para mantener alive
        elif not self.motors_enabled:
             try: self.arduino_port.write(f"CMD:0,0,0,{int(self.gripper_val)}\n".encode())
             except: pass

        msg = Float64MultiArray(); msg.data = [q_d[0], q_d[1], q_d[2]]
        self.arm_pub.publish(msg)

    # Funciones de utilidad
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
            self.motors_enabled = True # Auto-ON
        else:
            self.q_target_raw = np.zeros(3)
        return self.test_mode

    def update_target_deg(self, idx, deg): 
        if not self.test_mode: self.q_target_raw[idx] = np.deg2rad(deg)
    
    def joy_callback(self, msg):
        if self.ps4_enabled and not self.test_mode:
            self.joy_vel[0] = -msg.axes[3]*0.05
            self.joy_vel[1] = msg.axes[1]*0.05
            # Botones
            if msg.buttons[3]==1 and self.prev_buttons[3]==0: self.gripper_val=110
            if msg.buttons[1]==1 and self.prev_buttons[1]==0: self.gripper_val=80
            self.prev_buttons = list(msg.buttons)
            
    def ps4_update_loop(self):
        if self.ps4_enabled and not self.test_mode:
            self.q_target_raw = np.clip(self.q_target_raw + self.joy_vel, -1.57, 1.57)

    def reset_all(self):
        self.test_mode = False; self.q_target_raw = np.zeros(3); self.e_integral = np.zeros(3)
        if self.arduino_connected: self.arduino_port.write(b"RESET\n")

# ═══════════════════════════════════════════════════════
# TEMAS (TU CÓDIGO ORIGINAL PRESERVADO)
# ═══════════════════════════════════════════════════════
THEMES = {
    'dark': { 'bg_main': '#13141f', 'bg_panel': '#1f202e', 'fg_text': '#a9b1d6', 'accent_1': '#7aa2f7', 'accent_2': '#bb9af7', 'target_color': '#f7768e', 'real_color': '#9ece6a', 'grid_color': '#2f334d', 'grid_sub': '#24283b', 'btn_close': '#f7768e', 'btn_open': '#9ece6a', 'btn_reset': '#e0af68', 'ps4_active': '#9ece6a', 'ps4_inactive': '#3b4261', 'fg_entry': '#7aa2f7', 'bg_entry': '#13141f' },
    'light': { 'bg_main': '#f2f4f8', 'bg_panel': '#ffffff', 'fg_text': '#2c3e50', 'accent_1': '#2980b9', 'accent_2': '#8e44ad', 'target_color': '#e74c3c', 'real_color': '#27ae60', 'grid_color': '#dcdde1', 'grid_sub': '#f1f2f6', 'btn_close': '#ff6b6b', 'btn_open': '#51cf66', 'btn_reset': '#fcc419', 'ps4_active': '#2ecc71', 'ps4_inactive': '#bdc3c7', 'fg_entry': '#2980b9', 'bg_entry': '#ecf0f1' }
}

class GraphWidget(tk.Canvas):
    def __init__(self, parent, title, width=280, height=250, theme=THEMES['dark']):
        super().__init__(parent, width=width, height=height, highlightthickness=1)
        self.title_text = title; self.theme = theme
        self.max_points = 100; self.target_data = deque([0.0]*100, maxlen=100); self.real_data = deque([0.0]*100, maxlen=100)
        self.min_y = -95; self.max_y = 95
        self.bind("<Configure>", self.on_resize)
        self.apply_theme_colors(theme)

    def apply_theme_colors(self, theme):
        self.theme = theme; self.config(bg=theme['bg_panel'], highlightbackground=theme['grid_color']); self.draw_graph()
    def on_resize(self, event): self.width = event.width; self.height = event.height; self.draw_graph()
    def add_point(self, target, real): self.target_data.append(target); self.real_data.append(real); self.draw_graph()
    
    def draw_graph(self):
        self.delete("all")
        w = self.winfo_width(); h = self.winfo_height()
        if w < 10: w = int(self['width']); 
        if h < 10: h = int(self['height'])
        ml = 40; mr = 10; mt = 25; mb = 20
        gw = w - ml - mr; gh = h - mt - mb
        if gw <= 0 or gh <= 0: return

        # Grilla original
        for i in range(1, 10): x = ml + i * (gw/10); self.create_line(x, mt, x, h-mb, fill=self.theme['grid_sub'], width=1)
        y_zero = mt + (gh/2); y_mu = mt + (gh*0.25); y_ml = mt + (gh*0.75)
        self.create_line(ml, y_mu, w-mr, y_mu, fill=self.theme['grid_sub'], dash=(2, 4))
        self.create_line(ml, y_ml, w-mr, y_ml, fill=self.theme['grid_sub'], dash=(2, 4))
        self.create_line(ml, y_zero, w-mr, y_zero, fill=self.theme['grid_color'], width=1)
        self.create_text(ml-5, mt, text="90°", fill=self.theme['fg_text'], anchor="e", font=("Consolas", 8))
        self.create_text(ml-5, y_zero, text="0°", fill=self.theme['fg_text'], anchor="e", font=("Consolas", 8))
        self.create_text(ml-5, h-mb, text="-90°", fill=self.theme['fg_text'], anchor="e", font=("Consolas", 8))
        self.create_text(10, 5, text=self.title_text, fill=self.theme['accent_1'], anchor="nw", font=("Arial", 10, "bold"))
        
        self.draw_polyline(self.target_data, self.theme['target_color'], gw, gh, ml, mt)
        self.draw_polyline(self.real_data, self.theme['real_color'], gw, gh, ml, mt)

    def draw_polyline(self, data, color, gw, gh, ml, mt):
        points = []; ry = self.max_y - self.min_y; sx = gw / (self.max_points - 1)
        for i, val in enumerate(data):
            x = ml + (i * sx); ny = 1.0 - ((val - self.min_y) / ry); y = mt + (ny * gh)
            points.append(x); points.append(y)
        if len(points) >= 4: self.create_line(points, fill=color, width=2, smooth=True)

# ═══════════════════════════════════════════════════════
# INTERFAZ (Layout Original + Botones Nuevos)
# ═══════════════════════════════════════════════════════
class RobotGUI:
    def __init__(self, controller):
        self.controller = controller
        self.root = tk.Tk(); self.root.title("🤖 Robot Master Controller + Graphs"); self.root.geometry("1000x900")
        self.current_theme_key = 'dark'; self.current_theme = THEMES['dark']
        self.is_user_interacting = False; self.updating_from_entry = False
        self.frames = []; self.labels = []; self.sliders = {}; self.entries = {}; self.graphs = {}
        self.setup_ui(); self.apply_theme(self.current_theme_key)
        self.sync_gui_with_ps4(); self.update_graphs_loop()

    def setup_ui(self):
        self.main_frame = tk.Frame(self.root); self.main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        self.frames.append(self.main_frame)
        
        # Header
        header_frame = tk.Frame(self.main_frame, height=50); header_frame.pack(fill=tk.X, pady=(0, 10)); self.frames.append(header_frame)
        self.title_lbl = tk.Label(header_frame, text="ROBOT MASTER CONTROL", font=('Segoe UI', 24, 'bold')); self.title_lbl.pack(side=tk.TOP, pady=5); self.labels.append(self.title_lbl)
        self.btn_theme = tk.Button(header_frame, text="🌗", font=('Arial', 14), command=self.toggle_theme, relief=tk.FLAT, cursor='hand2'); self.btn_theme.place(relx=1.0, y=0, anchor='ne')
        
        st = "🟢 CONECTADO" if self.controller.arduino_connected else "🟡 SIMULACIÓN"
        self.status_lbl = tk.Label(self.main_frame, text=st, font=('Segoe UI', 11)); self.status_lbl.pack(pady=(5, 10)); self.labels.append(self.status_lbl)
        
        # --- BOTONERA DE CONTROL (NUEVO: Integrado en tu estilo) ---
        self.ctrl_frame = tk.Frame(self.main_frame); self.ctrl_frame.pack(fill=tk.X, pady=5); self.frames.append(self.ctrl_frame)
        
        self.btn_power = tk.Button(self.ctrl_frame, text="ACTIVAR MOTORES (OFF)", command=self.toggle_power, font=('Segoe UI', 11, 'bold'), bg='#f7768e', width=25)
        self.btn_power.pack(side=tk.LEFT, padx=10)
        
        self.btn_test = tk.Button(self.ctrl_frame, text="🌊 TEST SENO", command=self.toggle_test, font=('Segoe UI', 11, 'bold'), bg='#7aa2f7', width=15)
        self.btn_test.pack(side=tk.LEFT, padx=10)

        # --- PS4 ---
        self.ps4_frame = tk.Frame(self.main_frame, relief=tk.FLAT, bd=0); self.ps4_frame.pack(fill=tk.X, pady=10, ipady=10, ipadx=10); self.frames.append(self.ps4_frame)
        self.btn_ps4 = tk.Button(self.ps4_frame, text="🎮 CONTROL PS4: DESACTIVADO", command=self.toggle_ps4, font=('Segoe UI', 12, 'bold'), relief=tk.FLAT, width=30, height=2, cursor='hand2'); self.btn_ps4.pack(pady=5)

        # --- SLIDERS ---
        self.sliders_container = tk.Frame(self.main_frame); self.sliders_container.pack(fill=tk.X, pady=10); self.frames.append(self.sliders_container)
        joints = [("Base Rotation (M1)", "base", 0), ("Arm Joint 1 (M2)", "shoulder", 1), ("Arm Joint 2 (M3)", "elbow", 2)]
        for label, name, idx in joints: self.create_control_row(self.sliders_container, label, name, idx)

        # --- BOTONES ACCION ---
        self.btn_container = tk.Frame(self.main_frame); self.btn_container.pack(pady=15); self.frames.append(self.btn_container)
        self.btn_close = tk.Button(self.btn_container, text="✊ CERRAR GRIPPER", font=('Segoe UI', 10, 'bold'), relief=tk.FLAT, width=18, height=2); self.btn_close.pack(side=tk.LEFT, padx=10)
        self.btn_open = tk.Button(self.btn_container, text="✋ ABRIR GRIPPER", font=('Segoe UI', 10, 'bold'), relief=tk.FLAT, width=18, height=2); self.btn_open.pack(side=tk.LEFT, padx=10)
        self.btn_reset = tk.Button(self.btn_container, text="🔄 RESET HOME", command=self.reset_gui, font=('Segoe UI', 10, 'bold'), relief=tk.FLAT, width=18, height=2); self.btn_reset.pack(side=tk.LEFT, padx=10)

        # --- GRAFICAS ---
        self.graphs_frame = tk.Frame(self.main_frame); self.graphs_frame.pack(fill=tk.BOTH, expand=True, pady=10); self.frames.append(self.graphs_frame)
        self.graphs["base"] = GraphWidget(self.graphs_frame, "M1: Base", width=10, height=300); self.graphs["base"].pack(side=tk.LEFT, padx=5, fill=tk.BOTH, expand=True)
        self.graphs["shoulder"] = GraphWidget(self.graphs_frame, "M2: Arm 1", width=10, height=300); self.graphs["shoulder"].pack(side=tk.LEFT, padx=5, fill=tk.BOTH, expand=True)
        self.graphs["elbow"] = GraphWidget(self.graphs_frame, "M3: Arm 2", width=10, height=300); self.graphs["elbow"].pack(side=tk.LEFT, padx=5, fill=tk.BOTH, expand=True)
        
        self.legend_frame = tk.Frame(self.main_frame); self.legend_frame.pack(pady=5); self.frames.append(self.legend_frame)
        tk.Label(self.legend_frame, text="⬤ Target", fg=THEMES['dark']['target_color'], bg=THEMES['dark']['bg_main'], font=('Segoe UI', 10, 'bold')).pack(side=tk.LEFT, padx=10)
        tk.Label(self.legend_frame, text="⬤ Real", fg=THEMES['dark']['real_color'], bg=THEMES['dark']['bg_main'], font=('Segoe UI', 10, 'bold')).pack(side=tk.LEFT, padx=10)

    # --- LOGICA GUI MODIFICADA PARA UNIRSE AL CONTROLADOR ---
    def create_control_row(self, parent, label, name, idx):
        f = tk.Frame(parent, height=50); f.pack(fill=tk.X, pady=5, ipady=5); f.pack_propagate(False); self.frames.append(f)
        tk.Label(f, text=label, width=20, font=('Segoe UI', 12, 'bold')).pack(side=tk.LEFT, padx=20)
        s = tk.Scale(f, from_=-90, to=90, resolution=0.1, orient=tk.HORIZONTAL, highlightthickness=0, showvalue=0, length=300, command=lambda v, i=idx: self.on_slider_input(i, name, v))
        s.set(0.0); s.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=20)
        var = tk.StringVar(value="0.0"); e = tk.Entry(f, textvariable=var, width=6, font=('Consolas', 12, 'bold'), justify='center', relief=tk.FLAT)
        e.pack(side=tk.RIGHT, padx=20); e.bind('<Return>', lambda ev, i=idx, n=name, v=var: self.on_entry_input(i, n, v))
        tk.Label(f, text="deg", font=('Segoe UI', 9)).pack(side=tk.RIGHT)
        self.sliders[name] = s; self.entries[name] = (e, var)
        s.bind("<Button-1>", lambda e: self.set_interacting(True)); s.bind("<ButtonRelease-1>", lambda e: self.set_interacting(False))

    def on_slider_input(self, idx, name, val):
        if self.updating_from_entry: return
        self.controller.update_target_deg(idx, float(val))
        if not self.is_user_interacting: self.entries[name][1].set(f"{float(val):.1f}")

    def on_entry_input(self, idx, name, var):
        try:
            self.updating_from_entry = True
            val = float(var.get()); val = max(-90, min(90, val))
            self.controller.update_target_deg(idx, val)
            self.sliders[name].set(val); self.root.focus()
        except: pass
        self.root.after(100, lambda: setattr(self, 'updating_from_entry', False))

    def toggle_power(self):
        st = self.controller.toggle_power()
        self.btn_power.config(text="MOTORES: ON" if st else "MOTORES: OFF", bg='#9ece6a' if st else '#f7768e')

    def toggle_test(self):
        st = self.controller.toggle_test()
        self.btn_test.config(text="DETENER TEST" if st else "🌊 TEST SINE", bg='#e0af68' if st else '#7aa2f7')
        if st: self.btn_power.config(text="MOTORES: ON", bg='#9ece6a')

    def toggle_ps4(self):
        self.controller.ps4_enabled = not self.controller.ps4_enabled
        c = self.current_theme
        self.btn_ps4.config(text="🎮 ACTIVADO" if self.controller.ps4_enabled else "🎮 DESACTIVADO", 
                            bg=c['ps4_active'] if self.controller.ps4_enabled else c['ps4_inactive'])

    def toggle_theme(self):
        self.current_theme_key = 'light' if self.current_theme_key == 'dark' else 'dark'
        self.apply_theme(self.current_theme_key)

    def apply_theme(self, k):
        t = THEMES[k]; self.current_theme = t
        self.root.config(bg=t['bg_main'])
        for f in self.frames: f.config(bg=t['bg_main'] if f!=self.ps4_frame else t['bg_panel'])
        self.sliders_container.config(bg=t['bg_main']); self.ps4_frame.config(bg=t['bg_panel'])
        for child in self.sliders_container.winfo_children(): child.config(bg=t['bg_panel'])
        for l in self.labels: l.config(bg=t['bg_main'], fg=t['fg_text'])
        # Actualizar colores graficas
        for g in self.graphs.values(): g.apply_theme_colors(t)

    def reset_gui(self):
        self.controller.reset_all()
        for n in self.sliders: self.sliders[n].set(0.0); self.entries[n][1].set("0.0")
        self.btn_test.config(text="🌊 TEST SINE", bg='#7aa2f7')

    def sync_gui_with_ps4(self):
        # Actualizar sliders visualmente si el control mueve los valores
        if not self.is_user_interacting:
            targets = [self.controller.q_target_raw[0], self.controller.q_target_raw[1], self.controller.q_target_raw[2]]
            names = ["base", "shoulder", "elbow"]
            for i, n in enumerate(names):
                deg = np.rad2deg(targets[i])
                self.sliders[n].set(deg)
                self.entries[n][1].set(f"{deg:.1f}")
        self.root.after(50, self.sync_gui_with_ps4)

    def update_graphs_loop(self):
        # Mapeo de datos del controlador a la GUI
        q_d = np.rad2deg(self.controller.q_target_raw)
        q = np.rad2deg(self.controller.q_curr)
        self.graphs["base"].add_point(q_d[0], q[0])
        self.graphs["shoulder"].add_point(q_d[1], q[1])
        self.graphs["elbow"].add_point(q_d[2], q[2])
        self.root.after(50, self.update_graphs_loop)

    def set_interacting(self, s): self.is_user_interacting = s
    def run(self): self.root.mainloop()

def ros_spin(node): rclpy.spin(node)
def main(args=None):
    rclpy.init(args=args)
    controller = RobotGUIControllerDual()
    t = threading.Thread(target=ros_spin, args=(controller,), daemon=True); t.start()
    gui = RobotGUI(controller)
    try: gui.run()
    except KeyboardInterrupt: pass
    finally:
        if controller.arduino_connected: controller.reset_all(); controller.arduino_port.close()
        controller.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()