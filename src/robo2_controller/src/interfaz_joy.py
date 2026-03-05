#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
import tkinter as tk
from tkinter import ttk
import threading
import serial
import time
import math
from collections import deque

# ═══════════════════════════════════════════════════════
# TEMAS (DARK & LIGHT)
# ═══════════════════════════════════════════════════════
THEMES = {
    'dark': {
        'bg_main': '#13141f',      # Fondo muy oscuro
        'bg_panel': '#1f202e',     # Fondo de paneles
        'fg_text': '#a9b1d6',      # Texto principal
        'accent_1': '#7aa2f7',     # Azul brillante (Títulos)
        'accent_2': '#bb9af7',     # Púrpura (Subtítulos)
        'target_color': '#f7768e', # Rojo Neón
        'real_color': '#9ece6a',   # Verde Neón
        'grid_color': '#2f334d',   # Grilla
        'grid_sub': '#24283b',     # Grilla secundaria
        'btn_close': '#f7768e',    
        'btn_open': '#9ece6a',     
        'btn_reset': '#e0af68',
        'ps4_active': '#9ece6a',
        'ps4_inactive': '#3b4261',
        'fg_entry': '#7aa2f7',
        'bg_entry': '#13141f'
    },
    'light': {
        'bg_main': '#f2f4f8',      # Blanco grisáceo
        'bg_panel': '#ffffff',     # Blanco puro
        'fg_text': '#2c3e50',      # Gris oscuro
        'accent_1': '#2980b9',     # Azul fuerte
        'accent_2': '#8e44ad',     # Morado
        'target_color': '#e74c3c', # Rojo vivo
        'real_color': '#27ae60',   # Verde vivo
        'grid_color': '#dcdde1',   # Grilla gris claro
        'grid_sub': '#f1f2f6',     # Grilla secundaria
        'btn_close': '#ff6b6b',    
        'btn_open': '#51cf66',     
        'btn_reset': '#fcc419',
        'ps4_active': '#2ecc71',
        'ps4_inactive': '#bdc3c7',
        'fg_entry': '#2980b9',
        'bg_entry': '#ecf0f1'
    }
}

# Clase para manejar cada gráfica individualmente en Tkinter
class GraphWidget(tk.Canvas):
    def __init__(self, parent, title, width=280, height=250, theme=THEMES['dark']):
        super().__init__(parent, width=width, height=height, highlightthickness=1)
        self.title_text = title
        self.theme = theme
        
        # Historial de datos
        self.max_points = 100
        self.target_data = deque([0.0]*self.max_points, maxlen=self.max_points)
        self.real_data = deque([0.0]*self.max_points, maxlen=self.max_points)
        
        # Rangos
        self.min_y = -95
        self.max_y = 95
        
        # Configurar redimensionamiento
        self.bind("<Configure>", self.on_resize)
        self.apply_theme_colors(theme)

    def apply_theme_colors(self, theme):
        self.theme = theme
        self.config(bg=theme['bg_panel'], highlightbackground=theme['grid_color'])
        self.draw_graph()

    def on_resize(self, event):
        # Actualizar dimensiones internas cuando la ventana cambia
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
        # Si la ventana aún no se dibuja, usar valores por defecto
        if w < 10: w = int(self['width'])
        if h < 10: h = int(self['height'])

        margin_left = 40
        margin_right = 10
        margin_top = 25
        margin_bottom = 20
        
        graph_w = w - margin_left - margin_right
        graph_h = h - margin_top - margin_bottom
        
        if graph_w <= 0 or graph_h <= 0: return

        # ============ 1. GRILLA MEJORADA ============
        
        # -- Vertical Lines (Time divisions) --
        num_v_lines = 10
        step_v = graph_w / num_v_lines
        for i in range(1, num_v_lines):
            x = margin_left + i * step_v
            self.create_line(x, margin_top, x, h-margin_bottom, fill=self.theme['grid_sub'], width=1)

        # -- Horizontal Lines --
        y_zero = margin_top + (graph_h / 2)
        y_max = margin_top
        y_min = h - margin_bottom
        
        # Líneas intermedias (+45 y -45)
        y_mid_upper = margin_top + (graph_h * 0.25) # Approx 45 deg position
        y_mid_lower = margin_top + (graph_h * 0.75) # Approx -45 deg position
        
        self.create_line(margin_left, y_mid_upper, w-margin_right, y_mid_upper, fill=self.theme['grid_sub'], dash=(2, 4))
        self.create_line(margin_left, y_mid_lower, w-margin_right, y_mid_lower, fill=self.theme['grid_sub'], dash=(2, 4))

        # Línea Cero (Central) - Más notoria
        self.create_line(margin_left, y_zero, w-margin_right, y_zero, fill=self.theme['grid_color'], width=1)
        
        # Líneas Límite (+90 y -90)
        self.create_line(margin_left, y_max, w-margin_right, y_max, fill=self.theme['grid_color'], width=1)
        self.create_line(margin_left, y_min, w-margin_right, y_min, fill=self.theme['grid_color'], width=1)
        
        # ============ 2. TEXTO EJES ============
        font_axis = ("Consolas", 8)
        self.create_text(margin_left-5, y_max, text="90°", fill=self.theme['fg_text'], anchor="e", font=font_axis)
        self.create_text(margin_left-5, y_mid_upper, text="45°", fill=self.theme['fg_text'], anchor="e", font=font_axis)
        self.create_text(margin_left-5, y_zero, text="0°", fill=self.theme['fg_text'], anchor="e", font=font_axis)
        self.create_text(margin_left-5, y_mid_lower, text="-45°", fill=self.theme['fg_text'], anchor="e", font=font_axis)
        self.create_text(margin_left-5, y_min, text="-90°", fill=self.theme['fg_text'], anchor="e", font=font_axis)
        
        # ============ 3. TÍTULO ============
        self.create_text(10, 5, text=self.title_text, fill=self.theme['accent_1'], anchor="nw", font=("Arial", 10, "bold"))
        
        # ============ 4. DATOS ============
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
            y = max(mt, min(mt + gh, y)) # Clamp
            points.append(x)
            points.append(y)
            
        if len(points) >= 4:
            self.create_line(points, fill=color, width=2, smooth=True)


class RobotGUIControllerDual(Node):
    def __init__(self):
        super().__init__('robot_gui_controller_dual')
        
        self.arm_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.arduino_connected = False
        self.arduino_port = None
        self.connect_arduino()
        
        # Target State
        self.base_to_base = 0.0; self.base_to_arm = 0.0; self.base_to_arm2 = 0.0; self.gazebo_gripper_val = 0.0 
        self.M1_deg = 0.0; self.M2_deg = 0.0; self.M3_deg = 0.0; self.arduino_gripper_val = 90
        
        # Real State
        self.real_M1 = 0.0; self.real_M2 = 0.0; self.real_M3 = 0.0
        
        # PS4 Config
        self.ps4_enabled = False
        self.target_base_velocity = 0.0; self.target_arm_velocity = 0.0
        self.current_base_velocity = 0.0; self.current_arm_velocity = 0.0
        
        self.joystick_max_speed_rad = 0.025; self.button_speed_rad = 0.015; self.smoothing_factor = 0.15; self.deadzone = 0.15
        self.limits_rad = {'base_to_base': (-1.57, 1.57), 'base_to_arm': (-1.57, 1.57), 'base_to_arm2': (-1.57, 1.57)}
        
        self.prev_buttons = [0] * 20
        self.timer = self.create_timer(0.033, self.ps4_update_loop)
        
        if self.arduino_connected:
            self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.read_thread.start()
            
        self.get_logger().info('🤖 Robot Controller Initialized')

    def connect_arduino(self):
        possible_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        for port in possible_ports:
            try:
                self.arduino_port = serial.Serial(port, 115200, timeout=0.1, write_timeout=1.0)
                self.arduino_connected = True
                self.get_logger().info(f'✓ Arduino connected via {port}')
                time.sleep(2)
                self.arduino_port.write(b'A\n')
                time.sleep(0.1)
                return
            except: continue
        self.get_logger().warning('⚠ Arduino not connected.')

    def serial_read_loop(self):
        while self.arduino_connected:
            try:
                if self.arduino_port.in_waiting > 0:
                    line = self.arduino_port.readline().decode('utf-8', errors='ignore').strip()
                    if "FB:" in line:
                        clean_data = line.replace("FB:", "").strip()
                        parts = clean_data.split()
                        if len(parts) >= 3:
                            self.real_M1 = float(parts[0])
                            self.real_M2 = float(parts[1])
                            self.real_M3 = float(parts[2])
            except: pass
            time.sleep(0.01)

    def rad_to_deg(self, rad): return rad * 180.0 / math.pi
    def deg_to_rad(self, deg): return deg * math.pi / 180.0
    def clamp(self, value, min_val, max_val): return max(min_val, min(max_val, value))
    def apply_deadzone(self, value):
        if abs(value) < self.deadzone: return 0.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    def smooth_velocity(self, current, target, factor): return current + (target - current) * factor

    def joy_callback(self, msg):
        if not self.ps4_enabled: return
        left_stick_y = self.apply_deadzone(msg.axes[1])
        self.target_arm_velocity = -left_stick_y * self.joystick_max_speed_rad
        right_stick_x = self.apply_deadzone(msg.axes[3])
        self.target_base_velocity = -right_stick_x * self.joystick_max_speed_rad
        if msg.buttons[2] == 1: self.update_single_joint('base_to_arm2', self.button_speed_rad)
        if msg.buttons[0] == 1: self.update_single_joint('base_to_arm2', -self.button_speed_rad)
        if msg.buttons[3] == 1 and self.prev_buttons[3] == 0: self.command_close_gripper()
        if msg.buttons[1] == 1 and self.prev_buttons[1] == 0: self.command_open_gripper()
        self.prev_buttons = list(msg.buttons)

    def update_single_joint(self, joint, increment_rad):
        if joint == 'base_to_arm2':
            self.base_to_arm2 += increment_rad
            self.base_to_arm2 = self.clamp(self.base_to_arm2, *self.limits_rad['base_to_arm2'])
            self.M3_deg = self.rad_to_deg(self.base_to_arm2)
            self.send_all_commands()

    def ps4_update_loop(self):
        if not self.ps4_enabled: return
        self.current_base_velocity = self.smooth_velocity(self.current_base_velocity, self.target_base_velocity, self.smoothing_factor)
        self.current_arm_velocity = self.smooth_velocity(self.current_arm_velocity, self.target_arm_velocity, self.smoothing_factor)
        changed = False
        if abs(self.current_base_velocity) > 0.0001:
            self.base_to_base += self.current_base_velocity
            self.base_to_base = self.clamp(self.base_to_base, *self.limits_rad['base_to_base'])
            self.M1_deg = self.rad_to_deg(self.base_to_base)
            changed = True
        if abs(self.current_arm_velocity) > 0.0001:
            self.base_to_arm += self.current_arm_velocity
            self.base_to_arm = self.clamp(self.base_to_arm, *self.limits_rad['base_to_arm'])
            self.M2_deg = self.rad_to_deg(self.base_to_arm)
            changed = True
        if changed: self.send_all_commands()

    def send_all_commands(self):
        msg = Float64MultiArray(); msg.data = [self.base_to_base, self.base_to_arm, self.base_to_arm2]; self.arm_pub.publish(msg)
        g_msg = Float64MultiArray(); g_msg.data = [self.gazebo_gripper_val]; self.gripper_pub.publish(g_msg)
        if self.arduino_connected:
            try:
                cmd = f"{self.M1_deg:.2f} {self.M2_deg:.2f} {self.M3_deg:.2f} {self.arduino_gripper_val}\n"
                self.arduino_port.write(cmd.encode())
            except: pass

    def command_close_gripper(self):
        self.arduino_gripper_val = 110; self.gazebo_gripper_val = 0.0; self.send_all_commands()

    def command_open_gripper(self):
        def sequence():
            self.arduino_gripper_val = 80; self.gazebo_gripper_val = 0.19; self.send_all_commands()
            time.sleep(1.0)
            self.arduino_gripper_val = 90; self.send_all_commands()
        threading.Thread(target=sequence, daemon=True).start()

    def reset_all(self):
        self.base_to_base = 0.0; self.base_to_arm = 0.0; self.base_to_arm2 = 0.0
        self.M1_deg = 0.0; self.M2_deg = 0.0; self.M3_deg = 0.0; self.arduino_gripper_val = 90
        self.send_all_commands()
        if self.arduino_connected:
            try: self.arduino_port.write(b'0.01 0.01 0.01 90\n')
            except: pass

    def update_from_input_deg(self, joint, deg_value):
        rad_val = self.deg_to_rad(deg_value)
        if joint == "base_to_base": self.base_to_base = rad_val; self.M1_deg = deg_value
        elif joint == "base_to_arm": self.base_to_arm = rad_val; self.M2_deg = deg_value
        elif joint == "base_to_arm2": self.base_to_arm2 = rad_val; self.M3_deg = deg_value
        self.send_all_commands()

    def update_from_slider(self, joint, value):
        val = float(value)
        if joint == "base_to_base": self.base_to_base = val; self.M1_deg = self.rad_to_deg(val)
        elif joint == "base_to_arm": self.base_to_arm = val; self.M2_deg = self.rad_to_deg(val)
        elif joint == "base_to_arm2": self.base_to_arm2 = val; self.M3_deg = self.rad_to_deg(val)
        self.send_all_commands()

    def __del__(self):
        if self.arduino_connected and self.arduino_port: self.arduino_port.close()

# ═══════════════════════════════════════════════════════
# INTERFAZ GRÁFICA (GUI)
# ═══════════════════════════════════════════════════════
class RobotGUI:
    def __init__(self, controller):
        self.controller = controller
        self.root = tk.Tk()
        self.root.title("🤖 Robot Master Controller + Graphs")
        self.root.geometry("1000x900")
        
        # Variables de estado
        self.current_theme_key = 'dark'
        self.current_theme = THEMES['dark']
        self.is_user_interacting = False
        self.updating_from_entry = False # Bandera para evitar bucles
        
        # Colecciones de widgets para actualizar colores
        self.frames = []
        self.labels = []
        self.sliders = {}
        self.entries = {}
        self.graphs = {}
        
        self.setup_ui()
        self.apply_theme(self.current_theme_key)
        
        # Loops
        self.sync_gui_with_ps4()
        self.update_graphs_loop()

    def setup_ui(self):
        # Frame Principal
        self.main_frame = tk.Frame(self.root)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        self.frames.append(self.main_frame)
        
        # Header (Título y Botón Tema)
        header_frame = tk.Frame(self.main_frame, height=50)
        header_frame.pack(fill=tk.X, pady=(0, 10))
        # No agregamos header_frame a self.frames porque puede tener color diferente si se quisiera
        self.frames.append(header_frame)
        
        # TÍTULO CENTRADO: Usamos pack TOP en el centro
        self.title_lbl = tk.Label(header_frame, text="ROBOT MASTER CONTROL", font=('Segoe UI', 24, 'bold'))
        self.title_lbl.pack(side=tk.TOP, pady=5)
        self.labels.append(self.title_lbl)
        
        # BOTÓN TEMA: Usamos place para fijarlo a la derecha sin empujar el título
        self.btn_theme = tk.Button(header_frame, text="🌗", font=('Arial', 14), 
                                   command=self.toggle_theme, relief=tk.FLAT, cursor='hand2')
        self.btn_theme.place(relx=1.0, y=0, anchor='ne')
        
        # Estado Conexión
        status_text = "🟢 CONECTADO: Gazebo + Arduino" if self.controller.arduino_connected else "🟡 MODO SIMULACIÓN: Gazebo Only"
        self.status_lbl = tk.Label(self.main_frame, text=status_text, font=('Segoe UI', 11))
        self.status_lbl.pack(pady=(5, 10))
        self.labels.append(self.status_lbl)
        
        # --- PS4 PANEL ---
        self.ps4_frame = tk.Frame(self.main_frame, relief=tk.FLAT, bd=0)
        self.ps4_frame.pack(fill=tk.X, pady=10, ipady=10, ipadx=10)
        self.frames.append(self.ps4_frame)
        
        self.btn_ps4 = tk.Button(self.ps4_frame, text="🎮 CONTROL PS4: DESACTIVADO", command=self.toggle_ps4,
                                 font=('Segoe UI', 12, 'bold'), relief=tk.FLAT, width=30, height=2, cursor='hand2')
        self.btn_ps4.pack(pady=5)

        # --- SLIDERS PANEL ---
        # IMPORTANTE: Quitamos expand=True para que este frame solo ocupe lo necesario
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
        # IMPORTANTE: expand=True aquí hace que las gráficas tomen todo el espacio sobrante
        self.graphs_frame = tk.Frame(self.main_frame)
        self.graphs_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        self.frames.append(self.graphs_frame)
        
        # Inicializamos con height más grande por defecto
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
        
        # SLIDER CON RESOLUCIÓN FINA (0.001) para evitar "saltos" al redondear
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
        self.update_ps4_btn_color()

        for g in self.graphs.values():
            g.apply_theme_colors(t)

    def update_ps4_btn_color(self):
        t = self.current_theme
        if self.controller.ps4_enabled:
            self.btn_ps4.config(text="🎮 CONTROL PS4: ACTIVADO", bg=t['ps4_active'], fg='#1a1b26')
        else:
            self.btn_ps4.config(text="🎮 CONTROL PS4: DESACTIVADO", bg=t['ps4_inactive'], fg='#ffffff' if self.current_theme_key == 'dark' else '#2c3e50')

    def toggle_ps4(self):
        self.controller.ps4_enabled = not self.controller.ps4_enabled
        self.update_ps4_btn_color()

    def on_slider_input(self, name, value):
        # SI ESTAMOS ACTUALIZANDO DESDE LA ENTRADA MANUAL, NO SOBREESCRIBIR TEXTO
        if self.updating_from_entry:
            return

        val = float(value)
        deg = val * 180.0 / math.pi
        
        # Actualizar texto solo si no estamos editándolo manualmente
        if not self.is_user_interacting: 
             self.entries[name][1].set(f"{deg:.1f}")
             
        if self.is_user_interacting:
            self.controller.update_from_slider(name, val)

    def on_entry_input(self, name, str_var):
        try:
            self.updating_from_entry = True # BLOQUEAR ACTUALIZACIÓN INVERSA TEMPORALMENTE
            
            deg_val = float(str_var.get())
            deg_val = max(-90, min(90, deg_val))
            
            # 1. Enviar valor EXACTO al controlador/Arduino
            self.controller.update_from_input_deg(name, deg_val)
            
            # 2. Mover slider (esto disparará on_slider_input, pero la bandera lo frenará)
            rad_val = deg_val * math.pi / 180.0
            self.sliders[name].set(rad_val)
            
            self.root.focus()
        except: 
            pass
        
        # DESBLOQUEAR DESPUÉS DE UN MOMENTO (para que el evento del slider muera)
        self.root.after(100, self.reset_entry_flag)

    def reset_entry_flag(self):
        self.updating_from_entry = False

    def set_interacting(self, status): self.is_user_interacting = status
    def reset_gui(self):
        self.controller.reset_all()
        for name in self.sliders: self.sliders[name].set(0.0); self.entries[name][1].set("0.0")

    def sync_gui_with_ps4(self):
        if not self.is_user_interacting and self.controller.ps4_enabled:
            vals = {"base_to_base": self.controller.base_to_base, "base_to_arm": self.controller.base_to_arm, "base_to_arm2": self.controller.base_to_arm2}
            for name, rad_val in vals.items():
                self.sliders[name].set(rad_val)
                self.entries[name][1].set(f"{rad_val * 180.0 / math.pi:.1f}")
        self.root.after(50, self.sync_gui_with_ps4)

    def update_graphs_loop(self):
        self.graphs["M1"].add_point(self.controller.M1_deg, self.controller.real_M1)
        self.graphs["M2"].add_point(self.controller.M2_deg, self.controller.real_M2)
        self.graphs["M3"].add_point(self.controller.M3_deg, self.controller.real_M3)
        self.root.after(50, self.update_graphs_loop)

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
        if controller.arduino_connected: controller.reset_all()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()