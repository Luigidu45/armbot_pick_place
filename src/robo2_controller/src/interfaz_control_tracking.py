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
# 1. MATEMÁTICA Y FÍSICA
# ═══════════════════════════════════════════════════════
class DynamicsModel:
    def __init__(self):
        self.SCALE = 0.000001

    def compute_torque(self, q, dq, q_d, dq_d, ddq_d, Kp, Kv):
        q1, q2, q3 = q
        dq1, dq2, dq3 = dq

        c2 = np.cos(q2); s2 = np.sin(q2)
        c3 = np.cos(q3); s3 = np.sin(q3)
        c2q2 = np.cos(2*q2); s2q2 = np.sin(2*q2)
        c2q3 = np.cos(2*q3); s2q3 = np.sin(2*q3)
        c23 = np.cos(q2 - q3); s23 = np.sin(q2 - q3)
        
        # Inercia D(q)
        d11 = 6.96*s2q2 + 0.131*s2q3 - 289.0*c2*c3 + 121.0*c2q2 + 71.0*c2q3 + 652.0
        d12 = 183.0 * c2; d13 = 183.0 * c2
        d22 = 766.0; d23 = -144.0 * c23; d33 = 200.0
        D = np.array([[d11, d12, d13], [d12, d22, d23], [d13, d23, d33]]) * self.SCALE

        # Coriolis C(q,dq)
        c11 = dq3*(144.0*c2*s3 - 71.0*c3*s3 + 0.262*c2q3 - 0.131) - dq2*(121.0*c2*s2 - 144.0*c3*s2 - 13.9*c2q2 + 6.96)
        c12 = 183.0*dq2*c2 - 91.3*dq3*c2 - dq1*(121.0*c2*s2 - 144.0*c3*s2 - 13.9*c2q2 + 6.96)
        c13 = -91.3*dq2*c2 + dq1*(144.0*c2*s3 - 71.0*c3*s3 + 0.262*c2q3 - 0.131)
        c21 = 91.3*dq3*c2 + dq1*(121.0*c2*s2 - 144.0*c3*s2 - 13.9*c2q2 + 6.96)
        c22 = 0; c23 = 91.3*dq1*c2 - 144.0*dq3*s23
        c31 = -91.3*dq2*c2 - dq1*(144.0*c2*s3 - 71.0*c3*s3 + 0.262*c2q3 - 0.131)
        c32 = 144.0*dq2*s23 - 91.3*dq1*c2; c33 = 0
        C = np.array([[c11, c12, c13], [c21, c22, c23], [c31, c32, c33]]) * self.SCALE

        # Gravedad G(q)
        g1 = 0
        g2 = 6050*c2 + 1110*s2
        g3 = 268*s3 - 8320*c3 
        G = np.array([g1, g2, g3]) * self.SCALE

        e = q_d - q
        de = dq_d - dq
        aux = ddq_d + Kp @ e + Kv @ de
        tau = D @ aux + C @ dq + G
        return tau

class TrajectoryGen:
    def __init__(self, omega_n=4.0, zeta=1.0, dt=0.033):
        self.omega_n = omega_n; self.zeta = zeta; self.dt = dt
        self.q = np.zeros(3); self.dq = np.zeros(3); self.ddq = np.zeros(3)
    def update(self, target):
        error = target - self.q
        self.ddq = (self.omega_n**2 * error) - (2*self.zeta*self.omega_n*self.dq)
        self.dq += self.ddq * self.dt
        self.q += self.dq * self.dt
        return self.q, self.dq, self.ddq

# ═══════════════════════════════════════════════════════
# 2. CONTROLADOR (Lógica Corregida)
# ═══════════════════════════════════════════════════════
class RobotGUIControllerDual(Node):
    def __init__(self):
        super().__init__('robot_gui_controller_dual')
        self.arm_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.arduino_connected = False
        self.connect_arduino()
        
        self.dynamics = DynamicsModel()
        self.traj_gen = TrajectoryGen(omega_n=3.0, zeta=1.0)
        
        # --- TUNING SIN VIBRACIONES ---
        self.Kp = np.diag([300.0, 300.0, 350.0]) 
        self.Kv = np.diag([5.0, 5.0, 5.0]) # Bajo para evitar ruido
        
        self.q_curr = np.zeros(3); self.dq_curr = np.zeros(3)
        self.q_target_raw = np.zeros(3)
        self.gripper_val = 90
        
        # Estados
        self.motors_enabled = False # Empieza desactivado
        self.initialized_pose = False
        
        self.test_mode = False; self.test_start_time = 0
        self.test_amp = 0.5; self.test_freq = 0.25

        self.ps4_enabled = False; self.joy_vel = np.zeros(3); self.prev_buttons = [0]*20
        self.create_timer(0.05, self.ps4_update_loop)
        self.create_timer(0.02, self.control_loop) # 50Hz
        
        if self.arduino_connected:
            threading.Thread(target=self.serial_read_loop, daemon=True).start()

    def connect_arduino(self):
        ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', 'COM3', 'COM4']
        for p in ports:
            try:
                self.arduino_port = serial.Serial(p, 115200, timeout=0.05)
                # CORRECCIÓN: No borrar el buffer inmediatamente
                time.sleep(2.5) # Esperar reinicio Arduino
                self.arduino_connected = True # Asumimos conectado si abre el puerto
                print(f"✅ Arduino abierto en {p}")
                return
            except: pass
        print("⚠️ Arduino NO detectado")

    def serial_read_loop(self):
        while self.arduino_connected:
            try:
                if self.arduino_port.in_waiting:
                    line = self.arduino_port.readline().decode('utf-8', errors='ignore').strip()
                    # Si recibimos "READY" o datos "FB:", estamos sincronizados
                    if line.startswith("FB:"):
                        parts = line[3:].split(',')
                        if len(parts) >= 3:
                            degs = [float(p) for p in parts]
                            self.q_curr = np.array(degs) * np.pi / 180.0
                            
                            # Sincronización al inicio para evitar saltos
                            if not self.initialized_pose:
                                self.q_target_raw = self.q_curr.copy()
                                self.traj_gen.q = self.q_curr.copy()
                                self.initialized_pose = True
                                print("📍 Posición inicial sincronizada")
            except: pass
            time.sleep(0.002)

    def control_loop(self):
        # Si no hay Arduino o no hemos leído la posición inicial, no hacemos nada
        if not self.arduino_connected or not self.initialized_pose: return

        target = self.q_target_raw.copy()
        
        if self.test_mode:
            t = time.time() - self.test_start_time
            # Ondas suaves
            target[0] = 0.4 * np.sin(2 * np.pi * self.test_freq * t)
            target[1] = 0.4 * np.sin(2 * np.pi * self.test_freq * t + np.pi/2)
            target[2] = 0.4 * np.sin(2 * np.pi * self.test_freq * t)
            self.q_target_raw = target # Actualiza visual

        q_d, dq_d, ddq_d = self.traj_gen.update(target)
        
        # Solo calculamos si los motores están habilitados
        u = np.zeros(3)
        if self.motors_enabled:
            tau = self.dynamics.compute_torque(self.q_curr, self.dq_curr, q_d, dq_d, ddq_d, self.Kp, self.Kv)
            
            K_volts = 80.0 
            # Fricción estática suave para romper inercia
            friction = 6.5 * np.sign(dq_d) 
            u = (tau * K_volts) + friction
            
            # Clamp seguro
            u = np.clip(u, -12.0, 12.0)
            
        # ENVIAR SIEMPRE (aunque sea 0 para mantener vivo el link)
        try:
            cmd = f"CMD:{u[0]:.2f},{u[1]:.2f},{u[2]:.2f},{int(self.gripper_val)}\n"
            self.arduino_port.write(cmd.encode())
        except: pass

        msg = Float64MultiArray(); msg.data = [q_d[0], q_d[1], q_d[2]]
        self.arm_pub.publish(msg)

    # --- GUI CALLBACKS ---
    def toggle_test_mode(self):
        self.test_mode = not self.test_mode
        if self.test_mode:
            self.test_start_time = time.time(); 
            self.motors_enabled = True # Auto-activar motores al testear
            print("🌊 Test ON")
        else:
            self.q_target_raw = np.zeros(3); print("⏹️ Test OFF")

    def toggle_power(self):
        self.motors_enabled = not self.motors_enabled
        # Resincronizar target al activar para evitar saltos
        if self.motors_enabled:
            self.q_target_raw = self.q_curr.copy()
            self.traj_gen.q = self.q_curr.copy()
            print("⚡ MOTORES ACTIVADOS")
        else:
            print("💤 MOTORES DESACTIVADOS")
        return self.motors_enabled

    def update_target(self, idx, val): 
        if not self.test_mode: self.q_target_raw[idx] = val
    def command_close_gripper(self): self.gripper_val = 110
    def command_open_gripper(self): self.gripper_val = 80
    def reset_all(self):
        self.q_target_raw = np.zeros(3); self.test_mode = False
        if self.arduino_connected: self.arduino_port.write(b"RESET\n")
    def destroy_node(self):
        if self.arduino_connected: self.arduino_port.close()
        super().destroy_node()
    def joy_callback(self, msg): 
        if self.ps4_enabled: self.joy_vel[0] = -msg.axes[3]*0.05; self.joy_vel[1] = msg.axes[1]*0.05
    def ps4_update_loop(self):
        if self.ps4_enabled: self.q_target_raw = np.clip(self.q_target_raw + self.joy_vel, -1.57, 1.57)

# ═══════════════════════════════════════════════════════
# 3. INTERFAZ GRÁFICA
# ═══════════════════════════════════════════════════════
THEMES = {'dark': {'bg_main': '#13141f', 'bg_panel': '#1f202e', 'fg_text': '#a9b1d6', 'accent_1': '#7aa2f7', 'target_color': '#f7768e', 'real_color': '#9ece6a', 'grid_color': '#2f334d', 'btn_active': '#9ece6a', 'btn_inactive': '#f7768e'}}

class GraphWidget(tk.Canvas):
    def __init__(self, parent, title):
        super().__init__(parent, bg=THEMES['dark']['bg_panel'], height=250, highlightthickness=0)
        self.title = title; self.data_t = deque([0]*100, maxlen=100); self.data_r = deque([0]*100, maxlen=100)
        self.bind("<Configure>", self.draw)
    def add(self, t, r): self.data_t.append(t); self.data_r.append(r); self.draw()
    def draw(self, event=None):
        self.delete("all"); w=self.winfo_width(); h=self.winfo_height()
        if w<10: return
        self.create_text(10, 10, text=self.title, fill=THEMES['dark']['accent_1'], anchor="nw", font=("Arial", 10, "bold"))
        self.create_line(40, h/2, w, h/2, fill=THEMES['dark']['grid_color'])
        self.plot(self.data_t, THEMES['dark']['target_color'], w, h)
        self.plot(self.data_r, THEMES['dark']['real_color'], w, h)
    def plot(self, data, color, w, h):
        pts = []; dx = (w-50)/99
        for i, y in enumerate(data):
            py = h/2 - (y * h/180.0); pts.extend([40 + i*dx, py])
        if len(pts)>4: self.create_line(pts, fill=color, width=2)

class RobotGUI:
    def __init__(self, ctrl):
        self.c = ctrl; self.root = tk.Tk(); self.root.geometry("1000x800"); self.root.config(bg='#13141f')
        tk.Label(self.root, text="ROBOT MASTER CONTROL", bg='#13141f', fg='white', font=("Arial", 16)).pack(pady=10)
        
        f_btn = tk.Frame(self.root, bg='#13141f'); f_btn.pack(pady=5)
        self.btn_pow = tk.Button(f_btn, text="ACTIVAR MOTORES", command=self.toggle_power, bg=THEMES['dark']['btn_inactive'], width=20, font=("Arial", 10, "bold"))
        self.btn_pow.pack(side=tk.LEFT, padx=5)
        
        self.btn_test = tk.Button(f_btn, text="🌊 TEST SINE", command=self.toggle_test, bg='#339af0', width=15, font=("Arial", 10, "bold"))
        self.btn_test.pack(side=tk.LEFT, padx=5)
        
        tk.Button(f_btn, text="RESET", command=self.c.reset_all, bg='white', width=10).pack(side=tk.LEFT, padx=5)
        
        self.sliders = []
        for i, name in enumerate(["Base", "Hombro", "Codo"]):
            f = tk.Frame(self.root, bg='#13141f'); f.pack(fill=tk.X, padx=20)
            tk.Label(f, text=name, fg='white', bg='#13141f', width=10).pack(side=tk.LEFT)
            s = tk.Scale(f, from_=-90, to=90, orient=tk.HORIZONTAL, bg='#1f202e', fg='white', length=600, command=lambda v, idx=i: self.on_slide(idx, v))
            s.pack(side=tk.LEFT); self.sliders.append(s)
            
        f_g = tk.Frame(self.root, bg='#13141f'); f_g.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.graphs = [GraphWidget(f_g, n) for n in ["M1", "M2", "M3"]]
        for g in self.graphs: g.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)
        
        self.update_gui()
        
    def on_slide(self, idx, v): self.c.update_target(idx, float(v)*math.pi/180.0)
    
    def toggle_power(self):
        st = self.c.toggle_motors()
        self.btn_pow.config(text="MOTORES: ON" if st else "MOTORES: OFF", bg=THEMES['dark']['btn_active'] if st else THEMES['dark']['btn_inactive'])

    def toggle_test(self):
        self.c.toggle_test_mode()
        self.btn_test.config(text="DETENER TEST" if self.c.test_mode else "🌊 TEST SINE")
        if self.c.test_mode: self.btn_pow.config(text="MOTORES: ON", bg=THEMES['dark']['btn_active'])

    def update_gui(self):
        if self.c.test_mode:
            for i, s in enumerate(self.sliders): s.set(self.c.q_target_raw[i]*180/math.pi)
        for i, g in enumerate(self.graphs):
            g.add(self.c.q_target_raw[i]*180/math.pi, self.c.q_curr[i]*180/math.pi)
        self.root.after(50, self.update_gui)
    def run(self): self.root.mainloop()

def main():
    rclpy.init(); c = RobotGUIControllerDual()
    threading.Thread(target=rclpy.spin, args=(c,), daemon=True).start()
    RobotGUI(c).run()
    if c.arduino_connected: c.arduino_port.close()
    c.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()