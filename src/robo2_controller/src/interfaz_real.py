#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk
import threading
import serial
import time

class RobotGUIControllerDual(Node):
    def __init__(self):
        super().__init__('robot_gui_controller_dual')
        
        # ═══════════════════════════════════════════════════════
        # PUBLICADORES ROS2 (GAZEBO)
        # ═══════════════════════════════════════════════════════
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
        
        # ═══════════════════════════════════════════════════════
        # COMUNICACIÓN SERIAL CON ARDUINO
        # ═══════════════════════════════════════════════════════
        self.arduino_connected = False
        self.arduino_port = None
        
        # Intentar conectar con el Arduino
        possible_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        
        for port in possible_ports:
            try:
                self.arduino_port = serial.Serial(
                    port=port,
                    baudrate=115200,
                    timeout=0.1,
                    write_timeout=1.0
                )
                self.arduino_connected = True
                self.get_logger().info(f'✓ Arduino connected via {port}')
                time.sleep(2)  # Esperar que Arduino se inicialice
                
                # Configurar Arduino en modo ABSOLUTO
                self.arduino_port.write(b'A\n')
                time.sleep(0.1)
                
                break
            except Exception as e:
                continue
        
        if not self.arduino_connected:
            self.get_logger().warning('⚠ Arduino not connected. Only Gazebo control active.')
        
        # ═══════════════════════════════════════════════════════
        # POSICIONES ACTUALES
        # ═══════════════════════════════════════════════════════
        self.base_to_base = 0.0
        self.base_to_arm = 0.0
        self.base_to_arm2 = 0.0
        
        # Valores Arduino (Grados)
        self.M1_deg = 0.0
        self.M2_deg = 0.0
        self.M3_deg = 0.0
        self.arduino_gripper_val = 90 # Valor neutral inicial
        
        # Valor Gazebo Gripper (0.0 a 0.19)
        self.gazebo_gripper_val = 0.0 
        
        self.get_logger().info('🤖 Robot GUI Controller DUAL initialized')

    def rad_to_deg(self, rad):
        return rad * 180.0 / 3.14159265359
    
    def send_arm_command(self):
        """Envía comando al brazo en Gazebo"""
        msg = Float64MultiArray()
        msg.data = [self.base_to_base, self.base_to_arm, self.base_to_arm2]
        self.arm_pub.publish(msg)
    
    def send_gripper_to_gazebo(self):
        """Envía comando al gripper en Gazebo"""
        msg = Float64MultiArray()
        msg.data = [self.gazebo_gripper_val]
        self.gripper_pub.publish(msg)

    def send_to_arduino(self):
        """Envía comandos al Arduino usando las posiciones ACTUALES"""
        if not self.arduino_connected:
            return
        
        try:
            # Formato: "M1 M2 M3 Gripper\n"
            # Usamos los valores almacenados en self.M1_deg, etc.
            command = f"{self.M1_deg:.2f} {self.M2_deg:.2f} {self.M3_deg:.2f} {self.arduino_gripper_val}\n"
            self.arduino_port.write(command.encode())
            
            # Limpiar buffer rápido
            if self.arduino_port.in_waiting > 0:
                self.arduino_port.read(self.arduino_port.in_waiting)
            
        except Exception as e:
            self.get_logger().error(f'Error sending to Arduino: {e}')

    # ════════ LOGICA ESPECÍFICA DEL GRIPPER ════════
    
    def command_close_gripper(self):
        """Cierra el gripper (Mantiene fuerza en 110)"""
        self.get_logger().info("✊ Closing Gripper (110)")
        
        # 1. Configurar valores
        self.arduino_gripper_val = 110 # Cerrar
        self.gazebo_gripper_val = 0.0  # Cerrado en Gazebo
        
        # 2. Enviar (Usando la posición actual del brazo para no moverlo)
        self.send_to_arduino()
        self.send_gripper_to_gazebo()

    def command_open_gripper(self):
        """Secuencia de apertura: 80 -> Espera 1s -> 90"""
        
        def open_sequence():
            self.get_logger().info("✋ Opening Gripper (Sequence Start: 80)")
            
            # PASO 1: Abrir (80)
            self.arduino_gripper_val = 80
            self.gazebo_gripper_val = 0.19 # Abierto en Gazebo
            self.send_to_arduino()
            self.send_gripper_to_gazebo()
            
            # PASO 2: Esperar 1 segundo (sin bloquear la GUI)
            time.sleep(0.6)
            
            # PASO 3: Relajar (90)
            self.get_logger().info("✋ Gripper Relaxed (90)")
            self.arduino_gripper_val = 90
            self.send_to_arduino()
            
        # Ejecutar en un hilo separado
        threading.Thread(target=open_sequence, daemon=True).start()

    # ════════ ACTUALIZACIÓN DE JUNTAS ════════

    def update_base_to_base(self, value):
        self.base_to_base = float(value)
        self.M1_deg = self.rad_to_deg(self.base_to_base)
        self.send_arm_command()
        self.send_to_arduino()
    
    def update_base_to_arm(self, value):
        self.base_to_arm = float(value)
        self.M2_deg = self.rad_to_deg(self.base_to_arm)
        self.send_arm_command()
        self.send_to_arduino()
    
    def update_base_to_arm2(self, value):
        self.base_to_arm2 = float(value)
        self.M3_deg = self.rad_to_deg(self.base_to_arm2)
        self.send_arm_command()
        self.send_to_arduino()
    
    def reset_all(self):
        """Resetea todo a 0 y fuerza el movimiento físico"""
        self.get_logger().info('🔄 Resetting all joints...')
        
        # Resetear variables internas
        self.base_to_base = 0.0
        self.base_to_arm = 0.0
        self.base_to_arm2 = 0.0
        self.M1_deg = 0.0
        self.M2_deg = 0.0
        self.M3_deg = 0.0
        self.arduino_gripper_val = 90
        
        # 1. Enviar a Gazebo (Cero perfecto)
        self.send_arm_command()
        self.send_gripper_to_gazebo()
        
        # 2. Enviar a Arduino (TRUCO: Enviar 0.01)
        # Enviamos 0.01 para que el Arduino NO entre en modo "Calibración" (0 0 0)
        # sino que ejecute el PID para MOVER los motores a 0.01 grados (Home físico)
        if self.arduino_connected:
            try:
                # Enviamos el gripper en 90 también
                cmd = f"0.01 0.01 0.01 90\n"
                self.arduino_port.write(cmd.encode())
                self.get_logger().info('✓ Sent Physical Move-Home Command (0.01°)')
            except Exception as e:
                self.get_logger().error(f"Error resetting Arduino: {e}")

        return [0.0, 0.0, 0.0]
    
    def __del__(self):
        if self.arduino_connected and self.arduino_port:
            try:
                # Opcional: Mandar a Home al cerrar
                # self.arduino_port.write(b'0.01 0.01 0.01 90\n') 
                self.arduino_port.close()
            except: pass


class RobotGUI:
    def __init__(self, controller):
        self.controller = controller
        self.root = tk.Tk()
        self.root.title("🤖 Robot Arm Controller DUAL")
        self.root.geometry("850x800")
        self.root.configure(bg='#1e1e2e')
        self.setup_style()
        self.create_widgets()
        
    def setup_style(self):
        style = ttk.Style()
        style.theme_use('clam')
        bg_color = '#1e1e2e'
        fg_color = '#cdd6f4'
        
        style.configure('Title.TLabel', background=bg_color, foreground='#89b4fa', font=('Arial', 24, 'bold'))
        style.configure('Subtitle.TLabel', background=bg_color, foreground='#a6e3a1', font=('Arial', 10))
        style.configure('Joint.TLabel', background=bg_color, foreground=fg_color, font=('Arial', 14, 'bold'))
    
    def create_widgets(self):
        main_frame = tk.Frame(self.root, bg='#1e1e2e')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        # Header
        ttk.Label(main_frame, text="🤖 Robot Arm Controller DUAL", style='Title.TLabel').pack()
        conn_text = "✓ Gazebo + Arduino" if self.controller.arduino_connected else "⚠ Gazebo only"
        ttk.Label(main_frame, text=conn_text, style='Subtitle.TLabel').pack(pady=(0, 20))
        
        # Sliders Container
        sliders_frame = tk.Frame(main_frame, bg='#1e1e2e')
        sliders_frame.pack(fill=tk.BOTH, expand=True)
        
        self.sliders = {}
        self.value_labels = {}
        self.degree_labels = {}
        
        # Solo las 3 juntas del brazo
        joints = [
            ("🔄 Base Rotation (M1)", "base_to_base", -1.57, 1.57),
            ("📐 Arm Joint 1 (M2)", "base_to_arm", -1.57, 1.57),
            ("📐 Arm Joint 2 (M3)", "base_to_arm2", -1.57, 1.57),
        ]
        
        for label_text, joint_name, min_val, max_val in joints:
            self.create_joint_slider(sliders_frame, label_text, joint_name, min_val, max_val)
            
        # ════════ GRIPPER CONTROL ════════
        gripper_frame = tk.Frame(main_frame, bg='#313244', relief=tk.RIDGE, borderwidth=2)
        gripper_frame.pack(fill=tk.X, pady=20, ipady=10)
        
        tk.Label(gripper_frame, text="✋ GRIPPER CONTROL", font=('Arial', 14, 'bold'), 
                 bg='#313244', fg='#f9e2af').pack(pady=5)
        
        btn_container = tk.Frame(gripper_frame, bg='#313244')
        btn_container.pack(pady=10)
        
        # Botón Cerrar
        tk.Button(btn_container, text="✊ CERRAR (110)", command=self.controller.command_close_gripper,
                  font=('Arial', 12, 'bold'), bg='#f38ba8', fg='#1e1e2e', width=20, height=2,
                  activebackground='#eba0ac', cursor='hand2').pack(side=tk.LEFT, padx=20)
        
        # Botón Abrir
        tk.Button(btn_container, text="✋ ABRIR (80->90)", command=self.controller.command_open_gripper,
                  font=('Arial', 12, 'bold'), bg='#a6e3a1', fg='#1e1e2e', width=20, height=2,
                  activebackground='#94e2d5', cursor='hand2').pack(side=tk.LEFT, padx=20)

        # ════════ GENERAL BUTTONS ════════
        button_frame = tk.Frame(main_frame, bg='#1e1e2e')
        button_frame.pack(pady=10)
        
        tk.Button(button_frame, text="🔄 RESET ALL (Move Home)", command=self.reset_all_joints,
                  font=('Arial', 12, 'bold'), bg='#fab387', fg='#1e1e2e', padx=20, pady=10).pack(side=tk.LEFT, padx=10)
        
        tk.Button(button_frame, text="📍 Preset 1", command=self.load_preset1,
                  font=('Arial', 12, 'bold'), bg='#89b4fa', fg='#1e1e2e', padx=20, pady=10).pack(side=tk.LEFT, padx=10)

        self.status_label = tk.Label(main_frame, text="✅ Ready", font=('Arial', 10), bg='#1e1e2e', fg='#a6e3a1')
        self.status_label.pack(pady=5)

    def create_joint_slider(self, parent, label_text, joint_name, min_val, max_val):
        frame = tk.Frame(parent, bg='#313244', relief=tk.RIDGE, borderwidth=2)
        frame.pack(fill=tk.X, pady=5, ipady=5, ipadx=10)
        
        ttk.Label(frame, text=label_text, style='Joint.TLabel').pack(anchor=tk.W)
        
        val_frame = tk.Frame(frame, bg='#313244')
        val_frame.pack(fill=tk.X)
        
        slider = tk.Scale(val_frame, from_=min_val, to=max_val, resolution=0.01, orient=tk.HORIZONTAL,
                          bg='#313244', fg='#cdd6f4', activebackground='#74c7ec', highlightthickness=0,
                          troughcolor='#45475a', length=400, showvalue=0,
                          command=lambda v: self.on_slider_change(joint_name, v))
        slider.set(0.0)
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        
        self.value_labels[joint_name] = tk.Label(val_frame, text="0.00 rad", width=8, bg='#313244', fg='#74c7ec', font=('Arial', 11))
        self.value_labels[joint_name].pack(side=tk.LEFT)
        
        self.degree_labels[joint_name] = tk.Label(val_frame, text="(0.00°)", width=8, bg='#313244', fg='#a6e3a1', font=('Arial', 10))
        self.degree_labels[joint_name].pack(side=tk.LEFT)
        self.sliders[joint_name] = slider

    def on_slider_change(self, joint_name, value):
        val = float(value)
        self.value_labels[joint_name].config(text=f"{val:.2f}")
        self.degree_labels[joint_name].config(text=f"({val*180/3.14159:.1f}°)")
        
        if joint_name == "base_to_base": self.controller.update_base_to_base(val)
        elif joint_name == "base_to_arm": self.controller.update_base_to_arm(val)
        elif joint_name == "base_to_arm2": self.controller.update_base_to_arm2(val)

    def reset_all_joints(self):
        self.controller.reset_all()
        for j in ["base_to_base", "base_to_arm", "base_to_arm2"]:
            self.sliders[j].set(0.0)
            
    def load_preset1(self):
        preset = [0.0, -0.5, 1.0] 
        self.sliders["base_to_base"].set(preset[0])
        self.sliders["base_to_arm"].set(preset[1])
        self.sliders["base_to_arm2"].set(preset[2])
        self.controller.update_base_to_base(preset[0])
        self.controller.update_base_to_arm(preset[1])
        self.controller.update_base_to_arm2(preset[2])

    def run(self):
        self.root.mainloop()

def ros_spin(controller):
    rclpy.spin(controller)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotGUIControllerDual()
    ros_thread = threading.Thread(target=ros_spin, args=(controller,), daemon=True)
    ros_thread.start()
    gui = RobotGUI(controller)
    try: gui.run()
    except KeyboardInterrupt: pass
    finally:
        if controller.arduino_connected: controller.reset_all()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()