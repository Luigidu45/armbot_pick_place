#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk
import threading

class RobotGUIController(Node):
    def __init__(self):
        super().__init__('robot_gui_controller')
        
        # Publicadores
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
        
        # Posiciones actuales
        self.base_to_base = 0.0
        self.base_to_arm = 0.0
        self.base_to_arm2 = 0.0
        self.gripper = 0.0
        
        self.get_logger().info('Robot GUI Controller initialized')
    
    def send_arm_command(self):
        """Envía comando al brazo con las posiciones actuales"""
        msg = Float64MultiArray()
        msg.data = [self.base_to_base, self.base_to_arm, self.base_to_arm2]
        self.arm_pub.publish(msg)
        self.get_logger().info(
            f'Arm: [{self.base_to_base:.2f}, {self.base_to_arm:.2f}, {self.base_to_arm2:.2f}]'
        )
    
    def send_gripper_command(self):
        """Envía comando al gripper"""
        msg = Float64MultiArray()
        msg.data = [self.gripper]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'Gripper: {self.gripper:.2f}')
    
    def update_base_to_base(self, value):
        self.base_to_base = float(value)
        self.send_arm_command()
    
    def update_base_to_arm(self, value):
        self.base_to_arm = float(value)
        self.send_arm_command()
    
    def update_base_to_arm2(self, value):
        self.base_to_arm2 = float(value)
        self.send_arm_command()
    
    def update_gripper(self, value):
        self.gripper = float(value)
        self.send_gripper_command()
    
    def reset_all(self):
        """Resetea todas las juntas a 0"""
        self.base_to_base = 0.0
        self.base_to_arm = 0.0
        self.base_to_arm2 = 0.0
        self.gripper = 0.0
        self.send_arm_command()
        self.send_gripper_command()
        return [0.0, 0.0, 0.0, 0.0]


class RobotGUI:
    def __init__(self, controller):
        self.controller = controller
        
        # Crear ventana principal
        self.root = tk.Tk()
        self.root.title("🤖 Robot Arm Controller")
        self.root.geometry("800x700")
        self.root.configure(bg='#1e1e2e')
        
        # Configurar estilo
        self.setup_style()
        
        # Crear interfaz
        self.create_widgets()
        
    def setup_style(self):
        """Configura el estilo moderno de la interfaz"""
        style = ttk.Style()
        style.theme_use('clam')
        
        # Colores
        bg_color = '#1e1e2e'
        fg_color = '#cdd6f4'
        accent_color = '#89b4fa'
        slider_color = '#74c7ec'
        
        # Configurar estilos
        style.configure('Title.TLabel',
                       background=bg_color,
                       foreground=accent_color,
                       font=('Arial', 24, 'bold'))
        
        style.configure('Joint.TLabel',
                       background=bg_color,
                       foreground=fg_color,
                       font=('Arial', 14, 'bold'))
        
        style.configure('Value.TLabel',
                       background=bg_color,
                       foreground=slider_color,
                       font=('Arial', 12))
        
        style.configure('TScale',
                       background=bg_color,
                       troughcolor='#313244',
                       bordercolor=bg_color,
                       lightcolor=slider_color,
                       darkcolor=slider_color)
        
    def create_widgets(self):
        """Crea todos los widgets de la interfaz"""
        
        # Frame principal
        main_frame = tk.Frame(self.root, bg='#1e1e2e')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        # Título
        title_label = ttk.Label(
            main_frame,
            text="🤖 Robot Arm Controller",
            style='Title.TLabel'
        )
        title_label.pack(pady=(0, 30))
        
        # Contenedor de sliders
        sliders_frame = tk.Frame(main_frame, bg='#1e1e2e')
        sliders_frame.pack(fill=tk.BOTH, expand=True)
        
        # Crear sliders para cada junta
        self.sliders = {}
        self.value_labels = {}
        
        joints = [
            ("🔄 Base Rotation (base_to_base)", "base_to_base", -1.57, 1.57),
            ("📐 Arm Joint 1 (base_to_arm)", "base_to_arm", -1.57, 1.57),
            ("📐 Arm Joint 2 (base_to_arm2)", "base_to_arm2", -1.57, 1.57),
            ("✋ Gripper", "gripper", -1.57, 1.57),
        ]
        
        for i, (label_text, joint_name, min_val, max_val) in enumerate(joints):
            self.create_joint_slider(
                sliders_frame,
                label_text,
                joint_name,
                min_val,
                max_val,
                i
            )
        
        # Frame de botones
        button_frame = tk.Frame(main_frame, bg='#1e1e2e')
        button_frame.pack(pady=30)
        
        # Botón Reset
        reset_button = tk.Button(
            button_frame,
            text="🔄 RESET ALL",
            command=self.reset_all_joints,
            font=('Arial', 14, 'bold'),
            bg='#f38ba8',
            fg='#1e1e2e',
            activebackground='#eba0ac',
            activeforeground='#1e1e2e',
            relief=tk.FLAT,
            padx=30,
            pady=15,
            cursor='hand2'
        )
        reset_button.pack(side=tk.LEFT, padx=10)
        
        # Botón Home Position
        home_button = tk.Button(
            button_frame,
            text="🏠 HOME",
            command=self.go_home,
            font=('Arial', 14, 'bold'),
            bg='#89b4fa',
            fg='#1e1e2e',
            activebackground='#74c7ec',
            activeforeground='#1e1e2e',
            relief=tk.FLAT,
            padx=30,
            pady=15,
            cursor='hand2'
        )
        home_button.pack(side=tk.LEFT, padx=10)
        
        # Botón Preset 1
        preset1_button = tk.Button(
            button_frame,
            text="📍 Preset 1",
            command=self.load_preset1,
            font=('Arial', 12, 'bold'),
            bg='#a6e3a1',
            fg='#1e1e2e',
            activebackground='#94e2d5',
            activeforeground='#1e1e2e',
            relief=tk.FLAT,
            padx=20,
            pady=15,
            cursor='hand2'
        )
        preset1_button.pack(side=tk.LEFT, padx=10)
        
        # Estado
        self.status_label = tk.Label(
            main_frame,
            text="✅ Ready",
            font=('Arial', 10),
            bg='#1e1e2e',
            fg='#a6e3a1'
        )
        self.status_label.pack(pady=10)
        
    def create_joint_slider(self, parent, label_text, joint_name, min_val, max_val, row):
        """Crea un slider para una junta específica"""
        
        # Frame para cada junta
        joint_frame = tk.Frame(parent, bg='#313244', relief=tk.RIDGE, borderwidth=2)
        joint_frame.pack(fill=tk.X, pady=10, ipady=10, ipadx=20)
        
        # Label del nombre de la junta
        label = ttk.Label(
            joint_frame,
            text=label_text,
            style='Joint.TLabel'
        )
        label.pack(anchor=tk.W, pady=(5, 10))
        
        # Frame para slider y valor
        slider_frame = tk.Frame(joint_frame, bg='#313244')
        slider_frame.pack(fill=tk.X, padx=10)
        
        # Label del valor mínimo
        min_label = tk.Label(
            slider_frame,
            text=f"{min_val:.2f}",
            font=('Arial', 9),
            bg='#313244',
            fg='#6c7086'
        )
        min_label.pack(side=tk.LEFT, padx=5)
        
        # Slider
        slider = tk.Scale(
            slider_frame,
            from_=min_val,
            to=max_val,
            resolution=0.01,
            orient=tk.HORIZONTAL,
            length=400,
            bg='#313244',
            fg='#cdd6f4',
            activebackground='#74c7ec',
            highlightthickness=0,
            troughcolor='#45475a',
            sliderrelief=tk.FLAT,
            font=('Arial', 10),
            command=lambda v: self.on_slider_change(joint_name, v)
        )
        slider.set(0.0)
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        
        # Label del valor máximo
        max_label = tk.Label(
            slider_frame,
            text=f"{max_val:.2f}",
            font=('Arial', 9),
            bg='#313244',
            fg='#6c7086'
        )
        max_label.pack(side=tk.LEFT, padx=5)
        
        # Label del valor actual
        value_label = tk.Label(
            joint_frame,
            text="0.00 rad",
            font=('Arial', 12, 'bold'),
            bg='#313244',
            fg='#74c7ec'
        )
        value_label.pack(pady=(10, 5))
        
        self.sliders[joint_name] = slider
        self.value_labels[joint_name] = value_label
        
    def on_slider_change(self, joint_name, value):
        """Callback cuando cambia un slider"""
        value = float(value)
        self.value_labels[joint_name].config(text=f"{value:.2f} rad")
        
        # Actualizar el controlador según la junta
        if joint_name == "base_to_base":
            self.controller.update_base_to_base(value)
        elif joint_name == "base_to_arm":
            self.controller.update_base_to_arm(value)
        elif joint_name == "base_to_arm2":
            self.controller.update_base_to_arm2(value)
        elif joint_name == "gripper":
            self.controller.update_gripper(value)
        
        self.update_status("Moving...")
        self.root.after(500, lambda: self.update_status("✅ Ready"))
    
    def reset_all_joints(self):
        """Resetea todas las juntas a 0"""
        values = self.controller.reset_all()
        
        for i, joint_name in enumerate(["base_to_base", "base_to_arm", "base_to_arm2", "gripper"]):
            self.sliders[joint_name].set(values[i])
            self.value_labels[joint_name].config(text=f"{values[i]:.2f} rad")
        
        self.update_status("🔄 Reset Complete")
        self.root.after(2000, lambda: self.update_status("✅ Ready"))
    
    def go_home(self):
        """Posición home (todo en 0)"""
        self.reset_all_joints()
    
    def load_preset1(self):
        """Carga una posición preset"""
        preset = [0.78, -0.5, 0.3, 0.1]  # Ejemplo de preset
        joint_names = ["base_to_base", "base_to_arm", "base_to_arm2", "gripper"]
        
        for i, joint_name in enumerate(joint_names):
            self.sliders[joint_name].set(preset[i])
            self.value_labels[joint_name].config(text=f"{preset[i]:.2f} rad")
        
        self.controller.base_to_base = preset[0]
        self.controller.base_to_arm = preset[1]
        self.controller.base_to_arm2 = preset[2]
        self.controller.gripper = preset[3]
        self.controller.send_arm_command()
        self.controller.send_gripper_command()
        
        self.update_status("📍 Preset 1 Loaded")
        self.root.after(2000, lambda: self.update_status("✅ Ready"))
    
    def update_status(self, message):
        """Actualiza el mensaje de estado"""
        self.status_label.config(text=message)
    
    def run(self):
        """Ejecuta la interfaz gráfica"""
        self.root.mainloop()


def ros_spin(controller):
    """Función para ejecutar ROS2 spin en un thread separado"""
    rclpy.spin(controller)


def main(args=None):
    rclpy.init(args=args)
    
    # Crear controlador
    controller = RobotGUIController()
    
    # Crear thread para ROS2
    ros_thread = threading.Thread(target=ros_spin, args=(controller,), daemon=True)
    ros_thread.start()
    
    # Crear y ejecutar GUI en el thread principal
    gui = RobotGUI(controller)
    
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()