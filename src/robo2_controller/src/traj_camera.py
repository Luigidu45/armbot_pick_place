#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import time
import numpy as np

class YoloPickAndPlace(Node):
    def __init__(self):
        super().__init__('yolo_pick_and_place')
        
        # Publicadores de YOLO
        self.position_pub = self.create_publisher(Point, '/yolo/object_position', 10)
        self.image_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)
        
        # Publicadores del brazo y gripper
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
        
        # Setup YOLO
        self.bridge = CvBridge()
        self.model = YOLO("my_model.pt")
        
        # SOLUCIÓN 1: Configurar cámara con backend específico
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Forzar V4L2
        
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera')
            return
        
        # SOLUCIÓN 2: Configurar ANTES de empezar a capturar
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Verificar qué resolución aceptó la cámara
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f'Camera resolution: {int(actual_width)}x{int(actual_height)} @ {actual_fps} FPS')
        
        # SOLUCIÓN 3: Forzar resize MANUALMENTE si la cámara no respeta
        self.target_width = 320
        self.target_height = 240
        self.force_resize = True  # Siempre hacer resize
        
        # Variables de control
        self.sequence_running = False
        self.last_detection_x = 0.0
        self.last_detection_y = 0.0
        
        # Zona de trigger ajustada para 320x240
        self.x_min = 150
        self.x_max = 170
        self.y_min = 100
        self.y_max = 120
        
        # SOLUCIÓN 4: Procesar solo cada N frames
        self.frame_skip = 1  # Ajusta: 2=mitad de frames, 3=un tercio, etc.
        self.frame_count = 0
        
        # SOLUCIÓN 5: Cache del último frame procesado
        self.last_annotated_frame = None
        
        # Timer optimizado
        self.timer = self.create_timer(0.033, self.detect_objects)  # 30 Hz
        
        self.get_logger().info('YOLO Pick and Place Node initialized (OPTIMIZED)')
        self.get_logger().info(f'Target resolution: {self.target_width}x{self.target_height}')
        self.get_logger().info(f'Trigger zone: X=[{self.x_min}, {self.x_max}], Y=[{self.y_min}, {self.y_max}]')
        self.get_logger().info(f'Frame skip: Processing 1 every {self.frame_skip} frames')

    def send_arm_command(self, base_to_base, base_to_arm, base_to_arm2):
        """Envía comando al brazo"""
        msg = Float64MultiArray()
        msg.data = [base_to_base, base_to_arm, base_to_arm2]
        self.arm_pub.publish(msg)
        self.get_logger().info(f'Arm: [{base_to_base:.2f}, {base_to_arm:.2f}, {base_to_arm2:.2f}]')
    
    def send_gripper_command(self, position):
        """Envía comando al gripper"""
        msg = Float64MultiArray()
        msg.data = [position]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'Gripper: {position:.3f}')

    def detect_objects(self):
        """Detecta objetos con YOLO"""
        if self.sequence_running:
            return
        
        ret, frame = self.cap.read()
        if not ret:
            return

        # SOLUCIÓN 6: Forzar resize SIEMPRE
        if self.force_resize:
            frame = cv2.resize(frame, (self.target_width, self.target_height))
        
        self.frame_count += 1
        
        # SOLUCIÓN 7: Publicar último frame mientras procesamos
        if self.frame_count % self.frame_skip != 0:
            # No procesar, solo publicar último frame conocido
            if self.last_annotated_frame is not None:
                img_msg = self.bridge.cv2_to_imgmsg(self.last_annotated_frame, encoding='bgr8')
                self.image_pub.publish(img_msg)
            return
        
        # Procesar con YOLO
        results = self.model(
            frame,
            imgsz=320,          # IMPORTANTE: Mismo tamaño que el frame
            conf=0.4,           # Confianza más baja para detectar más
            verbose=False,      # No logs
            device='cpu'        # Cambiar a 'cuda:0' si tienes GPU
        )
        
        annotated_frame = results[0].plot()
        annotated_frame = annotated_frame.copy()

        # Procesar detecciones
        boxes = results[0].boxes
        if boxes is not None and len(boxes) > 0:
            x1, y1, x2, y2 = boxes.xyxy[0].tolist()
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2

            self.last_detection_x = cx
            self.last_detection_y = cy

            # Publicar posición
            point_msg = Point()
            point_msg.x = float(cx)
            point_msg.y = float(cy)
            point_msg.z = 0.0
            self.position_pub.publish(point_msg)

            # Dibujar zona de trigger
            cv2.rectangle(annotated_frame, 
                         (self.x_min, self.y_min), 
                         (self.x_max, self.y_max), 
                         (0, 255, 0), 2)
            
            # Verificar trigger
            if (self.x_min <= cx <= self.x_max and 
                self.y_min <= cy <= self.y_max):
                
                cv2.putText(annotated_frame, "TRIGGER!", 
                           (int(cx) - 40, int(cy) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                self.get_logger().info(f'TRIGGER! X={cx:.1f}, Y={cy:.1f}')
                
                self.sequence_running = True
                self.timer.cancel()
                self.execute_pick_and_place()
                return
                
            else:
                # Mostrar distancia
                dx = min(abs(cx - self.x_min), abs(cx - self.x_max))
                dy = min(abs(cy - self.y_min), abs(cy - self.y_max))
                cv2.putText(annotated_frame, f"D:{int(dx)},{int(dy)}", 
                           (int(cx), int(cy) - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        else:
            # Sin detecciones, mostrar zona
            cv2.rectangle(annotated_frame, 
                         (self.x_min, self.y_min), 
                         (self.x_max, self.y_max), 
                         (0, 255, 0), 2)

        # Guardar frame para usar en frames skipeados
        self.last_annotated_frame = annotated_frame

        # Publicar imagen
        img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.image_pub.publish(img_msg)

    def execute_pick_and_place(self):
        """Ejecuta secuencia de pick and place"""
        
        self.get_logger().info('=== Starting Pick and Place ===')
        
        self.send_arm_command(0.0, 0.0, 0.0)
        self.send_gripper_command(0.0)
        time.sleep(3.0)
        
        self.send_arm_command(0.0, -0.8, -0.720)
        time.sleep(3.0)
        
        self.send_gripper_command(0.16)
        time.sleep(2.0)
        
        self.send_arm_command(1.57, 0.0, 0.0)
        time.sleep(5.0)
        
        self.send_arm_command(1.57, -0.8, -0.720)
        time.sleep(3.0)
        
        self.send_gripper_command(0.0)
        time.sleep(2.0)

        self.send_arm_command(1.05, 0.0, 0.0)
        time.sleep(1.5)
        self.send_arm_command(0.52, 0.0, 0.0)
        time.sleep(1.5)
        self.send_arm_command(0.2, 0.0, 0.0)
        time.sleep(1.5)
        self.send_arm_command(0.0, 0.0, 0.0)
        time.sleep(2.0)
        
        self.get_logger().info('=== Pick and Place Completed ===')
        
        self.sequence_running = False
        self.timer = self.create_timer(0.033, self.detect_objects)

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = YoloPickAndPlace()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()