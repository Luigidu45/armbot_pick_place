#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from geometry_msgs.msg import Point
from std_msgs.msg import String  # <--- 1. IMPORTAMOS STRING
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Publicadores de YOLO
        self.position_pub = self.create_publisher(Point, '/yolo/object_position', 10)
        self.class_pub = self.create_publisher(String, '/yolo/object_class', 10) # <--- 2. NUEVO PUBLICADOR
        self.image_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)
        
        # Setup YOLO
        self.bridge = CvBridge()
        # Asegúrate de que este path sea correcto
        self.model = YOLO("my_model.pt")
        
        # Configurar cámara con backend específico
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Forzar V4L2
        
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera')
            return
        
        # Configurar ANTES de empezar a capturar
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
        
        # Forzar resize MANUALMENTE si la cámara no respeta
        self.target_width = 320
        self.target_height = 240
        self.force_resize = True
        
        # Procesar solo cada N frames
        self.frame_skip = 1 
        self.frame_count = 0
        
        # Cache del último frame procesado
        self.last_annotated_frame = None
        
        # Timer optimizado
        self.timer = self.create_timer(0.033, self.detect_objects)  # 30 Hz
        
        self.get_logger().info('YOLO Detector initialized (OPTIMIZED)')

    def detect_objects(self):
        """Detecta objetos con YOLO"""
        
        ret, frame = self.cap.read()
        if not ret:
            return

        # Forzar resize SIEMPRE
        if self.force_resize:
            frame = cv2.resize(frame, (self.target_width, self.target_height))
        
        self.frame_count += 1
        
        # Publicar último frame mientras procesamos (para mantener fluidez visual)
        if self.frame_count % self.frame_skip != 0:
            if self.last_annotated_frame is not None:
                img_msg = self.bridge.cv2_to_imgmsg(self.last_annotated_frame, encoding='bgr8')
                self.image_pub.publish(img_msg)
            return
        
        # Procesar con YOLO
        results = self.model(
            frame,
            imgsz=320,
            conf=0.4,
            verbose=False,
            device='cpu' 
        )
        
        # Anotar imagen
        annotated_frame = results[0].plot()
        
        # Procesar detecciones
        boxes = results[0].boxes
        
        # Obtenemos el diccionario de nombres del modelo {0: 'bateria', 1: 'motor', etc}
        names = results[0].names 
        
        if boxes is not None and len(boxes) > 0:
            # --- DATOS DEL PRIMER OBJETO DETECTADO ---
            # Coordenadas
            x1, y1, x2, y2 = boxes.xyxy[0].tolist()
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2

            # Clase (ID numérico)
            class_id = int(boxes.cls[0].item())
            # Nombre (String) recuperado del diccionario del modelo
            class_name = names[class_id]

            # 1. Publicar posición
            point_msg = Point()
            point_msg.x = float(cx)
            point_msg.y = float(cy)
            point_msg.z = 0.0
            self.position_pub.publish(point_msg)
            
            # 2. Publicar clase (TEXTO) <--- AQUÍ ESTÁ LO NUEVO
            class_msg = String()
            class_msg.data = class_name
            self.class_pub.publish(class_msg)
            
            # (Opcional) Log para ver qué detecta en terminal
            # self.get_logger().info(f"Detectado: {class_name} en ({cx:.1f}, {cy:.1f})")

        # Guardar frame para frames skipeados
        self.last_annotated_frame = annotated_frame

        # Publicar imagen
        img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.image_pub.publish(img_msg)

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()