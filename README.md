# VIPER: Brazo Robótico para Clasificación de Residuos Electrónicos (E-Waste)

VIPER es un ecosistema robótico desarrollado en **ROS 2 Jazzy** diseñado para tareas de *Pick and Place* automatizadas. El proyecto integra un diseño mecánico avanzado, una simulación de alta fidelidad que actúa como gemelo digital y un sistema de percepción basado en **YOLOv11** entrenado específicamente para la detección y clasificación de componentes electrónicos.

<div align="center">
  <img src="docs/media/demo_1.gif" width="48%" />
  <img src="docs/media/demo_2.gif" width="48%" />
  <p><em>Fragmentos del funcionamiento del sistema en tiempo real.</em></p>
</div>

---

## 🚀 Características Principales

### 1. Diseño Mecánico y URDF
El brazo cuenta con un diseño cinemático optimizado para paralelogramos, permitiendo mantener la orientación del actuador final. El modelo está definido mediante archivos **Xacro** modulares, incluyendo:
*   **Mimic Joints:** Configuración avanzada de juntas para modelar el comportamiento de los eslabones paralelos.
*   **Mallas STL:** Geometrías precisas para colisiones y visualización.
*   **Inerciales:** Propiedades físicas calculadas para una respuesta dinámica realista.

<div align="center">
  <img src="docs/media/urdf_view.png" width="70%" />
  <p><em>Visualización del modelo URDF en RViz2.</em></p>
</div>

### 2. Simulación en Gazebo (Gemelo Digital)
Utilizando **Gazebo Sim (Harmonic)** y el motor de física **Bullet**, la simulación sirve como un gemelo digital exacto del hardware físico. 
*   **ros2_control:** Implementación de `gz_ros2_control` para el manejo de hardware interfaces.
*   **Controladores:** Uso de `JointStateBroadcaster` y `ForwardCommandController` (Position) para un control preciso.
*   **Sincronización:** Los comandos enviados a la simulación pueden ser replicados en el brazo físico mediante una comunicación serial (ESP32).

<div align="center">
  <img src="docs/media/gazebo_sim.png" width="70%" />
  <p><em>Entorno de simulación con el brazo y los contenedores de reciclaje.</em></p>
</div>

### 3. Percepción con YOLOv11 en ROS 2
El sistema de visión utiliza una red neuronal **YOLOv11** personalizada y entrenada para identificar tres clases críticas de residuos:
*   **Pila (Battery)**
*   **Motor (Motor)**
*   **Placa (PCB)**

**Nodos de Percepción:**
*   `/yolo/detection_image`: Tópico de imagen con *bounding boxes* y etiquetas.
*   `/yolo/detections`: Mensajes personalizados que contienen la clase, confianza y coordenadas (X, Y) para el cálculo de la trayectoria de *pick*.

<div align="center">
  <img src="docs/media/yolo_detection.png" width="70%" />
  <p><em>Detección de componentes electrónicos mediante YOLOv11.</em></p>
</div>

### 4. Interfaz de Control (GUI)
Interfaz desarrollada para la monitorización y control manual del brazo, permitiendo el ajuste de ángulos de las juntas y la activación del gripper de forma intuitiva.

<div align="center">
  <img src="docs/media/control_gui.png" width="70%" />
  <p><em>Panel de control del operador.</em></p>
</div>

---

## 🛠️ Stack Tecnológico
*   **OS:** Ubuntu 24.04 LTS
*   **Middleware:** ROS 2 Jazzy Jalisco
*   **Simulador:** Gazebo Harmonic (GZ Sim)
*   **Visión Artificial:** YOLOv11 (Ultralytics)
*   **Lenguajes:** Python 3, C++
*   **Hardware Sync:** Micro-ROS / Serial Communication (ESP32)

---

## 📂 Estructura del Repositorio
*   `src/robot_description`: Archivos URDF/Xacro, mallas STL y configuración del mundo en Gazebo.
*   `src/robo2_controller`: Configuración de `ros2_control`, parámetros de los controladores y nodos de trayectoria.
*   `src/robo2_moveit`: (Opcional) Configuración para la planificación de trayectorias complejas.

---

## 🔧 Instalación y Ejecución

1.  **Clonar el repositorio:**
    ```bash
    mkdir -p ~/viper_ws/src
    cd ~/viper_ws/src
    git clone https://github.com/tu-usuario/viper.git
    ```
2.  **Instalar dependencias:**
    ```bash
    cd ~/viper_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
3.  **Compilar:**
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```
4.  **Lanzar Simulación:**
    ```bash
    ros2 launch robot_description gazebo.launch.py
    ```
5.  **Lanzar Controladores:**
    ```bash
    ros2 launch robo2_controller controller.launch.py
    ```

---

