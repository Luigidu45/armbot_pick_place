#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

class FixedConectorTF(Node):
    def __init__(self):
        super().__init__('fixed_conector_tf_broadcaster')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Parámetros de frames
        self.world_frame = 'world'
        self.input_frame = 'joint_arm1_paralel1'         # frame original del URDF
        self.fixed_frame = 'conector_1_fixed'   # nuevo frame sin rotación

        # Publicar a 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.broadcast_fixed_tf)

    def broadcast_fixed_tf(self):
        try:
            # Obtener transform actual de conector_1 respecto al mundo
            trans = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.input_frame,
                rclpy.time.Time()
            )

            # Crear transform nuevo (solo traslación, rotación fija)
            fixed_tf = TransformStamped()
            fixed_tf.header.stamp = self.get_clock().now().to_msg()
            fixed_tf.header.frame_id = self.world_frame
            fixed_tf.child_frame_id = self.fixed_frame
            fixed_tf.transform.translation = trans.transform.translation

            # Rotación fija (sin rotar)
            q = quaternion_from_euler(0.0, 0.0, 0.0)
            fixed_tf.transform.rotation.x = q[0]
            fixed_tf.transform.rotation.y = q[1]
            fixed_tf.transform.rotation.z = q[2]
            fixed_tf.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(fixed_tf)

        except Exception as e:
            self.get_logger().warn_throttle(5.0, f'Esperando transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FixedConectorTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()