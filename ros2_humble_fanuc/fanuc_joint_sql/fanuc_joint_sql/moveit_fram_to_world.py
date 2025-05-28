import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np


class TransformObjectToBase(Node):
    def __init__(self):
        super().__init__('transform_object_to_base')

        # TF2 Buffer 和 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 座標系名稱
        self.object_frame = 'object_frame'  # 物體座標系 (來自 YOLO 輸出的 TF)
        self.camera_frame = 'camera_link'  # 相機座標系
        self.link6_frame = 'link_6'  # link6 座標系
        self.base_frame = 'fanuc_world'  # 基座座標系

        # 啟動定時器，每 1 秒執行一次
        self.timer = self.create_timer(1.0, self.transform_object_to_base)

    def transform_object_to_base(self):
        try:
            # 1. 取得 object_frame 到 camera_link 的 TF 變換
            transform_camera_to_object = self.tf_buffer.lookup_transform(
                self.camera_frame, self.object_frame, rclpy.time.Time()
            )
            # 從 TF 提取平移座標 (x, y, z)
            object_point_camera = np.array([
                transform_camera_to_object.transform.translation.x,
                transform_camera_to_object.transform.translation.y,
                transform_camera_to_object.transform.translation.z,
                1.0  # 齊次座標
            ])

            # 2. 取得 camera_link 到 link6 的變換
            transform_link6_to_camera = self.tf_buffer.lookup_transform(
                self.link6_frame, self.camera_frame, rclpy.time.Time()
            )
            T_link6_to_camera = self.transform_to_matrix(transform_link6_to_camera)

            # 3. 取得 base_link 到 link6 的變換
            transform_base_to_link6 = self.tf_buffer.lookup_transform(
                self.base_frame, self.link6_frame, rclpy.time.Time()
            )
            T_base_to_link6 = self.transform_to_matrix(transform_base_to_link6)

            # 4. 合併變換矩陣：base_link <- link6 <- camera_link
            T_base_to_camera = np.dot(T_base_to_link6, T_link6_to_camera)

            # 5. 將物體座標轉換到 base_link
            point_base = np.dot(T_base_to_camera, object_point_camera)
            # 6. 廣播新的 TF
            self.broadcast_object_tf(point_base[:3])

            # 輸出結果
            self.get_logger().info(f"Object point in base_link: {point_base[:3]}")

        except Exception as e:
            self.get_logger().error(f"Failed to transform object point: {str(e)}")

    def transform_to_matrix(self, transform: TransformStamped):
        """ 將 TF 變換轉換為 4x4 齊次變換矩陣 """
        trans = transform.transform.translation
        rot = transform.transform.rotation

        # 旋轉矩陣 (四元數轉換)
        q = [rot.x, rot.y, rot.z, rot.w]
        R = self.quaternion_to_rotation_matrix(q)

        # 平移向量
        T = np.array([[R[0, 0], R[0, 1], R[0, 2], trans.x],
                      [R[1, 0], R[1, 1], R[1, 2], trans.y],
                      [R[2, 0], R[2, 1], R[2, 2], trans.z],
                      [0, 0, 0, 1]])
        return T

    def quaternion_to_rotation_matrix(self, q):
        """ 將四元數轉換為旋轉矩陣 """
        x, y, z, w = q
        R = np.array([
            [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x ** 2 - 2 * y ** 2]
        ])
        return R

    def broadcast_object_tf(self, position):
        """ 廣播物體的 TF 到 base_link """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame  # 基座座標系
        t.child_frame_id = 'object_in_base'  # 新的物體 TF 名稱

        # 平移部分
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        # 假設旋轉為單位四元數 (物體沒有旋轉)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # 發布 TF
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published object TF at {position}")


def main(args=None):
    rclpy.init(args=args)
    node = TransformObjectToBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

