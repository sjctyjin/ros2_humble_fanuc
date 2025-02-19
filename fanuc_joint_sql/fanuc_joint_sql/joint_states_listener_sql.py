import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

import pyodbc
import numpy as np
import time

class JointStatesToSQL(Node):
    def __init__(self):
        super().__init__('joint_states_to_sql')

        # 1) 建立資料庫連線
        try:
            self.conn = pyodbc.connect(
                'DRIVER={ODBC Driver 17 for SQL Server};'
                'SERVER=192.168.1.101;'   # 資料庫伺服器的 IP 或名稱
                'DATABASE=Fanuc;'    # 資料庫名稱
                'UID=sa;'            # 帳號
                'PWD=pass;'          # 密碼
            )
            self.cursor = self.conn.cursor()
            self.get_logger().info("Successfully connected to SQL Server.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect SQL Server: {e}")

        # 2) 訂閱 /joint_states
        #    這裡預期包含至少 6 個 position (J1 ~ J6)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def joint_callback(self, msg: JointState):
        """
        每次收到 /joint_states 都會觸發此函式。
        """
        # 確保有足夠關節 (至少 6 軸)
        if len(msg.position) < 6:
            self.get_logger().warn("Received joint_states with fewer than 6 positions.")
            return

        # 取得前6軸 (J1~J6) 的弧度值
        joint_poses_rad = np.array(msg.position[:6], dtype=float)
        # 轉成角度
        joint_poses_deg = np.degrees(joint_poses_rad)

        # 根據題示，需要 Z_J3 = J3 - J2 (以角度為單位)
        j1 = round(joint_poses_deg[0], 2)
        j2 = round(joint_poses_deg[1], 2)
        # j3 為 "J3 - J2"
        j3 = round(joint_poses_deg[2] - j2, 2)
        j4 = round(joint_poses_deg[3], 2)
        j5 = round(joint_poses_deg[4], 2)
        j6 = round(joint_poses_deg[5], 2)

        # 取得目前時間
        current_time = time.strftime('%Y-%m-%d %H:%M:%S')

        # 組合 SQL 指令
        sql_query = (
            f"UPDATE PR_Status SET "
            f" X_J1 = '{j1}',"
            f" Y_J2 = '{j2}',"
            f" Z_J3 = '{j3}',"
            f" W_J4 = '{j4}',"
            f" P_J5 = '{j5}',"
            f" R_J6 = '{j6}',"
            f" move='1',"
            f" moveType='joint',"
            f" time='{current_time}'"
            f" WHERE PR_No = 'PR[4]'"
        )

        # 執行更新
        try:
            self.cursor.execute(sql_query)
            self.conn.commit()
            self.get_logger().info(f"SQL update success: {sql_query}")
        except Exception as e:
            self.get_logger().error(f"SQL update failed: {e}")

    def destroy_node(self):
        """
        在節點銷毀前關閉資料庫連線。
        """
        if hasattr(self, 'cursor') and self.cursor:
            self.cursor.close()
        if hasattr(self, 'conn') and self.conn:
            self.conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointStatesToSQL()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
