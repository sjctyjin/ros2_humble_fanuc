#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import pymssql
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation as R

class SQLToROSNode(Node):
    def __init__(self):
        super().__init__('sql_to_ros_node')
        
        # 1) 建立資料庫連線 (改為 pymssql)
        try:
            self.conn = pymssql.connect(
                server='192.168.3.105',
                user='sa',
                password='pass',
                database='Fanuc'
            )
            self.cursor = self.conn.cursor()
            self.get_logger().info("✅ Successfully connected to SQL Server using pymssql.")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect SQL Server: {e}")
            self.conn = None
            self.cursor = None
        
        # 2) 訂閱 /joint_states 寫入 SQL（預設先註解）
        self.subscription = self.create_subscription(
             JointState,
             '/joint_states',
             self.joint_callback,
             10
         )
        
        # 3) 發布末端位姿
        self.end_pose_pub = self.create_publisher(Pose, '/end_pose', 10)
        
        # 4) 發布關節狀態
        self.joint_ctrl_single_pub = self.create_publisher(JointState, '/joint_ctrl_single', 10)
        
        # 5) 建立定時器每 0.1 秒查詢 SQL 並發布資料
        self.timer = self.create_timer(0.1, self.read_sql_and_publish)
        
        self.joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'gripper']
    
    def joint_callback(self, msg: JointState):
        """接收 /joint_states 並寫入 SQL"""
        if not self.cursor:
            return
            
        if len(msg.position) < 6:
            self.get_logger().warn("Received joint_states with fewer than 6 positions.")
            return
        
        joint_poses_rad = np.array(msg.position[:6], dtype=float)
        joint_poses_deg = np.degrees(joint_poses_rad)
        
        j1 = round(joint_poses_deg[0], 2)
        j2 = round(joint_poses_deg[1], 2)
        j3 = round(joint_poses_deg[2] - j2, 2)  # J3 - J2
        j4 = round(joint_poses_deg[3], 2)
        j5 = round(joint_poses_deg[4], 2)
        j6 = round(joint_poses_deg[5], 2)
        
        current_time = time.strftime('%Y-%m-%d %H:%M:%S')
        
        sql_query = (
            "UPDATE PR_Status SET "
            f"X_J1 = '{j1}', "
            f"Y_J2 = '{j2}', "
            f"Z_J3 = '{j3}', "
            f"W_J4 = '{j4}', "
            f"P_J5 = '{j5}', "
            f"R_J6 = '{j6}', "
            "move = '1', "
            "moveType = 'joint', "
            f"time = '{current_time}' "
            "WHERE PR_No = 'PR[4]'"
        )
        
        try:
            self.cursor.execute(sql_query)
            self.conn.commit()
            self.get_logger().debug("✅ SQL update success for PR_Status")
        except Exception as e:
            self.get_logger().error(f"❌ SQL update failed: {e}")
    
    def read_sql_and_publish(self):
        """從 SQL 讀取資料並發布至 ROS topic"""
        if not self.cursor:
            return
            
        try:
            self.cursor.execute("""
                SELECT [X], [Y], [Z], [W], [P], [R], [J1], [J2], [J3], [J4], [J5], [J6], [time]
                FROM [Coord_Status]
            """)
            row = self.cursor.fetchone()
            
            if not row:
                self.get_logger().warn("⚠️ No data found in Coord_Status table")
                return
            
            x, y, z, w, p, r, j1, j2, j3, j4, j5, j6, timestamp = row
            
            # 發布末端位姿
            endpos = Pose()
            endpos.position.x = float(x) / 1000
            endpos.position.y = float(y) / 1000
            endpos.position.z = float(z) / 1000

            roll = math.radians(float(w) )
            pitch = math.radians(float(p) )
            yaw = math.radians(float(r) )

            quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
            endpos.orientation.x, endpos.orientation.y, endpos.orientation.z, endpos.orientation.w = quat
            self.end_pose_pub.publish(endpos)
            
            # 發布關節值
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self.joint_names
            joint_state.position = [
                math.radians(float(j1)),
                math.radians(float(j2)),
                math.radians(float(j3)),
                math.radians(float(j4)),
                math.radians(float(j5)),
                math.radians(float(j6)),
                0.0  # gripper 預設值
            ]
            self.joint_ctrl_single_pub.publish(joint_state)
            self.get_logger().debug(f"Published pose and joint data from SQL (timestamp: {timestamp})")
        except Exception as e:
            self.get_logger().error(f"❌ Error reading from SQL or publishing: {e}")
    
    def destroy_node(self):
        """在節點銷毀前關閉資料庫連線"""
        if hasattr(self, 'cursor') and self.cursor:
            self.cursor.close()
        if hasattr(self, 'conn') and self.conn:
            self.conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SQLToROSNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

