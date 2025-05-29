#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointRelayNode(Node):
    def __init__(self):
        super().__init__('joint_relay_node')

        self.joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6','gripper']

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.publisher_custom = self.create_publisher(JointState, '/joint_custom_state', 10)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_custom_state',
            self.joint_callback,
            10
        )
        self.init_timer = 0

        self.latest_joint_state = None
        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # ➕ 啟動後立即發一次初始化的 joint_states
        self.initial_position_sent = False
        self.create_timer(0.5, self.publish_initial_joint_state)

        self.get_logger().info('✅ joint_relay_node 啟動，定時轉發 /joint_custom_state → /joint_states')

    def publish_initial_joint_state(self):
        if self.initial_position_sent:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.publisher.publish(msg)
        self.publisher_custom.publish(msg)
        self.init_timer += 1
        self.get_logger().info("🚀 已發送{self.init_timer}次初始 JointState 訊息")
        if self.init_timer >= 4:
            self.initial_position_sent = True  # 防止再次觸發
        

    def joint_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def timer_callback(self):
        if self.latest_joint_state is None:
            return

        relay_msg = JointState()
        relay_msg.header.stamp = self.get_clock().now().to_msg()
        relay_msg.name = self.joint_names

        input_position = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        relay_msg.position = [input_position.get(name, 0.0) for name in self.joint_names]

        relay_msg.velocity = [0.0] * len(self.joint_names)
        relay_msg.effort = [0.0] * len(self.joint_names)

        self.publisher.publish(relay_msg)
        self.get_logger().debug(f"🔄 每0.2秒轉發一次 JointState: {relay_msg.position}")

def main(args=None):
    rclpy.init(args=args)
    node = JointRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

