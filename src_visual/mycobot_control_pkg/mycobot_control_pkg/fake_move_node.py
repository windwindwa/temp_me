import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class FakeMoveNode(Node):
    def __init__(self):
        super().__init__('fake_move_node')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        self.current_positions = [0.0] * 6
        self.target_positions = [0.3, -0.2, 0.1, -0.3, 0.5, 0.0]
        self.step_size = 0.01

        self.timer = self.create_timer(0.05, self.update_joint_states)

    def update_joint_states(self):
        updated = False
        for i in range(6):
            delta = self.target_positions[i] - self.current_positions[i]
            if abs(delta) > self.step_size:
                self.current_positions[i] += self.step_size if delta > 0 else -self.step_size
                updated = True
            else:
                self.current_positions[i] = self.target_positions[i]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        msg.position = self.current_positions
        self.publisher_.publish(msg)

        if not updated:
            self.get_logger().info("到达目标点！")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = FakeMoveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
