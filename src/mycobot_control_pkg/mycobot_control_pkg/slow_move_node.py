import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class SlowMoveNode(Node):
    def __init__(self):
        super().__init__('slow_move_node')
        self.declare_parameter('goal', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                               ParameterDescriptor(description="目标角度"))
        self.declare_parameter('duration', 5,
                               ParameterDescriptor(description="运动时长（秒）"))
        self.goal_positions = self.get_parameter('goal').get_parameter_value().double_array_value
        self.duration = self.get_parameter('duration').get_parameter_value().integer_value

        if len(self.goal_positions) != 6:
            self.get_logger().error("目标位置必须包含 6 个关节角度")
            rclpy.shutdown()
            return

        self.publisher_ = self.create_publisher(JointTrajectory, '/mycobot_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.send_trajectory)

    def send_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=self.duration)
        traj.points.append(point)
        self.publisher_.publish(traj)
        self.get_logger().info(f'发布轨迹：{self.goal_positions}，时长：{self.duration} 秒')
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = SlowMoveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
