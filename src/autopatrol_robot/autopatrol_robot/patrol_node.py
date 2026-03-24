from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_from_euler
import rclpy
import time


class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)

        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.declare_parameter('buff_point', [0.0, 0.0, 0.0])
        self.declare_parameter('home_point', [0.0, 0.0, 0.0])

        self.initial_point_ = self.get_parameter('initial_point').value
        self.buff_point_ = self.get_parameter('buff_point').value
        self.home_point_ = self.get_parameter('home_point').value

        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)

        self.current_state_ = None   # 当前正在执行的状态
        self.pending_state_ = None   # sub_callback 收到的新状态，等待主循环处理

        self.initial_pose_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.state_subscriber_ = self.create_subscription(
            String, '/patrol_state', self.sub_callback, 10)

    def sub_callback(self, msg):
        """
        只更新 pending_state_，不做任何导航操作
        主循环负责检测状态变化并切换导航目标
        """
        new_state = msg.data
        if new_state != self.pending_state_:
            self.get_logger().info(f"收到新状态：{new_state}")
            self.pending_state_ = new_state

    def get_pose_by_xyyaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def setInitialPose_(self):
        msg = PoseWithCovarianceStamped()
        pose_temp = self.get_pose_by_xyyaw(
            self.initial_point_[0], self.initial_point_[1], self.initial_point_[2])
        msg.pose.pose = pose_temp.pose
        msg.header.frame_id = pose_temp.header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            f'发送initialpose x:{pose_temp.pose.position.x} y:{pose_temp.pose.position.y}')
        self.initial_pose_publisher_.publish(msg)

    def localization_ready(self):
        try:
            self.buffer_.lookup_transform(
                'map', 'odom',
                rclpy.time.Time(seconds=0),
                rclpy.time.Duration(seconds=1))
            return True
        except Exception as e:
            self.get_logger().warn(f'不能获取坐标变换：{str(e)}')
            return False

    def get_destination_by_state(self, state):
        """根据状态返回目标 PoseStamped"""
        if state == "Go_buff":
            self.buff_point_ = self.get_parameter('buff_point').value
            return self.get_pose_by_xyyaw(
                self.buff_point_[0], self.buff_point_[1], self.buff_point_[2])
        elif state == "Go_home":
            self.home_point_ = self.get_parameter('home_point').value
            return self.get_pose_by_xyyaw(
                self.home_point_[0], self.home_point_[1], self.home_point_[2])
        return None


def main():
    rclpy.init()
    patrol = PatrolNode()

    # 等待定位就绪
    while not patrol.localization_ready():
        patrol.get_logger().info("等待定位...")
        patrol.setInitialPose_()
        rclpy.spin_once(patrol, timeout_sec=0.1)
        time.sleep(0.2)

    patrol.get_logger().info("定位成功，等待指令...")

    navigating = False  # 当前是否正在导航

    while rclpy.ok():
        # 处理一次消息（让 sub_callback 有机会更新 pending_state_）
        rclpy.spin_once(patrol, timeout_sec=0.05)

        # 检测到新状态
        if patrol.pending_state_ != patrol.current_state_:
            new_state = patrol.pending_state_

            # 取消当前任务
            if navigating:
                patrol.get_logger().info(f"状态切换：{patrol.current_state_} → {new_state}，取消当前任务")
                patrol.cancelTask()
                navigating = False

            # 启动新导航
            destination = patrol.get_destination_by_state(new_state)
            if destination is not None:
                patrol.get_logger().info(f"开始导航到 {new_state}：({destination.pose.position.x}, {destination.pose.position.y})")
                patrol.goToPose(destination)
                navigating = True
                patrol.current_state_ = new_state
            else:
                patrol.get_logger().warn(f"未知状态：{new_state}，忽略")
                patrol.current_state_ = new_state

        # 检查导航是否完成
        if navigating and patrol.isTaskComplete():
            result = patrol.getResult()
            if result == TaskResult.SUCCEEDED:
                patrol.get_logger().info(f"到达目标：{patrol.current_state_}")
            elif result == TaskResult.CANCELED:
                patrol.get_logger().info("任务已取消")
            else:
                patrol.get_logger().warn(f"导航失败：{result}")
            navigating = False

    rclpy.shutdown()