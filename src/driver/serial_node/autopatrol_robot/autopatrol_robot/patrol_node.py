from geometry_msgs.msg import PoseStamped,Pose,PoseWithCovarianceStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler 
from rclpy.executors import MultiThreadedExecutor
import rclpy
import time

class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        # 声明相关参数
        self.declare_parameter('initial_point',[0.0, 0.0, 0.0])
        self.declare_parameter('target_points',[0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.declare_parameter('buff_point',[0.0, 0.0, 0.0])
        self.declare_parameter('home_point',[0.0, 0.0, 0.0])
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        self.buff_point_ = self.get_parameter('buff_point').value
        self.home_point_ = self.get_parameter('home_point').value

        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        self.last_state_ = None
        self.destination_ = PoseStamped()

        self.initial_pose_publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose',10)
        self.state_subscriber_ = self.create_subscription(String, '/patrol_state', self.sub_callback, 10)
    
    def sub_callback(self,msg):
        if msg.data != self.last_state_:
            self.get_logger().info(f"状态转变为：{msg.data}")
            # self.cancelTask()
        if msg.data == "Go_buff":
            self.last_state_ = "Go_buff"
            self.get_logger().info(f"收到标志位：{msg.data}")
            self.buff_point_ = self.get_parameter('buff_point').value
            self.destination_ = self.get_pose_by_xyyaw(self.buff_point_[0], self.buff_point_[1], self.buff_point_[2])
            self.nav_to_pose(self.destination_)
            
        elif msg.data == "Go_home":
            self.last_state_ = "Go_home"
            self.get_logger().info(f"收到标志位：{msg.data}")
            self.home_point_ = self.get_parameter('home_point').value
            self.destination_ = self.get_pose_by_xyyaw(self.home_point_[0], self.home_point_[1], self.home_point_[2])
            self.nav_to_pose(self.destination_)

    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        通过x y yaw 获得 PoseStamped
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        # 返回： xyzw
        quat = quaternion_from_euler(0,0,yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose
        
    def setInitialPose_(self):
        msg = PoseWithCovarianceStamped()
        pose_temp = self.get_pose_by_xyyaw(self.initial_point_[0], self.initial_point_[1], self.initial_point_[2])
        msg.pose.pose = pose_temp.pose
        msg.header.frame_id = pose_temp.header.frame_id
        msg.header.stamp = pose_temp.header.stamp
        self.get_logger().info(f'发送initialpose x:{pose_temp.pose.position.x} y: {pose_temp.pose.position.y}')
        self.initial_pose_publisher_.publish(msg)
        return

    def nav_to_pose(self, target_point):
        """
        导航到目标点 target_point:PoseStamped()
        """
        self.goToPose(target_point)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            x = target_point.pose.position.x
            y = target_point.pose.position.y
            self.get_logger().info(f'前往：{x,y} 剩余距离：{feedback.distance_remaining}')
            # nav.cancelTask()
        result = self.getResult()
        self.get_logger().info(f'导航结果：{result}')

    def localization_ready(self):
        """
        获取机器人当前位置
        """
        try:
            self.buffer_.lookup_transform(
                'map', 'camera_init', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
            return True
        except Exception as e:
            self.get_logger().warn(f'不能够获取坐标变换，原因: {str(e)}')
            return False
        
def main():
    rclpy.init()
    patrol = PatrolNode()
    # 等待 保证第一次initialpose可以启动
    while not patrol.localization_ready():
        patrol.get_logger().info("定位失败")
        patrol.setInitialPose_()
        rclpy.spin_once(patrol, timeout_sec=0.1)
        time.sleep(0.2)

    patrol.get_logger().info("定位成功")
    
    while rclpy.ok():
        if patrol.destination_ != None and patrol.last_state_ != None:
            patrol.nav_to_pose(patrol.destination_)
        else:
            rclpy.spin_once(patrol, timeout_sec=0.1)
    
    rclpy.shutdown()
