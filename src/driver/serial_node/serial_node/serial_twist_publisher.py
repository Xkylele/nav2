import rclpy
import serial
import struct
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String



class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        # 参数配置
        self.declare_parameter('port', '/dev/ttyCH341USB1')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('linear_scale', 1000.0)  
        self.declare_parameter('angular_scale', 1000.0)  

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value
        
                # 初始化串口
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0)
            self.get_logger().info(f'串口 {port} 已打开，波特率 {baudrate}')
        except Exception as e:
            self.get_logger().error(f'无法打开串口: {e}')
            raise

        # 订阅 /cmd_vel 话题
        self.subscription = self.create_subscription(
            Twist,           # 消息类型
            '/cmd_vel',      # 话题名称
            self.listener_callback,  # 回调函数
            10               # QoS profile depth
        )
        self.get_logger().info("正在监听 /cmd_vel 话题...")

        self.publisher_ = self.create_publisher(
            String,
            'patrol_state',
            10
        )
        self.timer_ = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        # 如果缓存区 有数据
        if self.ser.in_waiting > 0:
            data = self.ser.read(1)
            value = data[0]
            self.get_logger().info(f"接收到：{value}")

            if value == 0x01:
                msg = String()
                msg.data = "Go_buff"
                self.publisher_.publish(msg)
            elif value == 0x02:
                msg = String()
                msg.data = "Go_home"
                self.publisher_.publish(msg)

    def listener_callback(self, msg):
        # # 提取线速度和角速度
        # linear_x = msg.linear.x
        # angular_z = msg.angular.z

        # 提取线速度和角速度
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # 缩放并转换为整数（int16）
        speed_x = int(vx * self.linear_scale)
        speed_y = int(vy * self.linear_scale)
        angular = int(wz * self.angular_scale)

        # 限制在 int16 范围内 (-32768 ~ 32767)
        speed_x = max(min(speed_x, 0x7FFF), -0x8000)
        speed_y = max(min(speed_y, 0x7FFF), -0x8000)
        angular = max(min(angular, 0x7FFF), -0x8000)

        # 转换为 2 字节（有符号，大端）
        speed_bytes_x = struct.pack('>h', speed_x)  # > 表示大端，h 表示 int16
        speed_bytes_y = struct.pack('>h', speed_y)  # > 表示大端，h 表示 int16
        angular_bytes = struct.pack('>h', angular)

        # 构造数据包
        packet = bytes([
            0xFF,
            speed_bytes_x[0], speed_bytes_x[1],
            speed_bytes_y[0], speed_bytes_y[1],
            angular_bytes[0], angular_bytes[1],
            0xFE
        ])

        # # 发送数据
        # self.ser.write(packet)
        # self.get_logger().debug(f'发送数据包: {packet.hex()}')
        try:
            self.ser.write(packet)
            self.get_logger().info(f'发送数据包: {packet.hex()}')
        except Exception as e:
            self.get_logger().error(f'串口写入失败: {e}')


        # 打印出来
        # self.get_logger().info(f'接收到速度指令: 线速度 x={vx:.2f}, y={vy:.2f} 角速度 z={wz:.2f}')


def main(args=None):
    rclpy.init(args=args)

    cmd_vel_subscriber = CmdVelSubscriber()

    try:
        rclpy.spin(cmd_vel_subscriber)
    except KeyboardInterrupt:
        pass


    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()