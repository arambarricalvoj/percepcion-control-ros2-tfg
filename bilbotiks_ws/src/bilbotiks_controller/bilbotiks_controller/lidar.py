import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
import ydlidar
from sensor_msgs.msg import LaserScan
import math

class LidarArgitaratzailea(LifecycleNode):
    def __init__(self):
        super().__init__('lidar_argitaratzailea')
        self.lidar = None
        self.port = None
        self.baudrate = None
        self.lidar_type = None
        self.device_type = None
        self.frame_id = None
        self.scan_frequency = None
        self.sample_rate = None
        self.angle_min = None
        self.angle_max = None
        self.range_min = None
        self.range_max = None
        self.single_channel = None

        self.publisher_ = None
        self.timer_ = None

        self.active = None

        # Parametroak
        self.declare_parameter("port", "/dev/ydlidar")
        #self.declare_parameter("port", "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0")
        self.declare_parameter("baudrate", 128000)
        self.declare_parameter("lidar_type", ydlidar.TYPE_TRIANGLE)
        self.declare_parameter("device_type", ydlidar.YDLIDAR_TYPE_SERIAL)
        self.declare_parameter("frame_id", "laser_frame")
        self.declare_parameter("scan_frequency", 10.0) # 6 - 12 Hz
        self.declare_parameter("sample_rate", 5)
        self.declare_parameter("angle_min", -180.0)
        self.declare_parameter("angle_max", 180.0)
        self.declare_parameter("range_min", 0.12)
        self.declare_parameter("range_max", 10.0)
        self.declare_parameter("singleChannel", True)

        self.get_logger().info("lidar_argitaratzailea nodoa IN constructor")
    
    def on_configure(self, state: LifecycleState):
        self.get_logger().info("lidar_argitaratzailea nodoa IN on_configure")
        
        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.get_logger().info(f'{self.port}')
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.lidar_type = self.get_parameter("lidar_type").get_parameter_value().integer_value
        self.device_type = self.get_parameter("device_type").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.scan_frequency = self.get_parameter("scan_frequency").get_parameter_value().double_value
        self.sample_rate = self.get_parameter("sample_rate").get_parameter_value().integer_value
        self.angle_min = self.get_parameter("angle_min").get_parameter_value().double_value
        self.angle_max = self.get_parameter("angle_max").get_parameter_value().double_value
        self.range_min = self.get_parameter("range_min").get_parameter_value().double_value
        self.range_max = self.get_parameter("range_max").get_parameter_value().double_value
        self.single_channel = self.get_parameter("singleChannel").get_parameter_value().bool_value

        # Argitaratzailea
        self.publisher_ = self.create_lifecycle_publisher(LaserScan, 'lidar_datuak', 10)
        self.timer_ = self.create_timer(0.2, self.publish_lidar_data)  # 100ms-tara (0.1 Hz) argitaratu
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info("lidar_argitaratzailea nodoa IN on_cleanup")
        self.lidar = None
        self.port = None
        self.baudrate = None
        self.lidar_type = None
        self.device_type = None
        self.frame_id = None
        self.scan_frequency = None
        self.sample_rate = None
        self.angle_min = None
        self.angle_max = None
        self.range_min = None
        self.range_max = None
        self.single_channel = None
        self.active = None
        self.destroy_lifecycle_publisher(self.publisher_)
        self.destroy_timer(self.timer_)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info("lidar_argitaratzailea nodoa IN on_activate")

        # Lidar hasiarazi
        self.initialize_lidar()

        self.timer_.reset()

        self.get_logger().info('LIDAR neurriak /lidar_datuak argitaratzen 0.100s-tara', once=True)

        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info("lidar_argitaratzailea nodoa IN on_deactivate")
        self.active = not (self.lidar.turnOff())
        self.timer_.cancel()

        return super().on_deactivate(state)

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("lidar_argitaratzailea nodoa IN on_shutdown")
        return TransitionCallbackReturn.SUCCESS
    
    def initialize_lidar(self):
        ydlidar.os_init()
        self.lidar = ydlidar.CYdLidar()
        self.lidar.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
        self.lidar.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.baudrate)
        self.lidar.setlidaropt(ydlidar.LidarPropLidarType, self.lidar_type)
        self.lidar.setlidaropt(ydlidar.LidarPropDeviceType, self.device_type)
        self.lidar.setlidaropt(ydlidar.LidarPropScanFrequency, self.scan_frequency)
        self.lidar.setlidaropt(ydlidar.LidarPropSampleRate, self.sample_rate)
        self.lidar.setlidaropt(ydlidar.LidarPropSingleChannel, self.single_channel)

        if not self.lidar.initialize():
            self.get_logger().error("LiDAR initialization failed")
        else:
            self.active = self.lidar.turnOn()

    def publish_lidar_data(self):
        if self.active:
            scan = ydlidar.LaserScan()
            if self.lidar.doProcessSimple(scan):
                msg = LaserScan()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.angle_min = math.degrees(scan.config.min_angle)
                msg.angle_max = math.degrees(scan.config.max_angle)
                msg.angle_increment = math.degrees(scan.config.angle_increment)
                msg.scan_time = scan.config.scan_time
                msg.range_min = scan.config.min_range
                msg.range_max = scan.config.max_range

                sorted_points = sorted(scan.points, key=lambda point: math.degrees(point.angle))
                msg.ranges = [p.range for p in sorted_points]
                msg.intensities = [p.intensity for p in scan.points]

                self.publisher_.publish(msg)
            else:
                self.get_logger().error("ERROR lidar datuak lortzerakoan")

def main(args=None):
    rclpy.init(args=args)
    node = LidarArgitaratzailea()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Erabiltzaileak nodoa gelditu du.")

        if node.active:
            node.lidar.turnOff()
            node.lidar = None
            node.port = None
            node.baudrate = None
            node.lidar_type = None
            node.device_type = None
            node.frame_id = None
            node.scan_frequency = None
            node.sample_rate = None
            node.angle_min = None
            node.angle_max = None
            node.range_min = None
            node.range_max = None
            node.single_channel = None
            node.active = None
            node.destroy_lifecycle_publisher(node.publisher_)
            node.destroy_timer(node.timer_)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
