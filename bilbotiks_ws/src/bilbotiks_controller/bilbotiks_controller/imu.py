import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from bilbotiks_interfazeak.msg import Imu
import board
import adafruit_bno055

class IMUArgitaratzailea(LifecycleNode):
    def __init__(self):
        super().__init__('imu_argitaratzailea')
        self.i2c = None
        self.sensor = None
        self.publisher_ = None
        self.timer_ = None

        self.get_logger().info("imu_argitaratzailea nodoa IN constructor")
        
    def on_configure(self, state: LifecycleState):
        self.get_logger().info("imu_argitaratzailea nodoa IN on_configure")
        
        # IMU sentsorea hasieratu
        self.i2c = board.I2C()  # SCL eta SDA erabili
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

        self.publisher_ = self.create_lifecycle_publisher(Imu, 'imu_datuak', 20)
        self.timer_ = self.create_timer(0.1, self.publish_imu_data)  # 100ms-tara (0.1 Hz) argitaratu

        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info("imu_argitaratzailea nodoa IN on_cleanup")
        self.i2c = None
        self.sensor = None
        self.destroy_lifecycle_publisher(self.publisher_)
        self.destroy_timer(self.timer_)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info("imu_argitaratzailea nodoa IN on_activate")

        self.timer_.reset()

        self.get_logger().info('IMU neurriak /imu_datuak argitaratzen 0.100s-tara', once=True)

        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info("imu_argitaratzailea nodoa IN on_deactivate")

        self.timer_.cancel()

        return super().on_deactivate(state)

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("imu_argitaratzailea nodoa IN on_shutdown")
        return TransitionCallbackReturn.SUCCESS
    
    def publish_imu_data(self):
        msg = Imu()
        
        # Tenperatura
        msg.tenperatura = self.sensor.temperature or 0.0

        # Magnetometroa
        magnetic = self.sensor.magnetic
        if magnetic:
            msg.magnetometroa.x = magnetic[0] or 0.0
            msg.magnetometroa.y = magnetic[1] or 0.0
            msg.magnetometroa.z = magnetic[2] or 0.0

        # Giroskopioa
        gyro = self.sensor.gyro
        if gyro:
            msg.giroskopioa.x = gyro[0] or 0.0
            msg.giroskopioa.y = gyro[1] or 0.0
            msg.giroskopioa.z = gyro[2] or 0.0

        # Euler angeluak
        euler_angles = self.sensor.euler
        if euler_angles:
            msg.euler_angeluak.x = euler_angles[0] or 0.0  # Roll
            msg.euler_angeluak.y = euler_angles[1] or 0.0  # Pitch
            msg.euler_angeluak.z = euler_angles[2] or 0.0  # Yaw

        # Kuaternioak
        quaternion = self.sensor.quaternion
        if quaternion:
            msg.kuaternioak.x = quaternion[0] or 0.0
            msg.kuaternioak.y = quaternion[1] or 0.0
            msg.kuaternioak.z = quaternion[2] or 0.0
            msg.kuaternioak.w = quaternion[3] or 0.0

        # Azelerazio lineala
        acceleration = self.sensor.acceleration
        if acceleration:
            msg.azelerazio_lineala.x = acceleration[0] or 0.0
            msg.azelerazio_lineala.y = acceleration[1] or 0.0
            msg.azelerazio_lineala.z = acceleration[2] or 0.0

        # Grabitatea
        gravity = self.sensor.gravity
        if gravity:
            msg.grabitatea.x = gravity[0] or 0.0
            msg.grabitatea.y = gravity[1] or 0.0
            msg.grabitatea.z = gravity[2] or 0.0

        # Publicar el mensaje
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUArgitaratzailea()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Erabiltzaileak nodoa gelditu du.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
