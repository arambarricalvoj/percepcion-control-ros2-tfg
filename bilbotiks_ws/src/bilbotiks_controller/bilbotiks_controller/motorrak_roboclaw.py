import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from geometry_msgs.msg import Twist
from bilbotiks_controller.roboclaw_3 import Roboclaw
from bilbotiks_interfazeak.msg import MotorrakMugitu #, MotorrenKodetzaileak

class MotorKontrolatzailea(LifecycleNode):
    def __init__(self):
        super().__init__('motor_kontrolatzailea')
        self.rc = None
        self.baud_rate = None
        self.motor_addresses = None
        self.publisher_ = None
        self.timer_ = None

	    # Parametroak ezarri lehenetsitako balioak
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('motorren_helbideak', [128, 129, 130])

        self.get_logger().info("motorrak_roboclaw nodoa IN constructor")

    def on_configure(self, state: LifecycleState):
        self.get_logger().info("motorrak_roboclaw nodoa IN on_configure")

        # Parametroak irakurri YAML fitxerotik
        self.baud_rate = self.get_parameter('baud_rate').value
        self.motor_addresses = self.get_parameter('motorren_helbideak').value
        self.get_logger().info(f"{self.motor_addresses}")

        for address in self.motor_addresses:
            self.rc = self.test_connection(address)
            if self.rc is None:
                self.get_logger().info("RC is None")
                #rclpy.shutdown()
        self.get_logger().info("    TOPIKO HARPIDETZAK: /motorrak_mugitu")

        # /motorren_kodeatzaileak topikoan datuak argitaratzeko konfiguratu
        #self.publisher_ = self.create_lifecycle_publisher(MotorrenKodetzaileak, '/motorren_kodetzaileak', 10)
        #timer_period = 0.2  # Publicar cada 1 segundo
        #self.timer_ = self.create_timer(timer_period, self.kodetzaileak_callback)

        self.get_logger().info("    TOPIKO ARGITARATZAILE: /motorres_kodetzaileak")

        self.motorrak_gelditu()

        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info("motorrak_roboclaw nodoa IN on_cleanup")
        self.baud_rate = None
        self.motor_addresses = None
        self.subscription = None
        #self.destroy_lifecycle_publisher(self.publisher_)
        #self.destroy_timer(self.timer_)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info("motorrak_roboclaw nodoa IN on_activate")
        
        # /motorrak_mugitu topikorako harpidetza konfiguratu
        self.subscription = self.create_subscription(
            MotorrakMugitu,
            'motorrak_mugitu',
            self.motorrak_mugitu_callback,
            10 # Buffer isatsaren tamaina
        )

        #self.timer_.reset()

        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info("motorrak_roboclaw nodoa IN on_deactivate")
        self.motorrak_gelditu()
        self.subscription = None
        #self.timer_.cancel()

        return super().on_deactivate(state)

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("motorrak_roboclaw nodoa IN on_shutdown")
        self.rc = None
        return TransitionCallbackReturn.SUCCESS
    
    def test_connection(self, address):
        roboclaw0 = Roboclaw("/dev/serial0", self.baud_rate)
        roboclaw1 = Roboclaw("/dev/serial1", self.baud_rate)
        connected0 = roboclaw0.Open() == 1
        connected1 = roboclaw1.Open() == 1
        if connected0:
            self.get_logger().info(f"Roboclaw {address} helbidera konektatuta /dev/serial0")
            #print("Connected to /dev/serial0.")
            return roboclaw0
        elif connected1:
            self.get_logger().info(f"Roboclaw {address} helbidera konektatuta /dev/serial1")
            #print("Connected to /dev/serial1.")
            return roboclaw1
        else:
            self.get_logger().info(f"Ezin da Roboclaw {address} helbidera konektatu, ez /dev/serial0 ezta /dev/serial1")
            #print("Could not open comport /dev/serial0 or /dev/serial1.")
        return None
    
    def motorrak_gelditu(self):
        for address in self.motor_addresses:
            self.rc.ForwardM1(address, 0)
            self.rc.ForwardM2(address, 0)
        self.get_logger().info("Motorrak geldiarazi dira")

    def motorrak_mugitu_callback(self, msg):
        motorra_1 = int(msg.abiadurak[0])
        motorra_2 = int(msg.abiadurak[1])
        motorra_3 = int(msg.abiadurak[2])
        motorra_4 = int(msg.abiadurak[3])
        motorra_5 = int(msg.abiadurak[4])
        motorra_6 = int(msg.abiadurak[5])

        self.get_logger().info("Motorrak mugitzen hurrengo abiadurara:")

        for address in self.motor_addresses:
            if address == 128: # 2. eta 4. motorrak
                if motorra_2 < 0:
                    self.rc.BackwardM1(address, motorra_2*(-1))
                else:
                    self.rc.ForwardM1(address, motorra_2)

                self.get_logger().info(f"   Motorra 2 mugitzen {motorra_2} abiadurara")

                if motorra_4 < 0:
                    self.rc.BackwardM2(address, motorra_4*(-1))
                else:
                    self.rc.ForwardM2(address, motorra_4)

                self.get_logger().info(f"   Motorra 4 mugitzen {motorra_4} abiadurara")

            if address == 129: # 5. eta 6. motorrak
                if motorra_5 < 0:
                    self.rc.BackwardM1(address, motorra_5*(-1))
                else:
                    self.rc.ForwardM1(address, motorra_5)
                self.get_logger().info(f"   Motorra 5 mugitzen {motorra_5} abiadurara")

                if motorra_6 < 0:
                    self.rc.BackwardM2(address, motorra_6*(-1))
                else:
                    self.rc.ForwardM2(address, motorra_6)
                self.get_logger().info(f"   Motorra 6 mugitzen {motorra_6} abiadurara")

            if address == 130: # 1.go eta 3. motorrak
                if motorra_1 < 0:
                    self.rc.BackwardM1(address, motorra_1*(-1))
                else:
                    self.rc.ForwardM1(address, motorra_1)
                self.get_logger().info(f"   Motorra 1 mugitzen {motorra_1} abiadurara")

                if motorra_3 < 0:
                    self.rc.BackwardM2(address, motorra_3*(-1))
                else:
                    self.rc.ForwardM2(address, motorra_3)
                self.get_logger().info(f"   Motorra 3 mugitzen {motorra_3} abiadurara")


    """def kodetzaileak_callback(self):
            msg = MotorrenKodetzaileak()
            msg.pultsoak = []

            self.publisher_.publish(msg)
            self.get_logger().info(f'Publicando: {msg.data}')"""

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorKontrolatzailea()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.get_logger().info("Erabiltzaileak nodoa gelditu du.")
        if motor_controller.rc:
            motor_controller.motorrak_gelditu()
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
