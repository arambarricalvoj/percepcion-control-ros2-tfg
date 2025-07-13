import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import Int16, Int16MultiArray
from adafruit_servokit import ServoKit
from bilbotiks_interfazeak.msg import ServoakMugitu

class ServoKontrolatzailea(LifecycleNode):
    def __init__(self):
        super().__init__('servo_kontrolatzailea')
        self.kit = None
        self.subscription = None
        self.active = None

        # Parametroak ezarri lehenetsitako balioak
        self.declare_parameter('actuation_range', 300)
        self.declare_parameter('pulse_width_range', (500, 2500))

        self.get_logger().info("servo_kontrolatzailea nodoa IN constructor")

    def on_configure(self, state: LifecycleState):
        self.get_logger().info("servo_kontrolatzailea nodoa IN on_configure")
        self.active = False

        # Parametroak irakurri YAML fitxerotik
        self.actuation_range = self.get_parameter('actuation_range').value
        self.pulse_width_range = tuple(self.get_parameter('pulse_width_range').value)

        # Configuración inicial de servos
        self.kit = ServoKit(channels=16)
        self.servoak_konfiguratu()

        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info("servo_kontrolatzailea nodoa IN on_cleanup")
        self.kit = None

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info("servo_kontrolatzailea nodoa IN on_activate")
        self.active = True
        
        # /servoak_mugitu topikorako harpidetza konfiguratu
        self.subscription = self.create_subscription(
            ServoakMugitu,
            'servoak_mugitu',
            self.servoak_mugitu_callback,
            10
        )

        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info("motorrak_roboclaw nodoa IN on_deactivate")
        self.subscription = None
        self.active = False

        return super().on_deactivate(state)

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("motorrak_roboclaw nodoa IN on_shutdown")
        return TransitionCallbackReturn.SUCCESS
    
    def servoak_konfiguratu(self):
            for i in range(4):  # Control de los canales 0 a 3
                self.kit.servo[i].actuation_range = self.actuation_range
                self.kit.servo[i].set_pulse_width_range(*self.pulse_width_range)

    def servoak_mugitu_callback(self, msg):
        if self.active:
            for i, angle in enumerate(msg.angeluak):
                if angle != -1:  # Ángulo válido
                    self.kit.servo[i].angle = angle
                    self.get_logger().info(f"Servomotorra {i} mugituta {angle:.2f} gradutara")

def main(args=None):
    rclpy.init(args=args)
    node = ServoKontrolatzailea()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Erabiltzaileak nodoa gelditu du.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
