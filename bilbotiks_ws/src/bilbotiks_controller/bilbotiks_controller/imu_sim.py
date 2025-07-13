import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from bilbotiks_interfazeak.msg import Imu
import random
import math

class IMUArgitaratzailea(LifecycleNode):
    def __init__(self):
        super().__init__('imu_argitaratzailea')
        self.publisher_ = None
        self.timer_ = None

        self.get_logger().info("imu_argitaratzailea nodoa IN constructor")
        
    def on_configure(self, state: LifecycleState):
        self.get_logger().info("imu_argitaratzailea nodoa IN on_configure")
        
        # Modo simulación: No se inicializa hardware (I2C ni sensor real)
        self.publisher_ = self.create_lifecycle_publisher(Imu, 'imu_datuak', 20)
        self.timer_ = self.create_timer(0.1, self.publish_imu_data)  # Cada 100ms

        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info("imu_argitaratzailea nodoa IN on_cleanup")
        self.destroy_lifecycle_publisher(self.publisher_)
        self.destroy_timer(self.timer_)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info("imu_argitaratzailea nodoa IN on_activate")
        self.timer_.reset()
        self.get_logger().info('Publicando datos de la IMU en /imu_datuak cada 100ms', once=True)
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
        
        # Simulación de la temperatura (valor entero)
        msg.tenperatura = random.randint(-10, 40)  # Ejemplo: rango entre -10 y +40 °C

        # Simulación del magnetómetro (valores en float)
        msg.magnetometroa.x = random.uniform(-100.0, 100.0)
        msg.magnetometroa.y = random.uniform(-100.0, 100.0)
        msg.magnetometroa.z = random.uniform(-100.0, 100.0)

        # Simulación del giroscopio (valores en float)
        msg.giroskopioa.x = random.uniform(-500.0, 500.0)
        msg.giroskopioa.y = random.uniform(-500.0, 500.0)
        msg.giroskopioa.z = random.uniform(-500.0, 500.0)

        # Simulación de los ángulos Euler (valores float entre 0 y 360)
        msg.euler_angeluak.x = random.uniform(0.0, 360.0)
        msg.euler_angeluak.y = random.uniform(0.0, 360.0)
        msg.euler_angeluak.z = random.uniform(0.0, 360.0)

        # Simulación del cuaternión: genero 4 números aleatorios y los normalizo para obtener un cuaternión unitario
        q = [random.uniform(-1.0, 1.0) for _ in range(4)]
        norm = math.sqrt(sum(x * x for x in q))
        q = [x / norm for x in q]
        msg.kuaternioak.x = q[0]
        msg.kuaternioak.y = q[1]
        msg.kuaternioak.z = q[2]
        msg.kuaternioak.w = q[3]

        # Simulación de la aceleración lineal (valores en float)
        msg.azelerazio_lineala.x = random.uniform(-10.0, 10.0)
        msg.azelerazio_lineala.y = random.uniform(-10.0, 10.0)
        msg.azelerazio_lineala.z = random.uniform(-10.0, 10.0)

        # Simulación de la gravedad (valores en float)
        msg.grabitatea.x = random.uniform(-9.81, 9.81)
        msg.grabitatea.y = random.uniform(-9.81, 9.81)
        msg.grabitatea.z = random.uniform(-9.81, 9.81)

        # Publicar el mensaje
        self.publisher_.publish(msg)
        self.get_logger().debug("Datos simulados publicados.")

def main(args=None):
    rclpy.init(args=args)
    node = IMUArgitaratzailea()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("El usuario ha interrumpido el nodo.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
