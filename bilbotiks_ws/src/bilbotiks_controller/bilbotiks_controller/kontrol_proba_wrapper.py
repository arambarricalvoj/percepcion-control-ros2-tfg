##########################################################################
# ROS2 imports...
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from sensor_msgs.msg import LaserScan

from bilbotiks_interfazeak.msg import MotorrakMugitu, ServoakMugitu

# Liburutegien imports...
import numpy as np

# SAVE DATA TFG
import csv
import time
from datetime import datetime
# SAVE DATA TFG
##########################################################################


##########################################################################
class KontrolProbaWrapper(Node):
    def __init__(self):
        super().__init__('kontrol_proba_wrapper')

        # Behar diren klase objektuak hasiarazi
        self.last_error = 0
        self.integral = 0
        self.lidar_datuak = None
        self.mugitzen = True

        # Parametroak ezarri lehenetsitako balioak
        self.declare_parameter('kp', 100.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 25.0)
        self.declare_parameter('vel', 20)
        self.declare_parameter('izena', 'izena')
        self.declare_parameter('eskuina', [-90.0, -45.0])
        self.declare_parameter('ezkerra', [135.0, 180.0])
        self.declare_parameter('aurrera', [-180.0, -90.0])

        # Parametroak irakurri YAML fitxerotik
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.vel = self.get_parameter('vel').value
        self.nombre = self.get_parameter('izena').value    
        self.eskuina_angeluak_lidar = self.get_parameter('eskuina').value
        self.ezkerra_angeluak_lidar = self.get_parameter('ezkerra').value 
        self.aurrera_angeluak_lidar = self.get_parameter('aurrera').value 

        # Argitaratzaileak (PUBLISHERS)
        self.motor_publisher = self.create_publisher(MotorrakMugitu, 'motorrak_mugitu', 10) 
        self.servo_publisher = self.create_publisher(ServoakMugitu, 'servoak_mugitu', 10)

        # Harpidetzak (SUBSCRIBERS)
        self.subscription = self.create_subscription(
            LaserScan,
            'lidar_datuak',
            self.lidar_callback,
            10
        )

        self.nodoa_kudeatu('motorrak_roboclaw', Transition.TRANSITION_CONFIGURE) # Motorrak konfiguratu
        self.nodoa_kudeatu('motorrak_roboclaw', Transition.TRANSITION_ACTIVATE) # Motorrak aktibatu
        self.nodoa_kudeatu('servoak', Transition.TRANSITION_CONFIGURE) # Servoak konfiguratu
        self.nodoa_kudeatu('servoak', Transition.TRANSITION_ACTIVATE) # Servoak aktibatu
        self.nodoa_kudeatu('lidar_argitaratzailea', Transition.TRANSITION_CONFIGURE) # Kamera aktibatu
        self.nodoa_kudeatu('lidar_argitaratzailea', Transition.TRANSITION_ACTIVATE) # Lidar aktibatu
        
        self.servoak_kokatu(self.servo_posizioak_kalkulatu(0))
        self.motorrak_mugitu()
        self.get_logger().info('/kontrola_proba_wrapper nodoa hasita')

    
    ##########################################################################
    # LIFECYCLE nodoen trantzisioak kudeatzeko
    def nodoa_kudeatu(self, nodoa, transition_id):
        # Lifecycle nodoen egoera kudeatzeko funtzioa
        self.get_logger().info(f'/{nodoa}/change_state')
        self.change_state = self.create_client(ChangeState, f'/{nodoa}/change_state')
        while not self.change_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'/{nodoa}/change_state zerbitzua prest egoteko zain...')    
        
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.change_state.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Trantsizioa ongi burutu da: {nodoa}, {transition_id}')
        else:
            self.get_logger().error('Errorea trantsizioa egitean.')
    ##########################################################################
    
    ##########################################################################
    # MUGIMENDUAREN LAGUNTZA FUNTZIOAK
    def servoak_kokatu(self, angeluak):
        # Servomotorrak kokatu
        msg = ServoakMugitu()
        msg.angeluak = angeluak  # 360º-ko biraketa egiteko behar diren servomotorren angeluak
        self.servo_publisher.publish(msg)
        self.get_logger().info(f'Servoak posizio egokira mugitzeko /servoak_mugitu topikoan argitaratuta: {msg.angeluak}')

    def motorrak_mugitu(self, abiadura=[-20, 20, -20, 20, -20, 20]):
        # Motorrak mugitu
        msg = MotorrakMugitu()
        abiadura =[self.vel*(-1), self.vel, self.vel*(-1), self.vel, self.vel*(-1), self.vel]
        msg.abiadurak = abiadura
        self.motor_publisher.publish(msg)
        self.get_logger().info(f'motorrak mugitzeko /motorrak_mugitu topikoan argitaratutako abiadurak: {msg.abiadurak}')

        return True

    def servo_posizioak_kalkulatu(self, desired_turn_angle):
        if desired_turn_angle > 90:
            desired_turn_angle = 90
        elif desired_turn_angle < -90:
            desired_turn_angle = -90
        
        self.get_logger().info(f'DESIRED: {desired_turn_angle}')

        servo_angles = [0] * 4
        servo_angles[0] = 15 #int(max(0, 15 + (desired_turn_angle / 90) * (60 - 15)))
        servo_angles[1] = int(max(0, 140 - (desired_turn_angle / 90) * (140 - 120)))
        servo_angles[2] = int(max(0, 25 - (desired_turn_angle / 90) * (25 - 0)))
        servo_angles[3] = 145 #int(max(0, 145 + (desired_turn_angle / 90) * (180 - 145)))

        self.get_logger().info(f'{servo_angles}')

        return servo_angles
    ##########################################################################
    

    ##########################################################################
    # LIDAR harpidetza (LIDAR SUBSCRIBER)
    def lidar_callback(self, msg):
        if self.mugitzen:
            self.lidar_datuak = msg
            self.angeluak = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            self.ranges = np.nan_to_num(np.array(msg.ranges))
            self.ranges[self.ranges < 0.0011] = 1e6
            self.ranges[self.ranges > 8.0] = 8.0

            correcion = self.norabidea_kalkulatu()
            servo_angles = self.servo_posizioak_kalkulatu(correcion)
            self.servoak_kokatu(servo_angles)
    ##########################################################################


    ##########################################################################
    # PID
    def norabidea_kalkulatu(self):
        
        #Eskuina
        angle_start = self.eskuina_angeluak_lidar[0] #-90
        angle_end = self.eskuina_angeluak_lidar[1] #-67.5 #45
        start_index = int((angle_start - self.lidar_datuak.angle_min) / self.lidar_datuak.angle_increment)
        end_index = int((angle_end - self.lidar_datuak.angle_min) / self.lidar_datuak.angle_increment)
        sublista = self.ranges[start_index:end_index+1]
        min_right = np.argmin(self.ranges[start_index:end_index+1])
        distanciaDerecha = sublista[min_right]

        #Ezkerra
        angle_start = self.ezkerra_angeluak_lidar[0] #157.5 #135
        angle_end = self.ezkerra_angeluak_lidar[1] #180
        start_index = int((angle_start - self.lidar_datuak.angle_min) / self.lidar_datuak.angle_increment)
        end_index = int((angle_end - self.lidar_datuak.angle_min) / self.lidar_datuak.angle_increment)
        sublista = self.ranges[start_index:end_index+1]
        min_left = np.argmin(self.ranges[start_index:end_index+1])
        distanciaIzquierda = sublista[min_left]

        #Aurrera
        angle_start = self.aurrera_angeluak_lidar[0] #-180
        angle_end = self.aurrera_angeluak_lidar[1] #-90
        start_index = int((angle_start - self.lidar_datuak.angle_min) / self.lidar_datuak.angle_increment)
        end_index = int((angle_end - self.lidar_datuak.angle_min) / self.lidar_datuak.angle_increment)
        sublista = self.ranges[start_index:end_index+1]
        min_front = np.argmin(self.ranges[start_index:end_index+1])
        distanciaFrente = sublista[min_front]

        self.get_logger().info(f'Distancia derecha: {distanciaDerecha}')
        self.get_logger().info(f'Distancia frente: {distanciaFrente}')
        self.get_logger().info(f'Distancia izquierda: {distanciaIzquierda}')
        
        # Calcular la corrección PID
        error = (distanciaIzquierda - distanciaDerecha)
        d = error - self.last_error
        i = self.integral + error
        correcion = (error * (self.kp) + i * (self.ki) +d * (self.kd)) * (-1)

        self.get_logger().info(f'ERROR: {error}')
        self.get_logger().info(f'CORRECCIÓN: {correcion}\n')

        ##########################################################################
        #SAVE FOR TFG
        # Ruta del archivo donde se guardarán los datos
        archivo = f"performance_{self.nombre}_{self.kp}_{self.ki}_{self.kd}_{self.vel}.csv"

        # Abrir el archivo en modo escritura (o crear si no existe)
        with open(archivo, mode='a', newline='') as file:
            escritor = csv.writer(file)

            # Escribir encabezados solo si el archivo está vacío
            if file.tell() == 0:
                escritor.writerow(["Error", "Correccion", "CorrecionEscalado","DistanciaFrente", "DistanciaDerecha", "DistanciaIzquierda", "Timestamp"])

            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            valores = [error, correcion, distanciaFrente, distanciaDerecha, distanciaIzquierda, timestamp]
            escritor.writerow(valores)
        #SAVE FOR TFG
        ##########################################################################

        return correcion
    ##########################################################################

        
def main(args=None):
    rclpy.init(args=args)
    node = KontrolProbaWrapper()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()