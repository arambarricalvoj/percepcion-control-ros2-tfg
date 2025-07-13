##########################################################################
# ROS2 imports...
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from sensor_msgs.msg import LaserScan, Image
from bilbotiks_interfazeak.msg import MotorrakMugitu, ServoakMugitu, Imu
from bilbotiks_interfazeak.action import Bira360

# Liburutegien imports...
import numpy as np
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
import time
##########################################################################


##########################################################################
class GidatzeProbaWrapper(Node):
    def __init__(self):
        super().__init__('gidatze_proba_wrapper')

        self.get_logger().info("/gidatze_proba_wrapper nodoa CONSTRUCTOR")

        # Parametroak
        self.declare_parameter('focal_length', 495)
        self.declare_parameter('marker_size', 0.3)
        self.declare_parameter('servoak_360', [90, 65, 105, 65])
        self.declare_parameter('bira_abiadura', 25)  
        self.declare_parameter('atzerapena_aruco_bilatzeko', 5.0) 
        """koordenatuak_default = {
            20: (1, 5),
            28: (4, 5),
            40: (0, 1.5),
            49: (0, 3.5),
            60: (1.5, 0),
            67: (3.5, 0),
            90: (5, 1.5),
            96: (5, 3.5)
        }
        self.declare_parameter('koordenatuak', koordenatuak_default)""" 

        self.servo_angeluak_360 = self.get_parameter('servoak_360').value
        self.bira_abiadura = self.get_parameter('bira_abiadura').get_parameter_value().integer_value
        self.atzerapena_aruco_bilatzeko = self.get_parameter('atzerapena_aruco_bilatzeko').get_parameter_value().double_value
        self.FOCAL_LENGTH = self.get_parameter('focal_length').get_parameter_value().integer_value
        self.MARKER_SIZE = self.get_parameter('marker_size').get_parameter_value().double_value
        #koordenatuak_yaml = self.get_parameter('koordenatuak').value
        #self.koordenatuak = {int(k): tuple(v) for k, v in koordenatuak_yaml.items()}

        # ARuCO objektuak
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)
        
        # Paralel callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Behar diren klase objektuak hasiarazi
        self.imu_datuak = None
        self.imu_datu_berriak = False
        self.lidar_datuak = None
        self.kamera_irudia = None
        self.bridge = CvBridge()
        self.arucos = {}
        self.listaTuplas = []
        self.hasierakoAngelua = None
        self.koordenatuak = {
            20: (1, 5),
            28: (4, 5),
            40: (0, 1.5),
            49: (0, 3.5),
            60: (1.5, 0),
            67: (3.5, 0),
            90: (5, 1.5),
            96: (5, 3.5)
        }

        # Argitaratzaileak (PUBLISHERS)
        self.motor_publisher = self.create_publisher(MotorrakMugitu, 'motorrak_mugitu', 10) 
        self.servo_publisher = self.create_publisher(ServoakMugitu, 'servoak_mugitu', 10)

        # Harpidetzak (SUBSCRIBERS)
        self.subscription_imu = self.create_subscription(
            Imu,
            'imu_datuak',
            self.imu_callback,
            20,
            callback_group=self.callback_group
        )
        
        """self.subscription_lidar = self.create_subscription(
            LaserScan,
            'lidar_datuak',
            self.lidar_callback,
            10
            #callback_group=self.callback_group
        )"""

        self.subscription_kamera = self.create_subscription(
            Image,
            'kamera_irudia',
            self.kamera_callback,
            10,
            callback_group=self.callback_group
        )

        """self.subscription_kodetzaileak = self.create_subscription(
            ???,
            'motorren_kodeatzaileak',
            self.kodetzaileak_callback,
            10
        )"""

        # Ekintza zerbitzaria (ACTION SERVER)
        self.action_server_arucoBilatu = ActionServer(
            self,
            Bira360,
            'arucoBilatu',
            self.arucoBilatu,
            callback_group=self.callback_group
        )

        # Temporizador para ejecutar el algoritmo periódicamente
        #self.timer = self.create_timer(0.2, self.algoritmo)

        self.get_logger().info('gidatze_proba_wrapper nodoa hasita.')
    

    ##########################################################################
    # MUGIMENDUAREN LAGUNTZA FUNTZIOAK
    def servoak_kokatu(self):
        msg = ServoakMugitu()
        msg.angeluak = self.servo_angeluak_360
        self.servo_publisher.publish(msg)
        self.get_logger().info(f'Moviendo servos a: {msg.angeluak}')

    def motorrak_mugitu(self, abiadura):
        msg = MotorrakMugitu()
        msg.abiadurak = abiadura
        self.motor_publisher.publish(msg)

    def motorrak_gelditu(self):
        msg = MotorrakMugitu()
        msg.abiadurak = [0, 0, 0, 0, 0, 0]
        self.motor_publisher.publish(msg)
    ##########################################################################

    
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
    def posizioa_kalkulatu(self):
        """
        Calcula la posición actual utilizando múltiples puntos y minimizando el error (best fit).

        puntos: Lista de tuplas (x, y, d), donde x, y son las coordenadas del punto 
        y d es la distancia al punto.
        """
        # Configurar las matrices para el sistema de ecuaciones
        A = []
        B = []

        for aruco_id in self.arucos:
            new = (self.koordenatuak[aruco_id][0], self.koordenatuak[aruco_id][1], self.arucos[aruco_id])
            self.listaTuplas.append(new)

        for x, y, d in self.listaTuplas:
            A.append([2 * x, 2 * y, -1])  # Coeficientes
            B.append(x**2 + y**2 - d**2)  # Distancias ajustadas

        A = np.array(A)
        B = np.array(B)

        # Resolver el sistema de ecuaciones utilizando mínimos cuadrados
        resultado, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
        posicion_x, posicion_y = resultado[0], resultado[1]

        self.get_logger().info(f"X {posicion_x} Y {posicion_y}") 
    ##########################################################################
    

    ##########################################################################
    # IMU harpidetza (IMU SUBSCRIBER)
    def imu_callback(self, msg):
        self.imu_datuak = msg
        self.imu_datu_berriak = True  # Indicar que hay nuevos datos disponibles
    ##########################################################################
    

    ##########################################################################
    # LiDAR harpidetza (LiDAR SUBSCRIBER)
    def lidar_callback(self, msg):
        self.lidar_datuak = msg
        self.angeluak = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        self.ranges = np.nan_to_num(np.array(msg.ranges))
        self.ranges[self.ranges < 0.0011] = 1e6
    ##########################################################################

        
    ##########################################################################
    # KAMERA harpidetza (KAMERA SUBSCRIBER)
    def kamera_callback(self, msg):
        self.kamera_irudia = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(self.kamera_irudia)

        if markerIds is not None:
            for i, corner in zip(markerIds, markerCorners):
                top_left, top_right, bottom_right, bottom_left = corner[0]
                pixel_width = np.linalg.norm(top_right - top_left)
                
                if pixel_width > 0:
                    distance = (self.MARKER_SIZE * self.FOCAL_LENGTH) / pixel_width
                    distance_text = f"Dist: {distance:.2f} m"
                    text_position = (int(top_left[0]) + 20, int(top_left[1]) + 20)

                    if i[0] not in self.arucos:
                        self.arucos[i[0]] = distance
                        self.get_logger().info(f"ARuCO añadido con id {i[0]} y distancia {distance}")  
    ##########################################################################
    
    ##########################################################################
    # KODETZAILEAK harpidetza (KAMERA SUBSCRIBER)
    def kodetzaileak_callback():
        return 0
    ##########################################################################

    
    ##########################################################################
    # EKINTZAREN ZERBITZARIA (ACTION SERVER) Bira egiteko prozesua  
    def arucoBilatu(self, goal_handle):
        denbora = time.time()
        hasierako_denbora = denbora
        self.nodoa_kudeatu('kamera_argitaratzailea', Transition.TRANSITION_CONFIGURE)
        self.nodoa_kudeatu('kamera_argitaratzailea', Transition.TRANSITION_ACTIVATE) 
        self.nodoa_kudeatu('imu_argitaratzailea', Transition.TRANSITION_CONFIGURE)
        self.nodoa_kudeatu('imu_argitaratzailea', Transition.TRANSITION_ACTIVATE)
        self.nodoa_kudeatu('motorrak_roboclaw', Transition.TRANSITION_CONFIGURE)
        self.nodoa_kudeatu('motorrak_roboclaw', Transition.TRANSITION_ACTIVATE)
        self.nodoa_kudeatu('servoak', Transition.TRANSITION_CONFIGURE)
        self.nodoa_kudeatu('servoak', Transition.TRANSITION_ACTIVATE)

        while denbora - hasierako_denbora < self.atzerapena_aruco_bilatzeko:
            rclpy.spin_once(self, timeout_sec=0.1) #?
            denbora = time.time()
            continue

        while self.imu_datuak is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().warn("No se ha recibido información de IMU aún. Abortando la acción.")
            continue 
        
        # Ekintzaren jomuga jaso (ACTION GOAL)
        noranzkoa = goal_handle.request.noranzkoa  # 0: ezkerra, 1: eskuina
        bira_kopurua = goal_handle.request.zenbakia

        emandako_birak = 0
        hasierako_angelua = self.imu_datuak.euler_angeluak.x
        self.hasierakoAngelua = hasierako_angelua
        self.get_logger().info(f'Hasierako angelua: {hasierako_angelua}')
        oraingo_angelua = hasierako_angelua
        anterior = hasierako_angelua
        self.servoak_kokatu()

        if noranzkoa == 0:
            self.motorrak_mugitu([self.bira_abiadura, self.bira_abiadura, self.bira_abiadura*(-1), self.bira_abiadura*(-1), self.bira_abiadura, self.bira_abiadura])
        else:
            self.motorrak_mugitu([self.bira_abiadura*(-1), self.bira_abiadura*(-1), self.bira_abiadura, self.bira_abiadura, self.bira_abiadura*(-1), self.bira_abiadura*(-1)])

        angulo_total = 0
        desfase = 10

        feedback_msg = Bira360.Feedback()
        while angulo_total < (360 * bira_kopurua)-desfase:
                rclpy.spin_once(self, timeout_sec=0.1) #?

                if self.imu_datu_berriak:
                    oraingo_angelua = self.imu_datuak.euler_angeluak.x if hasattr(self.imu_datuak, 'euler_angeluak') else oraingo_angelua
                    self.imu_datu_berriak = False                 

                    # Publicar feedback al cliente
                    feedback_msg.oraingo_angelua = oraingo_angelua
                    feedback_msg.bira_kopurua = emandako_birak
                    self.get_logger().info(f'Oraingo angelua: {feedback_msg.oraingo_angelua}')
                    #self.get_logger().info(f'Bira kopurua: {feedback_msg.bira_kopurua}')
                    goal_handle.publish_feedback(feedback_msg)

                    # Calcula la diferencia entre actual y anterior considerando el ciclo
                    if oraingo_angelua != 0.0 and oraingo_angelua > 0:
                        if noranzkoa:
                            diferencia = (oraingo_angelua - anterior + 360) % 360
                        else:
                            diferencia = (anterior - oraingo_angelua + 360) % 360
                        anterior = oraingo_angelua  # Actualiza la lectura anterior

                        angulo_total += diferencia

                    self.get_logger().info(f'TOTAL: {angulo_total}')

        self.motorrak_gelditu()
        self.nodoa_kudeatu('kamera_argitaratzailea', Transition.TRANSITION_DEACTIVATE) 
        self.nodoa_kudeatu('kamera_argitaratzailea', Transition.TRANSITION_CLEANUP) 
        self.nodoa_kudeatu('kamera_argitaratzailea', Transition.TRANSITION_UNCONFIGURED_SHUTDOWN)
        self.posizioa_kalkulatu()

        success = True
        goal_handle.succeed()
        
        result = Bira360.Result()
        result.arrakasta = success
        result.amaierako_angelua = oraingo_angelua
        result.hasierako_angelua = hasierako_angelua
        result.bira_kopurua = emandako_birak

        return result
    ##########################################################################      


def main(args=None):    
    rclpy.init(args=args)
    node = GidatzeProbaWrapper()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()