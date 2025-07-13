import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from bilbotiks_interfazeak.srv import Argazkia

class KameraArgitaratzailea(LifecycleNode):
    def __init__(self):
        super().__init__('kamera_argitaratzailea')
        
        self.cap_irudia = None       # Kamera irudia 
        self.cap_depth = None         # Kamera sakonera
        
        self.publisher_irudia = None  # Irudia argitaratzeko objektua
        self.publisher_depth = None   # Sakonera argitaratzeko objektua
        self.timer_ = None
        self.srv_argazkia_atera = None
        self.bridge = CvBridge()      # OpenCV - ROS konbertsioa

        # Parametroak
        self.port_irudia = None
        self.port_sakonera = None
        self.declare_parameter("port_irudia", 0)
        self.declare_parameter("port_sakonera", 2)

        self.get_logger().info("kamera_argitaratzailea nodoa IN constructor")
        
    def on_configure(self, state: LifecycleState):
        self.get_logger().info("kamera_argitaratzailea nodoa IN on_configure")

        # Parametroak
        self.port_irudia = self.get_parameter("port_irudia").get_parameter_value().integer_value
        self.port_sakonera = self.get_parameter("port_sakonera").get_parameter_value().integer_value
        self.get_logger().info(f'VideoCapture IMAGE: {self.port_irudia}, VideoCapture DEPTH: {self.port_sakonera}')

        self.srv_argazkia_atera = self.create_service(Argazkia, 'argazkia_atera', self.argazkia_atera)

        # Argitaratzaileak sortu
        self.publisher_irudia = self.create_lifecycle_publisher(Image, '/kamera_irudia', 10)
        #self.publisher_depth = self.create_lifecycle_publisher(Image, '/kamera_sakonera', 10)

        # Argitaratzaileen tenporizadoreak
        self.timer_ = self.create_timer(1.0/30.0, self.publish_camera_data)
        self.timer_.cancel()
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info("kamera_argitaratzailea nodoa IN on_cleanup")

        self.destroy_service(self.srv_argazkia_atera)
        self.srv_argazkia_atera = None
        
        # Kamera askatu
        if self.cap_irudia is not None:
            self.cap_irudia.release()
            self.cap_irudia = None
        if self.cap_depth is not None:
            #self.cap_depth.release()
            self.cap_depth = None
        
        # Argitaratzaileak eta tenporizadorea ezabatu
        self.destroy_lifecycle_publisher(self.publisher_irudia)
        #self.destroy_lifecycle_publisher(self.publisher_depth)
        self.destroy_timer(self.timer_)
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState):
        self.get_logger().info("kamera_argitaratzailea nodoa IN on_activate")
        
        # Argazkia ateratzeko zerbitzua itzali
        self.destroy_service(self.srv_argazkia_atera)
        self.srv_argazkia_atera = None
        
        # Kamera irudia ireki
        self.cap_irudia = cv2.VideoCapture(self.port_irudia)
        if not self.cap_irudia.isOpened():
            self.get_logger().error("No se pudo abrir la cámara de imagen (VideoCapture 0)")
            return TransitionCallbackReturn.FAILURE

        # Kamera sakonera ireki
        """self.cap_depth = cv2.VideoCapture(self.port_sakonera)
        if not self.cap_depth.isOpened():
            self.get_logger().error("No se pudo abrir el sensor de profundidad (VideoCapture 2)")
            return TransitionCallbackReturn.FAILURE"""
        
        # Tenporizadorea berritu
        self.timer_.reset()
        return super().on_activate(state)
    
    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info("kamera_argitaratzailea nodoa IN on_deactivate")
        
        # Tenporizadorea gelditu
        self.timer_.cancel()
        
        # Argazkia ateratzeko zerbitzua piztu
        self.srv_argazkia_atera = self.create_service(Argazkia, 'argazkia_atera', self.argazkia_atera)

        return super().on_deactivate(state)
    
    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("kamera_argitaratzailea nodoa IN on_shutdown")
        return TransitionCallbackReturn.SUCCESS

    def publish_camera_data(self):
        # Leer datos de la cámara de imagen
        ret_color, frame_color = self.cap_irudia.read()
        # Leer datos del sensor de profundidad
        #ret_depth, frame_depth = self.cap_depth.read()
        
        now = self.get_clock().now().to_msg()

        if ret_color:
            try:
                # Convertir la imagen capturada (BGR de OpenCV) a mensaje ROS
                img_msg = self.bridge.cv2_to_imgmsg(frame_color, encoding="bgr8")
                img_msg.header.stamp = now
                self.publisher_irudia.publish(img_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"Errorea kolore-irudiaren bihurketan: {str(e)}")
        else:
            self.get_logger().error("Errorea kameraren irudia atzitzean (VideoCapture 0)")
            
        """if ret_depth:
            try:
                # La codificación depende del sensor: en este ejemplo se usa 'mono16' para una imagen de profundidad
                depth_msg = self.bridge.cv2_to_imgmsg(frame_depth, encoding="mono16")
                depth_msg.header.stamp = now
                self.publisher_depth.publish(depth_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"Error en la conversión de imagen de profundidad: {str(e)}")
        else:
            self.get_logger().error("Error al capturar imagen de profundidad (VideoCapture 2)")"""

    def argazkia_atera(self, request, response):
        self.get_logger().info("'argazkia_atera' zerbitzua deitu da")
        
        self.cap_irudia = cv2.VideoCapture(self.port_irudia)
        if not self.cap_irudia.isOpened():
            self.get_logger().error(f"Ezin izan da kamera ireki (VideoCapture {self.port_irudia})")
        
        ret_color, frame_color = self.cap_irudia.read()
        self.cap_irudia.release()
        if ret_color:
            try:
                # Ateratako irudia (OpenCV-ren BGR) ROS mezu bihurtzea
                img_msg = self.bridge.cv2_to_imgmsg(frame_color, encoding="bgr8")
                # Nahi izanez gero, timestamp bat eslei dakioke header-ari
                img_msg.header.stamp = self.get_clock().now().to_msg()
                response.image = img_msg
                self.get_logger().info("Argazkia atera da.")
            except CvBridgeError as e:
                self.get_logger().error(f"Errorea argazkia ROS2 kodifikatzean {str(e)}")
        else:
            self.get_logger().error(f"Errorea argazkia ateratzerakoan (VideoCapture {self.port_irudia})")
        self.cap_irudia = None

        return response

def main(args=None):
    rclpy.init(args=args)
    node = KameraArgitaratzailea()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Erabiltzaileak nodoa gelditu du.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()