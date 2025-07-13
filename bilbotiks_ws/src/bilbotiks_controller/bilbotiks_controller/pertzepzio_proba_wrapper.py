##########################################################################
# ROS2 imports...
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

from sensor_msgs.msg import Image
from bilbotiks_interfazeak.msg import MotorrakMugitu, ServoakMugitu, Imu
from bilbotiks_interfazeak.action import Bira360
from bilbotiks_interfazeak.srv import KoloreaZenbakia, Argazkia

# Liburutegien imports...
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array
##########################################################################

##########################################################################
# Desactivar el uso de la GPU para TensorFlow
tf.config.set_visible_devices([], 'GPU')

# Cargar el modelo previamente entrenado
#model = load_model('/home/bilbotiks/Desktop/modelo_100epochs.keras')
# Define los nombres de tus clases
#class_labels = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "NaN"]
##########################################################################


##########################################################################
class PertzepzioProbaWrapper(Node):
    def __init__(self):
        super().__init__('pertzepzio_proba_wrapper')

        # Behar diren klase objektuak hasiarazi
        self.change_state = None
        self.imu_datuak = None
        self.bridge = CvBridge()
   
        # Parametroak
        self.declare_parameter('cnn_modelua', '/home/bilbotiks/moverRover/bilbotiks_ws/install/bilbotiks_controller/share/bilbotiks_controller/config//modelo_100epochs.keras')
        self.declare_parameter('servoak_360', [90, 65, 105, 65])
        self.declare_parameter('abiadura_azkar', 30)
        self.declare_parameter('abiadura_motel', 13)
        self.servo_angeluak_360 = self.get_parameter('servoak_360').value
        self.model = load_model(self.get_parameter('cnn_modelua').get_parameter_value().string_value)
        self.abiadura_azkar = self.get_parameter('abiadura_azkar').get_parameter_value().integer_value
        self.abiadura_motel = self.get_parameter('abiadura_motel').get_parameter_value().integer_value
        self.class_labels = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "NaN"]

        # Paralel callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Argitaratzaileak (PUBLISHERS)
        self.motor_publisher = self.create_publisher(MotorrakMugitu, 'motorrak_mugitu', 10)
        self.servo_publisher = self.create_publisher(ServoakMugitu, 'servoak_mugitu', 10)

        # Harpidetzak (SUBSCRIBERS)
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu_datuak',
            self.imu_callback,
            20,
            callback_group=self.callback_group
        )
        
        # Zerbitzu zerbitzaria (SERVICE SERVER)
        self.argazkia_zerbitzaria = self.create_service(KoloreaZenbakia, 'koloreaZenbakia_iragarri', self.koloreaZenbakia_iragarri, callback_group=self.callback_group)
        
        # Zerbitzu bezeroa (SERVICE CLIENT)
        self.argazkia_bezeroa = self.create_client(Argazkia, 'argazkia_atera')
        
        # Ekintza zerbitzaria (ACTION SERVER)
        self.action_server = ActionServer(
            self,
            Bira360,
            'pertzepzio_proba_360bira',
            self.bira360_callback,
            callback_group=self.callback_group
        )

        #self.nodoa_kudeatu('imu_argitaratzailea', Transition.TRANSITION_CONFIGURE)
        #self.nodoa_kudeatu('imu_argitaratzailea', Transition.TRANSITION_ACTIVATE)
        #self.nodoa_kudeatu('motorrak_roboclaw', Transition.TRANSITION_CONFIGURE)
        #self.nodoa_kudeatu('motorrak_roboclaw', Transition.TRANSITION_ACTIVATE)
        #self.nodoa_kudeatu('servoak', Transition.TRANSITION_CONFIGURE)
        #self.nodoa_kudeatu('servoak', Transition.TRANSITION_ACTIVATE)

        self.get_logger().info('pertzepzio_proba_wrapper nodoa hasita.')

    ##########################################################################
    # IRUDIAREN LAGUNTZA FUNTZIOAK
    def zoom_center(self, imagen, escala):
        altura, ancho = imagen.shape[:2]
        nuevo_ancho = int(ancho * escala)
        nueva_altura = int(altura * escala)
        inicio_x = (ancho - nuevo_ancho) // 2
        inicio_y = (altura - nueva_altura) // 2
        centro_crop = imagen[inicio_y:inicio_y + nueva_altura, inicio_x:inicio_x + nuevo_ancho]
        zoomed = cv2.resize(centro_crop, (ancho, altura), interpolation=cv2.INTER_LINEAR)
        return zoomed

    def load_and_preprocess_image(self, image, target_size=(64, 64)):
        canny = cv2.Canny(image, 20, 110)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        thickened_canny = cv2.dilate(canny, kernel, iterations=1)
        image_proc = cv2.cvtColor(thickened_canny, cv2.COLOR_BGR2RGB)
        image_proc = cv2.resize(image_proc, target_size)
        image_proc = img_to_array(image_proc)
        image_proc = np.expand_dims(image_proc, axis=0)
        image_proc = image_proc / 255.0
        return image_proc

    def predic_digit(self, model, image, class_labels):
        prediction = model.predict(image, verbose=0)
        predicted_class = np.argmax(prediction)
        print("Prediction:", class_labels[predicted_class])
        return class_labels[predicted_class]
    ##########################################################################


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
    # IMU harpidetza (IMU SUBSCRIBER)
    def imu_callback(self, msg: Imu):
        self.imu_datuak = msg
        self.imu_datu_berriak = True
    ##########################################################################
    
    ##########################################################################
    # ZERBITZUAREN ZERBITZARIA (SERVICE SERVER) Kolorea eta zenbakia iragartzeko prozesua   
    def koloreaZenbakia_iragarri(self, request, response):
        self.get_logger().info("koloreaZenbakia_iragarri zerbitzua deitu da.")
        
        # Argazkia atera
        self.nodoa_kudeatu('kamera_argitaratzailea', Transition.TRANSITION_CONFIGURE) # Kamera aktibatu
        request_argazkia = Argazkia.Request()                                         # Argazkia zerbitzu eskaeraren mezua sortu
        future = self.argazkia_bezeroa.call_async(request_argazkia)                   #
        rclpy.spin_until_future_complete(self, future)
        argazkia = future.result()
        self.nodoa_kudeatu('kamera_argitaratzailea', Transition.TRANSITION_CLEANUP)

        # Argazkia iragarri
        if argazkia.image is None:
            self.get_logger().error("No hay frame disponible para procesar")
            response.kolorea = -1
            response.zenbakia = -1
            return response      
        
        # Convertir sensor_msgs/Image a imagen OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(argazkia.image, desired_encoding="bgr8")
        frame = cv2.flip(cv_image, 1)
        zoom_frame = self.zoom_center(frame, escala=0.57)
        altura_zoom = zoom_frame.shape[0]
        recorte_vertical = zoom_frame[:int(altura_zoom * 0.70), :]
        ancho_recorte = recorte_vertical.shape[1]
        izquierda = int(ancho_recorte * 0.20)
        derecha = int(ancho_recorte * 0.80)
        final_frame = recorte_vertical[:, izquierda:derecha]
        final_frame = cv2.flip(final_frame, 1)

        final_hsv = cv2.cvtColor(final_frame, cv2.COLOR_BGR2HSV)

        blue_lower_bound = np.array([80, 50, 50])
        blue_upper_bound = np.array([130, 255, 255])
        red_lower_bound1 = np.array([0, 100, 100])
        red_upper_bound1 = np.array([10, 255, 255])
        red_lower_bound2 = np.array([160, 100, 100])
        red_upper_bound2 = np.array([180, 255, 255])

        blue_mask = cv2.inRange(final_hsv, blue_lower_bound, blue_upper_bound)
        red_mask1 = cv2.inRange(final_hsv, red_lower_bound1, red_upper_bound1)
        red_mask2 = cv2.inRange(final_hsv, red_lower_bound2, red_upper_bound2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        blue_contours, _ = cv2.findContours(blue_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_contours, _ = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        color_detected = "Ninguno"
        predicted_number = "NaN"

        self.get_logger().error("Antes de contornos")

        for contour in blue_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if 150 <= w <= 300 and 200 <= h <= 350:
                color_detected = "Azul"
                cropped_image = final_frame[y:y + h, x:x + w]
                preprocessed_img = self.load_and_preprocess_image(cropped_image)
                predicted_number = self.predic_digit(self.model, preprocessed_img, self.class_labels)
                detected = True
                break

        if not detected:
            for contour in red_contours:
                x, y, w, h = cv2.boundingRect(contour)
                if 150 <= w <= 300 and 200 <= h <= 350:
                    color_detected = "Rojo"
                    cropped_image = final_frame[y:y + h, x:x + w]
                    preprocessed_img = self.load_and_preprocess_image(cropped_image)
                    predicted_number = self.predic_digit(self.model, preprocessed_img, self.class_labels)
                    detected = True
                    break
        
        # Convertir sensor_msgs/Image a imagen OpenCV
        try:
            # Guarda la imagen en un fichero (por ejemplo, "capturada.jpg")
            filename = "capturadaWrapper.jpg"
            if cv2.imwrite(filename, cv_image):
                self.get_logger().info(f"Imagen guardada en {filename}")
        except CvBridgeError as e:
            self.get_logger().error("Error al convertir la imagen: {}".format(e))
        
        self.get_logger().info(f"Detección -> Color: {color_detected}, Número: {predicted_number}")
        # Mostrar la imagen usando OpenCV

        color_val = {"Azul": 1, "Rojo": 2}.get(color_detected, -1)

        try:
            number_val = int(predicted_number) if predicted_number != "NaN" else -1
        except Exception as e:
            self.get_logger().error(f"Error al parsear el dígito: {e}")
            number_val = -1

        self.get_logger().info(f"Detección -> Color: {color_detected} ({color_val}), Número: {predicted_number} ({number_val})")

        response.kolorea = color_val
        response.zenbakia = number_val
        return response
    ##########################################################################


    ##########################################################################
    # EKINTZAREN ZERBITZARIA (ACTION SERVER) Bira egiteko prozesua  
    def bira360_callback(self, goal_handle):
        self.nodoa_kudeatu('imu_argitaratzailea', Transition.TRANSITION_CONFIGURE)
        self.nodoa_kudeatu('imu_argitaratzailea', Transition.TRANSITION_ACTIVATE)
        self.nodoa_kudeatu('motorrak_roboclaw', Transition.TRANSITION_CONFIGURE)
        self.nodoa_kudeatu('motorrak_roboclaw', Transition.TRANSITION_ACTIVATE)
        self.nodoa_kudeatu('servoak', Transition.TRANSITION_CONFIGURE)
        self.nodoa_kudeatu('servoak', Transition.TRANSITION_ACTIVATE)

        # Ekintzaren jomuga jaso (ACTION GOAL)
        noranzkoa = goal_handle.request.noranzkoa  # 0: ezkerra, 1: eskuina
        bira_kopurua = goal_handle.request.zenbakia

        self.get_logger().info('Kolorea-Zenbakia ekintzaren jomuga:')
        self.get_logger().info(f'   noranzkoa={noranzkoa}')
        self.get_logger().info(f'   bira kopurua={bira_kopurua}')

        emandako_birak = 0
        hasierako_angelua = self.imu_datuak.euler_angeluak.x
        self.get_logger().info(f'Hasierako angelua: {hasierako_angelua}')
        oraingo_angelua = hasierako_angelua
        anterior = hasierako_angelua
        self.servoak_kokatu()

        if noranzkoa == 0:
            self.motorrak_mugitu([self.abiadura_azkar, self.abiadura_azkar, self.abiadura_azkar*(-1), self.abiadura_azkar*(-1), self.abiadura_azkar, self.abiadura_azkar])
        else:
            self.motorrak_mugitu([self.abiadura_azkar*(-1), self.abiadura_azkar*(-1), self.abiadura_azkar, self.abiadura_azkar, self.abiadura_azkar*(-1), self.abiadura_azkar*(-1)])

        angulo_total = 0
        desfase = 10

        feedback_msg = Bira360.Feedback()
        while angulo_total < (360 * (bira_kopurua-0.2))-desfase:
                rclpy.spin_once(self, timeout_sec=0.1) #?

                if self.imu_datu_berriak:
                    oraingo_angelua = self.imu_datuak.euler_angeluak.x if hasattr(self.imu_datuak, 'euler_angeluak') else oraingo_angelua
                    self.new_imu_data = False                 

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
        
        if noranzkoa:
            self.motorrak_mugitu([self.abiadura_motel*(-1), self.abiadura_motel*(-1), self.abiadura_motel, self.abiadura_motel, self.abiadura_motel*(-1), self.abiadura_motel*(-1)])
        else:
            self.motorrak_mugitu([self.abiadura_motel, self.abiadura_motel, self.abiadura_motel*(-1), self.abiadura_motel*(-1), self.abiadura_motel, self.abiadura_motel])

        while angulo_total < (360 * bira_kopurua) - desfase:
                
            rclpy.spin_once(self, timeout_sec=0.1) #?

            if self.imu_datu_berriak:
                oraingo_angelua = self.imu_datuak.euler_angeluak.x if hasattr(self.imu_datuak, 'euler_angeluak') else oraingo_angelua
                self.new_imu_data = False                 

                # Publicar feedback al cliente
                feedback_msg.oraingo_angelua = oraingo_angelua
                feedback_msg.bira_kopurua = emandako_birak
                self.get_logger().info(f'Oraingo angelua: {feedback_msg.oraingo_angelua}')
                #self.get_logger().info(f'Bira kopurua: {feedback_msg.bira_kopurua}')
                goal_handle.publish_feedback(feedback_msg)

                # Calcula la diferencia entre actual y anterior considerando el ciclo
                if oraingo_angelua != 0.0:
                    if noranzkoa:
                        diferencia = (oraingo_angelua - anterior + 360) % 360
                    else:
                        diferencia = (anterior - oraingo_angelua + 360) % 360
                    anterior = oraingo_angelua  # Actualiza la lectura anterior

                    angulo_total += diferencia

                self.get_logger().info(f'TOTAL: {angulo_total}')

        self.motorrak_gelditu()

        success = True
        goal_handle.succeed()
        
        result = Bira360.Result()
        result.arrakasta = success
        result.amaierako_angelua = oraingo_angelua
        result.hasierako_angelua = hasierako_angelua
        result.bira_kopurua = emandako_birak

        return result
    ##########################################################################

##########################################################################


def main(args=None):
    rclpy.init(args=args)
    node = PertzepzioProbaWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
