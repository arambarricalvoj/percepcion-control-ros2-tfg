import rclpy
from rclpy.node import Node
from bilbotiks_interfazeak.srv import Argazkia  # Reemplaza "your_package_name" por el nombre de tu paquete
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array

import time
import tensorflow as tf

# Cargar el modelo previamente entrenado
model = load_model('/home/bilbotiks/Desktop/modelo_100epochs.keras')

# Define los nombres de tus clases
class_labels = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "NaN"]

# Recortar un 15% por todos los lados
def recortar_imagen(imagen, porcentaje_recorte=0.20):
    altura, ancho = imagen.shape[:2]
    recorte_vertical = int(altura * porcentaje_recorte)
    recorte_horizontal = int(ancho * porcentaje_recorte)
    return imagen[recorte_vertical:altura - recorte_vertical, recorte_horizontal:ancho - recorte_horizontal]


def load_and_preprocess_image(image, target_size=(64, 64)):
    # Aplicar flip horizontal
    image = cv2.flip(image, 1)

    # Recortar la imagen
    image = recortar_imagen(image)
    
    # Aplicar detección de bordes con Canny
    canny = cv2.Canny(image, 20, 110)
    
    # Engrosar los contornos usando dilatación
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))  # Tamaño del kernel
    thickened_canny = cv2.dilate(canny, kernel, iterations=1)  # Aumentar iteraciones para engrosar más
    
    #filename = f"new/{count}.png"
    #cv2.imwrite(filename, thickened_canny)
    
    """Carga y preprocesa una imagen"""
    image = cv2.cvtColor(thickened_canny, cv2.COLOR_BGR2RGB)  # Convertir a RGB
    image = cv2.resize(image, target_size)  # Redimensionar
    image = img_to_array(image)  # Convertir a array
    image = np.expand_dims(image, axis=0)  # Añadir una dimensión extra
    image = image / 255.0  # Normalizar
    return image

def predic_digit(image):
    prediction = model.predict(image, verbose=0)
    predicted_class = np.argmax(prediction)
    print(class_labels[predicted_class])
    return class_labels[predicted_class]

class TakePhotoClient(Node):
    def __init__(self):
        super().__init__('take_photo_client')
        # Crear el cliente para el servicio "tomar_foto"
        self.cli = self.create_client(Argazkia, 'argazkia_atera')
        self.bridge = CvBridge()
        
        # Esperamos a que el servicio esté disponible
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando por el servicio "argazkia_atera"...')
        
        # En este ejemplo se hace una única llamada al servicio.
        self.send_request()
    
    def send_request(self):
        self.get_logger().info('Enviando solicitud para capturar la imagen...')
        # Como el request está vacío, se crea de esta forma.
        request = Argazkia.Request()
        future = self.cli.call_async(request)
        future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('La llamada al servicio falló: %r' % (e,))
            return
        
        # El campo "image" en la respuesta contiene el mensaje sensor_msgs/Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(response.image, desired_encoding="bgr8")
            frame = cv2.flip(cv_image, 1)
            zoom_frame = frame#zoom_center(frame, escala=0.50)
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
                    #self.count += 1
                    preprocessed_img = load_and_preprocess_image(cropped_image)
                    predicted_number = predic_digit(preprocessed_img)
                    detected = True
                    break

            if not detected:
                for contour in red_contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    if 150 <= w <= 300 and 200 <= h <= 350:
                        color_detected = "Rojo"
                        cropped_image = final_frame[y:y + h, x:x + w]
                        #self.count += 1
                        preprocessed_img = load_and_preprocess_image(cropped_image)
                        predicted_number = predic_digit(preprocessed_img)
                        detected = True
                        break
            
            # Convertir sensor_msgs/Image a imagen OpenCV
            try:
                # Guarda la imagen en un fichero (por ejemplo, "capturada.jpg")
                filename = "capturadaKameraSub.jpg"
                if cv2.imwrite(filename, cv_image):
                    self.get_logger().info(f"Imagen guardada en {filename}")
            except CvBridgeError as e:
                self.get_logger().error("Error al convertir la imagen: {}".format(e))
            
            self.get_logger().info(f"Detección -> Color: {color_detected}, Número: {predicted_number}")
            # Mostrar la imagen usando OpenCV
        except CvBridgeError as e:
            self.get_logger().error("Error al convertir la imagen: %s" % str(e))


def main(args=None):
    rclpy.init(args=args)
    node = TakePhotoClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
