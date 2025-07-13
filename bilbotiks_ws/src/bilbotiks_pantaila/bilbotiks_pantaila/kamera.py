import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('camera_lifecycle_node')
        self.get_logger().info("Constructor de CameraLifecycleNode")
        
        # Conversor entre ROS y OpenCV
        self.br = CvBridge()
        self.subscription = None
        self.gui_timer = None  # Timer para actualizar la ventana
        self.window_name = "camera"
        self.last_frame = None  # Aquí se guardará el último frame recibido

        self.is_active = False

    def on_configure(self, state: LifecycleState):
        self.get_logger().info("CameraLifecycleNode on_configure: configurando recursos y abriendo ventana (vacía)")

        try:
            # Crea la ventana redimensionable y la configura
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 800, 600)
            cv2.waitKey(1)  # Actualiza la ventana
            # Crear un timer para actualizar la ventana de forma periódica (p.ej. a 30 fps)
            self.gui_timer = self.create_timer(1/30.0, self.gui_update_callback)
        except Exception as e:
            self.get_logger().error(f"Error al configurar la ventana: {e}")
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info("CameraLifecycleNode on_activate: activando la suscripción y mostrando la imagen de la cámara")
        # Se crea la suscripción para recibir imágenes en el tópico 'kamera_irudia'
        self.subscription = self.create_subscription(
            Image,
            'kamera_irudia',
            self.listener_callback,
            10
        )

        self.is_active = True
        return TransitionCallbackReturn.SUCCESS

    def listener_callback(self, msg):
        if self.is_active:
            #self.get_logger().info("Recibiendo frame de la cámara")
            try:
                # Convertir el mensaje ROS a imagen OpenCV y guardar el último frame
                current_frame = self.br.imgmsg_to_cv2(msg)
                self.last_frame = current_frame
            except Exception as e:
                self.get_logger().error(f"Error al procesar frame: {e}")

    def gui_update_callback(self):
        # Actualiza la ventana con el último frame recibido o sin nada si aún no se recibe imagen
        if self.last_frame is not None:
            cv2.imshow(self.window_name, self.last_frame)
        else:
            # Si no existe un frame, simplemente se actualiza la ventana sin contenido forzado.
            cv2.imshow(self.window_name, 0)
        cv2.waitKey(1)  # Procesa los eventos de la ventana para mantenerla responsive

    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info("CameraLifecycleNode on_deactivate: desactivando la suscripción y congelando la imagen")
        
        self.is_active = False
        
        # Destruir la suscripción para dejar de recibir nuevos frames
        if self.subscription:
            self.subscription = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info("CameraLifecycleNode on_cleanup: limpiando recursos y cerrando la ventana")
        if self.gui_timer:
            self.gui_timer.cancel()
            self.gui_timer = None
        self.last_frame = None
        # Cerrar todas las ventanas creadas por OpenCV
        cv2.destroyAllWindows()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("CameraLifecycleNode on_shutdown: apagando nodo")
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraLifecycleNode()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        camera_node.get_logger().info("Nodo interrumpido por el usuario")
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
