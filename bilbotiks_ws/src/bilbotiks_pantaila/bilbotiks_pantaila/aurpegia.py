import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from bilbotiks_interfazeak.msg import Aurpegia  # Usamos el mensaje tal como está
import pygame
import sys
import random

class AurpegiaPantaila(LifecycleNode):
    def __init__(self):
        super().__init__('aurpegia')
        self.get_logger().info("Constructor de AurpegiaPantaila")
        
        # Variables que se inicializarán en on_configure
        self.screen = None
        self.WIDTH = None
        self.HEIGHT = None
        self.clock = None
        self.blink_timer = 0
        self.eyes_closed = False
        self.min_open_time = 500   # ms
        self.max_open_time = 2000  # ms
        self.closed_time = 150     # ms
        self.next_blink_interval = None
        
        # Dirección que se selecciona a partir de los mensajes, por defecto "center"
        self.desired_direction = "center"  # Puede ser "center", "left" o "right"
        
        # Referencias a los sprites (se cargarán en on_configure)
        self.left_eye_open = None
        self.left_eye_closed = None
        self.right_eye_open = None
        self.right_eye_closed = None
        
        self.left_eye_left = None
        self.right_eye_left = None
        self.left_eye_right = None
        self.right_eye_right = None
        
        self.smile = None
        self.smile_left = None
        self.smile_right = None
        
        # Posiciones en pantalla de los sprites
        self.left_eye_pos = (200, 150)
        self.right_eye_pos = (500, 150)
        self.smile_pos = (300, 300)
        
        # Banderas y apuntadores para la suscripción y el timer
        self.deactivated = False             # Controla si se pausa la animación/blinking
        self.subscription_active = False     # Controla si se deben procesar mensajes (on_activate)
        self.subscription = None
        self.timer = None

    def on_configure(self, state: LifecycleState):
        self.get_logger().info("AurpegiaPantaila en on_configure - Configurando ventana y sprites")
        # Inicializar Pygame y configurar la ventana
        pygame.init()
        self.WIDTH, self.HEIGHT = 800, 600
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("AurpegiaPantaila: Cara del Robot")
        
        # En este ejemplo se usa una ruta absoluta para cargar los recursos instalados.
        # Puedes ajustar la forma en que se obtiene la ruta, por ejemplo usando pkg_resources o importlib.resources.
        path = '/home/javierac/mounted/moverRover/bilbotiks_lifecyle_ws/install/bilbotiks_pantaila/lib/python3.10/site-packages/bilbotiks_pantaila/aurpegia_irudiak'
        
        # Cargar los sprites de los ojos y la boca
        try:
            self.left_eye_open = pygame.image.load(f"{path}/izquierda_abierto.png").convert_alpha()
            self.left_eye_closed = pygame.image.load(f"{path}/izquierda_cerrado.png").convert_alpha()
            self.right_eye_open = pygame.image.load(f"{path}/derecha_abierto.png").convert_alpha()
            self.right_eye_closed = pygame.image.load(f"{path}/derecha_cerrado.png").convert_alpha()
            
            # Sprites para mirar a los lados
            self.left_eye_left = pygame.image.load(f"{path}/mirar_izquierda_trans.png").convert_alpha()
            self.right_eye_left = pygame.image.load(f"{path}/mirar_izquierda_trans.png").convert_alpha()
            self.left_eye_right = pygame.image.load(f"{path}/mirar_derecha_trans.png").convert_alpha()
            self.right_eye_right = pygame.image.load(f"{path}/mirar_derecha_trans.png").convert_alpha()
            
            # Cargar los sprites de la boca
            self.smile = pygame.image.load(f"{path}/smile.png").convert_alpha()
            self.smile_left = pygame.image.load(f"{path}/smile_izquierda.png").convert_alpha()
            self.smile_right = pygame.image.load(f"{path}/smile_derecha.png").convert_alpha()
        except Exception as e:
            self.get_logger().error(f"Error al cargar sprites: {e}")
            return TransitionCallbackReturn.FAILURE
        
        # Escalar los sprites
        eye_size = (100, 100)
        self.left_eye_open = pygame.transform.scale(self.left_eye_open, eye_size)
        self.left_eye_closed = pygame.transform.scale(self.left_eye_closed, eye_size)
        self.right_eye_open = pygame.transform.scale(self.right_eye_open, eye_size)
        self.right_eye_closed = pygame.transform.scale(self.right_eye_closed, eye_size)
        self.left_eye_left = pygame.transform.scale(self.left_eye_left, eye_size)
        self.right_eye_left = pygame.transform.scale(self.right_eye_left, eye_size)
        self.left_eye_right = pygame.transform.scale(self.left_eye_right, eye_size)
        self.right_eye_right = pygame.transform.scale(self.right_eye_right, eye_size)
        
        smile_size = (200, 100)
        self.smile = pygame.transform.scale(self.smile, smile_size)
        self.smile_left = pygame.transform.scale(self.smile_left, smile_size)
        self.smile_right = pygame.transform.scale(self.smile_right, smile_size)
        
        # Inicializar el reloj y el temporizador del parpadeo
        self.clock = pygame.time.Clock()
        self.blink_timer = 0
        self.next_blink_interval = random.randint(self.min_open_time, self.max_open_time)
        
        self.get_logger().info("AurpegiaPantaila configurado correctamente")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info("AurpegiaPantaila en on_activate")
        # Activar tanto la animación como la suscripción
        self.deactivated = False
        self.subscription_active = True

        # Crear la suscripción al tópico /aurpegia (con mensaje de tipo Aurpegia)
        self.subscription = self.create_subscription(
            Aurpegia,
            '/aurpegia',
            self.aurpegia_callback,
            10
        )
        # Iniciar un timer para actualizar la animación (alrededor de 60 fps)
        if self.timer is None:
            self.timer = self.create_timer(1/60.0, self.timer_callback)
        return TransitionCallbackReturn.SUCCESS

    def aurpegia_callback(self, msg):
        # Solo procesamos si la suscripción está activa
        if not self.subscription_active:
            return

        """
        Callback para actualizar la dirección según el valor recibido en msg.begi_norabidea.
        0 → "center", 1 → "left", 2 → "right".
        """
        valor = msg.begi_norabidea
        if valor == 0:
            self.desired_direction = "center"
        elif valor == 1:
            self.desired_direction = "left"
        elif valor == 2:
            self.desired_direction = "right"
        else:
            self.get_logger().warn("Valor desconocido recibido, fijando a 'center'")
            self.desired_direction = "center"
        self.get_logger().info(f"Dirección actualizada a: {self.desired_direction}")

    def timer_callback(self):
        dt = self.clock.tick(60)
        # Procesar eventos de Pygame para mantener la ventana responsive
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        
        # Si el nodo está activado (no desactivado), actualizamos el parpadeo; 
        # si está desactivado, mantenemos la imagen fija.
        if not self.deactivated:
            self.blink_timer += dt
            if not self.eyes_closed:
                if self.blink_timer > self.next_blink_interval:
                    self.eyes_closed = True
                    self.blink_timer = 0
                    self.next_blink_interval = self.closed_time
            else:
                if self.blink_timer > self.next_blink_interval:
                    self.eyes_closed = False
                    self.blink_timer = 0
                    self.next_blink_interval = random.randint(self.min_open_time, self.max_open_time)
        else:
            # En modo desactivado, forzamos un estado estático (por ejemplo, ojos abiertos)
            self.eyes_closed = False
        
        # Dibujar fondo y sprites según la dirección y el estado (blinking o estático)
        self.screen.fill((135, 206, 235))  # Color azul cielo
        if not self.eyes_closed:
            if self.desired_direction == "center":
                self.screen.blit(self.left_eye_open, self.left_eye_pos)
                self.screen.blit(self.right_eye_open, self.right_eye_pos)
            elif self.desired_direction == "left":
                self.screen.blit(self.left_eye_left, self.left_eye_pos)
                self.screen.blit(self.right_eye_left, self.right_eye_pos)
            elif self.desired_direction == "right":
                self.screen.blit(self.left_eye_right, self.left_eye_pos)
                self.screen.blit(self.right_eye_right, self.right_eye_pos)
        else:
            self.screen.blit(self.left_eye_closed, self.left_eye_pos)
            self.screen.blit(self.right_eye_closed, self.right_eye_pos)
        
        # Seleccionar y dibujar la expresión de la boca según la dirección
        if self.desired_direction == "center":
            current_smile = self.smile
        elif self.desired_direction == "left":
            current_smile = self.smile_left
        elif self.desired_direction == "right":
            current_smile = self.smile_right
        
        self.screen.blit(current_smile, self.smile_pos)
        pygame.display.flip()

    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info("AurpegiaPantaila en on_deactivate")
        # En on_deactivate, la animación se pausa y la suscripción se inactiva
        self.deactivated = True
        self.subscription_active = False
        self.subscription = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info("AurpegiaPantaila en on_cleanup")
        self.subscription = None
        # Si se destruye la ventana, se llama a pygame.quit()
        self.screen = None
        pygame.quit()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("AurpegiaPantaila en on_shutdown. Cerrando Pygame")
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    aurpegia_pantaila = AurpegiaPantaila()
    try:
        rclpy.spin(aurpegia_pantaila)
    except KeyboardInterrupt:
        aurpegia_pantaila.get_logger().info("Nodo interrumpido por el usuario")
    finally:
        aurpegia_pantaila.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
