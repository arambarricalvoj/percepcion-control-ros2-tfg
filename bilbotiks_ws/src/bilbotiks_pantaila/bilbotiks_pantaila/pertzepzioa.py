#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import pygame
import numpy as np

# Asegúrate de que el paquete bilbotiks_interfazeak esté en tu path de ROS2
from bilbotiks_interfazeak.msg import Pertzepzioa

class CombinedDisplayNode(Node):
    def __init__(self):
        super().__init__('combined_display_node')
        self.get_logger().info('Nodo combined_display_node iniciado.')

        # Variables para recoger los 4 datos (aunque solo se muestran 2)
        self.imu = 0
        self.emandako_birak = 0  # Se recoge pero no se visualiza
        self.bira_totalak = 0
        self.norabidea = 0  # Se usará para definir el color de "Vueltas totales"

        # Suscripción para recibir mensajes del tópico de sensores
        self.create_subscription(
            Pertzepzioa,
            '/pertzepzioa_pantaila',
            self.sensor_callback,
            10
        )

        # Inicialización de la cámara
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Error: no se pudo acceder a la cámara")
            rclpy.shutdown()
            return

        # Configuración de Pygame:
        pygame.init()
        # Ventana de 1280x400 dividida en 3 columnas: izquierda (datos), central (cámara) y derecha (datos)
        self.width, self.height = 1280, 400
        self.col_width = self.width // 3
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.FULLSCREEN)
        pygame.display.set_caption("Combined Display: Webcam y Datos Sensor")
        self.clock = pygame.time.Clock()
        self.fullscreen = True  # Bandera para alternar pantalla completa

        # Nuevos tamaños de fuentes:
        # - Fuente para los labels descriptivos: 54 puntos (50% mayor que 36)
        # - Fuente para los datos: 120 puntos (100% mayor que 60)
        self.font_small = pygame.font.SysFont("Arial", 54)
        self.font_big = pygame.font.SysFont("Arial", 120)

    def sensor_callback(self, msg: Pertzepzioa):
        # Actualizamos todas las variables con los datos recibidos
        self.imu = msg.imu
        self.emandako_birak = msg.emandako_birak  # Recogido pero no se muestra
        self.bira_totalak = msg.bira_totalak
        self.norabidea = msg.norabidea

    def run(self):
        running = True
        while rclpy.ok() and running:
            # Procesamos eventos de Pygame para permitir cierre y alternar fullscreen
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        if self.fullscreen:
                            self.screen = pygame.display.set_mode((self.width, self.height))
                            self.fullscreen = False
                        else:
                            self.screen = pygame.display.set_mode((self.width, self.height), pygame.FULLSCREEN)
                            self.fullscreen = True

            # Leer y procesar el frame de la cámara para la columna central
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Error: no se pudo leer la imagen de la cámara")
                running = False
                break

            webcam_frame = cv2.resize(frame, (self.col_width, self.height))
            webcam_frame = cv2.cvtColor(webcam_frame, cv2.COLOR_BGR2RGB)
            frame_surface = pygame.surfarray.make_surface(np.rot90(webcam_frame))

            # Componer la ventana:
            self.screen.fill((0, 0, 0))
            # Columna izquierda: fondo blanco
            left_rect = pygame.Rect(0, 0, self.col_width, self.height)
            pygame.draw.rect(self.screen, (255, 255, 255), left_rect)
            # Columna central: fondo negro (para la cámara)
            center_rect = pygame.Rect(self.col_width, 0, self.col_width, self.height)
            pygame.draw.rect(self.screen, (0, 0, 0), center_rect)
            # Columna derecha: fondo blanco
            right_rect = pygame.Rect(2 * self.col_width, 0, self.col_width, self.height)
            pygame.draw.rect(self.screen, (255, 255, 255), right_rect)

            # Mostrar la imagen de la cámara en la columna central
            self.screen.blit(frame_surface, (self.col_width, 0))

            # Reposicionar los textos en las columnas laterales:
            # En cada columna el grupo (descriptivo + dato) se centra verticalmente en 200.
            # Se posiciona el label descriptivo más arriba para evitar solapamiento.
            # Por ejemplo, en la columna izquierda:
            left_center_x = self.col_width // 2
            imu_label_text = self.font_small.render("IMU", True, (0, 0, 0))
            # Ubicamos el label descriptivo en y = 100 (más arriba)
            imu_label_rect = imu_label_text.get_rect(center=(left_center_x, 100))
            imu_value_text = self.font_big.render(str(self.imu), True, (0, 0, 0))
            # Ubicamos el dato en y = 300 (más abajo)
            imu_value_rect = imu_value_text.get_rect(center=(left_center_x, 300))
            self.screen.blit(imu_label_text, imu_label_rect)
            self.screen.blit(imu_value_text, imu_value_rect)

            # En la columna derecha (para "Vueltas totales"):
            right_center_x = 2 * self.col_width + (self.col_width // 2)
            vueltas_label_text = self.font_small.render("Vueltas totales", True, (0, 0, 0))
            vueltas_label_rect = vueltas_label_text.get_rect(center=(right_center_x, 100))
            vueltas_color = (255, 0, 0) if self.norabidea == 0 else (0, 0, 255)
            vueltas_value_text = self.font_big.render(str(self.bira_totalak), True, vueltas_color)
            vueltas_value_rect = vueltas_value_text.get_rect(center=(right_center_x, 300))
            self.screen.blit(vueltas_label_text, vueltas_label_rect)
            self.screen.blit(vueltas_value_text, vueltas_value_rect)

            pygame.display.flip()
            self.clock.tick(30)  # Limitar a 30 FPS

            # Permitir procesar callbacks ROS sin bloquear
            rclpy.spin_once(self, timeout_sec=0.001)

        self.cap.release()
        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    node = CombinedDisplayNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por el usuario")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
