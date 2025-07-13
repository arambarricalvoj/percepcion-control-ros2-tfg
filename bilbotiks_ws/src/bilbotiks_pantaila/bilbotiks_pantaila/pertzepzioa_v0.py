#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState
import tkinter as tk
import sys

# Asegúrate de que el paquete bilbotiks_interfazeak esté en tu path de ROS2
from bilbotiks_interfazeak.msg import Pertzepzioa

class PertzepzioPantaila(LifecycleNode):
    def __init__(self):
        super().__init__('pertzepzio_pantaila')
        self.get_logger().info("/pertzepzio_pantaila nodoa IN constructor")
        
        # Variables para almacenar los datos recibidos
        self.imu = 0
        self.emandako_birak = 0
        self.bira_totalak = 0
        self.norabidea = 0  # Este valor se usa para determinar el color, pero no se visualiza
        
        # Ventana y widgets de tkinter
        self.window = None
        self.label_dato1 = None
        self.label_dato2 = None
        self.label_dato3 = None

        # Etiquetas descriptivas
        self.label_descr_imu = None
        self.label_descr_em_birak = None
        self.label_descr_bira_totalak = None
        
        # Timer para actualizar la ventana Tkinter
        self.timer = None

        # Suscripción al tópico (se crea en on_configure)
        self.subscription = None

        self.is_activate = False

    def on_configure(self, state: LifecycleState):
        self.get_logger().info("/pertzepzio_pantaila nodoa IN on_configure")
        try:
            # Crear la ventana principal y ajustar la geometría para disponer de dos filas
            self.window = tk.Tk()
            self.window.geometry("600x150")
            self.window.title("Pertzepzio probaren pantaila")
            self.window.configure(bg="white")
            
            # Configurar las columnas para que se distribuyan de forma homogénea
            self.window.columnconfigure(0, weight=1)
            self.window.columnconfigure(1, weight=1)
            self.window.columnconfigure(2, weight=1)
            
            # Fuente para el texto en la interfaz
            label_font = ("Arial", 20)
            
            # Fila 0: mostrar los datos
            self.label_dato1 = tk.Label(self.window, text="0", font=label_font, bg="white", fg="black")
            self.label_dato1.grid(row=0, column=0, padx=10, pady=5)

            self.label_dato2 = tk.Label(self.window, text="0", font=label_font, bg="white", fg="black")
            self.label_dato2.grid(row=0, column=1, padx=10, pady=5)

            # En la columna derecha se mostrará el dato con color variable
            self.label_dato3 = tk.Label(self.window, text="0", font=label_font, bg="white", fg="black")
            self.label_dato3.grid(row=0, column=2, padx=10, pady=5)
            
            # Fila 1: etiquetas descriptivas para identificar cada dato
            descr_font = ("Arial", 14)
            self.label_descr_imu = tk.Label(self.window, text="IMU", font=descr_font, bg="white", fg="black")
            self.label_descr_imu.grid(row=1, column=0, padx=10, pady=2)

            self.label_descr_em_birak = tk.Label(self.window, text="Vueltas dadas", font=descr_font, bg="white", fg="black")
            self.label_descr_em_birak.grid(row=1, column=1, padx=10, pady=2)

            self.label_descr_bira_totalak = tk.Label(self.window, text="Vueltas totales", font=descr_font, bg="white", fg="black")
            self.label_descr_bira_totalak.grid(row=1, column=2, padx=10, pady=2)
            
            # Configurar el cierre de la ventana para liberar recursos
            self.window.protocol("WM_DELETE_WINDOW", self._on_window_close)
            
            # Crear la suscripción al tópico /pertzepzioa_pantaila
            self.subscription = self.create_subscription(
                Pertzepzioa,
                '/pertzepzioa_pantaila',
                self.subscription_callback,
                10
            )
        except Exception as e:
            self.get_logger().error("Error al configurar tkinter o la suscripción: " + str(e))
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info("Interfaz y suscripción configuradas correctamente")
        return TransitionCallbackReturn.SUCCESS

    def subscription_callback(self, msg):
        if self.is_activate:
            # Actualizar los datos a partir del mensaje recibido.
            # Se asume que el mensaje Pertzepzioa tiene los atributos: imu, emandako_birak, bira_totalak y norabidea.
            self.imu = msg.imu
            self.emandako_birak = msg.emandako_birak
            self.bira_totalak = msg.bira_totalak
            self.norabidea = msg.norabidea  # Este campo se utiliza para determinar el color de la tercera columna

            # Actualizar las etiquetas con los datos
            self.label_dato1.config(text=f"{self.imu}")
            self.label_dato2.config(text=f"{self.emandako_birak}")
            
            # Decidir el color de la etiqueta derecha según el valor de norabidea:
            color = "red" if self.norabidea == 0 else "blue"
            self.label_dato3.config(text=f"{self.bira_totalak}", fg=color)
            
            # La línea de log se puede descomentar para visualizar la info recibida
            # self.get_logger().info(f"Mensaje recibido: {self.imu}, {self.emandako_birak}, {self.bira_totalak} (norabidea: {self.norabidea})")

    def _on_window_close(self):
        self.get_logger().info("Leihoa ixteko eskatu da.")
        if self.window:
            self.window.destroy()
        rclpy.shutdown()
        sys.exit(0)
    
    def on_activate(self, state: LifecycleState):
        self.get_logger().info("/pertzepzio_pantaila nodoa IN on_activate")
        self.is_activate = True
        # Crear un timer para refrescar la ventana (por ejemplo, 30 fps)
        if self.timer is None:
            self.timer = self.create_timer(1 / 30.0, self.timer_callback)
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        # Actualizar la ventana de tkinter para reflejar los cambios realizados en la suscripción
        if self.window:
            try:
                self.window.update_idletasks()
                self.window.update()
            except tk.TclError:
                pass

    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info("/pertzepzio_pantaila nodoa IN on_deactivate")
        self.is_activate = False
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info("/pertzepzio_pantaila nodoa IN on_cleanup")
        if self.window:
            self.window.destroy()
            self.window = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("/pertzepzio_pantaila nodoa IN on_shutdown")
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    interface_node = PertzepzioPantaila()
    try:
        rclpy.spin(interface_node)
    except KeyboardInterrupt:
        interface_node.get_logger().info("Erabiltzaileak nodoa gelditu du.")
    finally:
        interface_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
