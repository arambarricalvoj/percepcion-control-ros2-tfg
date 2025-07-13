#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState
import tkinter as tk
import sys
from bilbotiks_interfazeak.msg import Kontrola

class KontrolPantaila(LifecycleNode):
    def __init__(self):
        super().__init__('kontrol_pantaila')
        self.get_logger().info("/kontrol_pantaila nodoa IN constructor")
        
        # Jasoko diren datuak gordetzeko aldagaiak
        self.ezker_distantzia = 0
        self.aurreko_distantzia = 0
        self.eskuin_distantzia = 0
        
        # tkinter leihoak eta etiketak
        self.window = None
        self.label_dato1 = None
        self.label_dato2 = None
        self.label_dato3 = None

        # Etiquetas descriptivas
        self.label_ezkerra = None
        self.label_aurre = None
        self.label_eskuina = None
        
        # Timer para actualizar la ventana Tkinter
        self.timer = None

        # Suscripción al tópico (se crea en on_configure)
        self.subscription = None

        self.is_activate = False

    def on_configure(self, state: LifecycleState):
        self.get_logger().info("/kontrol_pantaila nodoa IN on_configure")
        try:
            # Crear la ventana principal y ajustar la geometría para disponer de dos filas
            self.window = tk.Tk()
            self.window.geometry("600x150")
            self.window.title("Kontrol probaren pantaila")
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
            self.label_ezkerra = tk.Label(self.window, text="Izquierda", font=descr_font, bg="white", fg="black")
            self.label_ezkerra.grid(row=1, column=0, padx=10, pady=2)

            self.label_aurre = tk.Label(self.window, text="Frente", font=descr_font, bg="white", fg="black")
            self.label_aurre.grid(row=1, column=1, padx=10, pady=2)

            self.label_eskuina = tk.Label(self.window, text="Derecha", font=descr_font, bg="white", fg="black")
            self.label_eskuina.grid(row=1, column=2, padx=10, pady=2)
            
            # Configurar el cierre de la ventana para liberar recursos
            self.window.protocol("WM_DELETE_WINDOW", self._on_window_close)
            
            # Crear la suscripción al tópico /pertzepzioa_pantaila
            self.subscription = self.create_subscription(
                Kontrola,
                '/kontrol_pantaila',
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
            self.ezker_distantzia = round(msg.ezker_distantzia, 2)
            self.aurreko_distantzia = round(msg.aurreko_distantzia, 2)
            self.eskuin_distantzia = round(msg.eskuin_distantzia, 2)


            # Actualizar las etiquetas con los datos
            self.label_dato1.config(text=f"{self.ezker_distantzia}")
            self.label_dato2.config(text=f"{self.aurreko_distantzia}")
            self.label_dato3.config(text=f"{self.eskuin_distantzia}")
            
            # La línea de log se puede descomentar para visualizar la info recibida
            # self.get_logger().info(f"Mensaje recibido: {self.ezker_distantzia}, {self.aurreko_distantzia}, {self.eskuin_distantzia} (norabidea: {self.norabidea})")

    def _on_window_close(self):
        self.get_logger().info("Leihoa ixteko eskatu da.")
        if self.window:
            self.window.destroy()
        rclpy.shutdown()
        sys.exit(0)
    
    def on_activate(self, state: LifecycleState):
        self.get_logger().info("/kontrol_pantaila nodoa IN on_activate")
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
        self.get_logger().info("/kontrol_pantaila nodoa IN on_deactivate")
        self.is_activate = False
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info("/kontrol_pantaila nodoa IN on_cleanup")
        if self.window:
            self.window.destroy()
            self.window = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("/kontrol_pantaila nodoa IN on_shutdown")
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    interface_node = KontrolPantaila()
    try:
        rclpy.spin(interface_node)
    except KeyboardInterrupt:
        interface_node.get_logger().info("Erabiltzaileak nodoa gelditu du.")
    finally:
        interface_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
