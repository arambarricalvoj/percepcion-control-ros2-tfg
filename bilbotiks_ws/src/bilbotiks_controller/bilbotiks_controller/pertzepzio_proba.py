##########################################################################
# ROS2 imports...
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from lifecycle_msgs.srv import ChangeState

from bilbotiks_interfazeak.srv import KoloreaZenbakia
from bilbotiks_interfazeak.action import Bira360
##########################################################################


##########################################################################
class PertzepzioProba(Node):
    def __init__(self):
        super().__init__('pertzepzio_proba')

        # Cliente de servicio
        self.cli_zerbitzua = self.create_client(KoloreaZenbakia, 'koloreaZenbakia_iragarri')
        
        # Esperar hasta que el servicio esté disponible
        while not self.cli_zerbitzua.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('KoloreaZenbakia zerbitzua itxaroten...')

        # Cliente de acción
        self.cli_ekintza = ActionClient(self, Bira360, 'pertzepzio_proba_360bira')
    
    def zerbitzua_deitu(self):
        # Solicitar servicio
        request = KoloreaZenbakia.Request()

        future = self.cli_zerbitzua.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Zerbitzuaren erantzuna: {future.result().zenbakia}, {future.result().kolorea}')
        else:
            self.get_logger().error('Errorea zerbitzua deitutakoan')

        return future.result().kolorea, future.result().zenbakia

    def ekintza_deitu(self, norabidea, bira_kopurua):
        # Llamar a la acción
        self.get_logger().info(f'{bira_kopurua} bira {"eskuinara" if norabidea == 1 else "ezkerretara"} egiteko ekintza deitu')

        goal_msg = Bira360.Goal()
        goal_msg.noranzkoa = norabidea  # 1 para derecha, 0 para izquierda
        goal_msg.zenbakia = bira_kopurua  # Número de bira_kopurua

        # Llamar a la acción y esperar la respuesta
        self.cli_ekintza.wait_for_server()

        # Enviar la solicitud de acción
        future = self.cli_ekintza.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info(f'Ekintza arrakastaz bidalita: {future.result()}')
        else:
            self.get_logger().error('Errorea ekintza bidalitakoan')

        # Asignar el resultado del future a goal_handle
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('El objetivo no fue aceptado por el servidor de acciones.')
            return

        self.get_logger().info('Objetivo aceptado. Esperando resultado...')

        # Esperar al resultado de la acción
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result is not None:
            self.get_logger().info(f'Resultado de la acción: {result.result}')
        else:
            self.get_logger().error('No se recibió el resultado de la acción.')
##########################################################################


def main(args=None):
    rclpy.init(args=args)
    bezeroa = PertzepzioProba()

    # Kolorea eta zenbakia iragarri
    kolorea, zenbakia = bezeroa.zerbitzua_deitu()

    # Bira egin (kolorea 1: eskuina; kolorea 2: ezkerra)
    if kolorea == 2:
        norabidea = True
    elif kolorea == 1:
        norabidea = False
    else:
        bezeroa.destroy_node()
        rclpy.shutdown()
    bezeroa.ekintza_deitu(norabidea=norabidea, bira_kopurua=zenbakia)

    # Nodoa amaitu
    bezeroa.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()