#include <gz/msgs/twist.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>

std::string topic_pub = "/cmd_vel";   // Publicar en este tema
gz::transport::Node node;
auto pub = node.Advertise<gz::msgs::Twist>(topic_pub);

void cb(const gz::msgs::LaserScan &_msg)
{
  gz::msgs::Twist data;

  // Parámetros
  float target_distance_side = 1.0; // Distancia deseada a la pared lateral
  int index_left = 639;            // Índice para la pared izquierda (+90°)
  int index_right = 0;            // Índice para la pared derecha (-90°)
  int index_front = 320;           // Índice para el frente (0°)
  float threshold_front = 1.5;     // Umbral para evitar colisiones frontales

  ///////////////////////////////////
  // Leer distancias del Lidar
  /* ESTRATEGIA 1 - MEDICIONES FIJAS*/
  //float distance_left = _msg.ranges(index_left);
  //float distance_right = _msg.ranges(index_right);
  //float distance_front = _msg.ranges(index_front);

  /* ESTRATEGIA 2 - DISTANCIA MÍNIMA EN UN RANGO*/
  // Lado izquierdo: índices 320 a 639
  float distance_left = std::numeric_limits<float>::infinity();
  for (int i = 320; i <= 639; ++i) {
      if (_msg.ranges(i) < distance_left) {
          distance_left = _msg.ranges(i);
      }
  }
  // Lado derecho: índices 0 a 320
  float distance_right = std::numeric_limits<float>::infinity();
  for (int i = 0; i <= 320; ++i) {
      if (_msg.ranges(i) < distance_right) {
          distance_right = _msg.ranges(i);
      }
  }
  ///////////////////////////////////

  // Calcular error lateral
  //float error_side = target_distance_side - distance_side;

  float error_side = distance_left - distance_right;

  // Control proporcional para corrección lateral
  float k_p_side = 1.0; // Ganancia proporcional para la pared lateral
  float angular_correction = k_p_side * error_side;

  data.mutable_linear()->set_x(0.5);        // Velocidad constante hacia adelante
  data.mutable_angular()->set_z(angular_correction); // Corrección basada en la pared izquierda

  // Evitar colisiones frontales
  /*if (distance_front < threshold_front)
  {
    // Reducir velocidad y priorizar el giro
    data.mutable_linear()->set_x(0.1);         // Velocidad baja hacia adelante
    data.mutable_angular()->set_z(-0.5);        // Gira hacia la izquierda para evitar colisión
  }
  else
  {
    // Movimiento normal: avanza con corrección lateral
    data.mutable_linear()->set_x(0.5);        // Velocidad constante hacia adelante
    data.mutable_angular()->set_z(angular_correction); // Corrección basada en la pared izquierda
  }*/

  // Publicar comandos de movimiento
  pub.Publish(data);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  std::string topic_sub = "/lidar";   // Suscribirse a este tema
  // Suscribirse a un tema registrando un callback.
  if (!node.Subscribe(topic_sub, cb))
  {
    std::cerr << "Error suscribiéndose al tema [" << topic_sub << "]" << std::endl;
    return -1;
  }

  // Esperar por la señal de apagado.
  gz::transport::waitForShutdown();

  return 0;
}
