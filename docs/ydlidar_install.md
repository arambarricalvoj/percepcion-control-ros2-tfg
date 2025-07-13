# Instalación de `YDLIDAR/ydlidar_ros2_driver`

Repositorio SDK: https://github.com/YDLIDAR/YDLidar-SDK

Repositorio oficial: https://github.com/YDLIDAR/ydlidar_ros2_driver

Repositorio adaptado: https://github.com/Oyefusi-Samuel/ydlidar_ros2_driver-master

Instrucciones para instalar el paquete `ydlidar_ros2_driver` en `ROS2 Humble`.

## 1. Instalación de ROS Humble

Seguir las instrucciones de la documentación oficial: https://docs.ros.org/en/humble/Installation.html.

Para redactar esta guía se ha instalado mediante `deb packages`: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html


## 2. Configurar la variable `$ROS_DISTRO`
Activar ROS2 Humble:
```bash
source /opt/ros/humble/setup.bash
```
Comprobar que la variables de entorno `$ROS_DISTRO`está correctamente configurada:
```bash
$ echo $ROS_DISTRO
```

El output del comando debe ser el siguiente:
```bash
humble
```

Si el output está vacío, declarar la variable (cuidado! si no se ha activado ROS2 la variable siempre saldrá vacía, acuérdate de activarlo primero):
```bash
export ROS_DISTRO=humble
```

Más información sobre cómo gestionar las variables de entorno y persistencia de estas variables: https://www.serverlab.ca/tutorials/linux/administration-linux/how-to-set-environment-variables-in-linux/

## 3. Instalar SDK
```
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
```

## 4. Instalar dependencias y librerías
```bash
sudo apt install cmake pkg-config
sudo apt install swig
sudo apt-get install python3-pip
```

## 5. Construir el paquete
```bash
cd YDLidar-SDK
cmake .
make 
sudo make install
```

Empaquetar el proyecto:
```bash
sudo cpack
```

## 6. Clonar el repositorio adaptado

```bash
mkdir -p ~/ydlidar_ros2_driver-master_ws/src
cd ~/ydlidar_ros2_driver-master_ws/src
git clone https://github.com/Oyefusi-Samuel/ydlidar_ros2_driver-master.git
```

## 6. Compilar el paquete
```bash
cd ~/ydlidar_ros2_driver-master_ws
source /opt/ros/humble/setup.bash 
colcon build --symlink-install
```

## 7. Modificar parámetros
Modificar `baudrate` a `128000` en los parámetros:
```bash
nano ~/ydlidar_ros2_driver-master_ws/src/ydlidar_ros2_driver-master/params/ydlidar_4ros.yaml
```

## 8. Ejecutar el lidar y visualizar nube de puntos
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ydlidar_ros2_driver ydlidar_4ros_view_launch.py
```

## -
## -
## -

# Python API
## 1. Instalación

## 2. Configuración del YDLIDAR X4

Ver los repositorios de YDLidar para más información sobre los parámetros y su configuración. En caso del X4, los valores son los siguiente:
```yaml
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0
    frame_id: laser_frame
    ignore_array: ""
    baudrate: 128000
    lidar_type: 1
    device_type: 0
    sample_rate: 5
    abnormal_check_count: 4
    fixed_resolution: true
    reversion: true
    inverted: true
    auto_reconnect: true
    isSingleChannel: false
    intensity: false
    support_motor_dtr: true
    angle_max: 180.0
    angle_min: -180.0
    range_max: 12.0
    range_min: 0.1
    frequency: 10.0
    invalid_range_is_inf: false
```

Para ejecutar los scripts de test de ``/YDLidar-SDK/python/examples``, hay que configurar correctamente los parámetros de nuestro modelo del lidar:

```python
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 6.0)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
```

Otra modificación que he tenido que hacer para evitar la división por cero:
```python
if r:
    if scan.config.scan_time < 1.0:
        timeNotZero = 1.0
    else:
        timeNotZero = scan.config.scan_time
    print("Scan received[",scan.stamp,"]:",scan.points.size(),"ranges is [",1.0/timeNotZero,"]Hz");
                
```
