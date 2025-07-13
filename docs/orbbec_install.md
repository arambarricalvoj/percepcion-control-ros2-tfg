# Instalación de `orbbec/ros2_astra_camera`

Repositorio: https://github.com/orbbec/ros2_astra_camera

Instrucciones para instalar el paquete `ros2_astra_camera` en `ROS2 Humble`.

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


## 3. Instalar dependencias y librerías
```bash
sudo apt install libgflags-dev
sudo apt install ros-$ROS_DISTRO-image-geometry
sudo apt install ros-$ROS_DISTRO-camera-info-manager
sudo apt install ros-$ROS_DISTRO-image-transport
sudo apt install ros-$ROS_DISTRO-image-publisher
sudo apt install libgoogle-glog-dev 
sudo apt install libusb-1.0-0-dev 
sudo apt install libeigen3-dev
sudo apt install nlohmann-json3-dev
```

Instalar `libuvc`:
```bash
cd ~
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig # Refreshing the link library
```

## 4. Crear el workspace
Crear el workspace (omitir si ya existe):
```bash
mkdir -p ~/ros2_ws/src
```

Ir a `~/ros2_ws/src`:
```bash
cd ~/ros2_ws/src
```

## 5. Clonar el repositorio

```bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/ros2_astra_camera.git
```

Instalar las reglas de `libusb`:
```bash
cd ~/ros2_ws/src/ros2_astra_camera/astra_camera/scripts
sudo bash install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 6. Compilar el paquete
```bash
 cd ~/ros2_ws
source /opt/ros/humble/setup.bash 
colcon build --event-handlers  console_direct+  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 7. Ejecutar la cámara
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch astra_camera astro_pro_plus.launch.xml
```

# Posibles ERRORES y SOLUCIONES

## Unable to lacete package
- `E: Unable to locate package ros-humble-camera-info-manager`
- `E: Unable to locate package ros-humble-image-publisher`

El error viene de `sudo apt-get update`que genera el siguiente output:
```bash
Hit:1 http://security.ubuntu.com/ubuntu jammy-security InRelease
Hit:2 http://es.archive.ubuntu.com/ubuntu jammy InRelease               
Ign:3 https://packages.ros.org/ros/ubuntu jammy InRelease               
Hit:4 http://es.archive.ubuntu.com/ubuntu jammy-updates InRelease       
Hit:5 http://es.archive.ubuntu.com/ubuntu jammy-backports InRelease
Ign:3 https://packages.ros.org/ros/ubuntu jammy InRelease
Ign:3 https://packages.ros.org/ros/ubuntu jammy InRelease
Err:3 https://packages.ros.org/ros/ubuntu jammy InRelease
  
Certificate verification failed: The certificate is NOT trusted. The name in the certificate does not match the expected.  Could not handshake: Error in the certificate verification. [IP: 64.50.233.100 443]


Reading package lists... Done
W: Failed to fetch https://packages.ros.org/ros/ubuntu/dists/jammy/InRelease  Certificate verification failed: The certificate is NOT trusted. The name in the certificate does not match the expected.  Could not handshake: Error in the certificate verification. [IP: 64.50.233.100 443]

W: Some index files failed to download. They have been ignored, or old ones used instead.
```

Se ha solucionado desinstalando ROS2 Humble (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#uninstall) y volviéndolo a instalar (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#uninstall).

## Error al compilar el workspace
```bash
/home/javierac/ros2_ws/src/ros2_astra_camera/astra_camera/src/ob_camera_info.cpp:14:10: fatal error: nlohmann/json.hpp: No such file or directory
   14 | #include <nlohmann/json.hpp>
      |          ^~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/astra_camera.dir/build.make:132: CMakeFiles/astra_camera.dir/src/ob_camera_info.cpp.o] Error 1
gmake[2]: *** Waiting for unfinished jobs....
[Processing: astra_camera]                             
gmake[1]: *** [CMakeFiles/Makefile2:143: CMakeFiles/astra_camera.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
--- stderr: astra_camera
/home/javierac/ros2_ws/src/ros2_astra_camera/astra_camera/src/ob_camera_info.cpp:14:10: fatal error: nlohmann/json.hpp: No such file or directory
   14 | #include <nlohmann/json.hpp>
      |          ^~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/astra_camera.dir/build.make:132: CMakeFiles/astra_camera.dir/src/ob_camera_info.cpp.o] Error 1
gmake[2]: *** Waiting for unfinished jobs....
gmake[1]: *** [CMakeFiles/Makefile2:143: CMakeFiles/astra_camera.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< astra_camera [49.1s, exited with code 2]

Summary: 1 package finished [1min 3s]
  1 package failed: astra_camera
  1 package had stderr output: astra_camera
```

Solución: falta la dependencia `nlohmann-json3-dev`:
```bash
sudo apt install nlohmann-json3-dev
```
## -
## -
## -
