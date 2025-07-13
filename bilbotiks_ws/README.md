# bilbotiks_ws/

### 1. Activar ROS2. 
```bash
source /opt/ros/humble/setup.bash
```

### 2. Compilar y activar el workspace. 
```bash
colcon build
```
```bash
source install/setup.bash
```

### 3. Ejecutar prueba de percepción.
En una terminal:
```bash
ros2 launch bilbotiks_controller pertzepzioa.py
```

En otra terminal:
```bash
ros2 run bilbotiks_controller pertzepzio_proba
```

### 4. Ejecutar prueba de control.
```bash
ros2 launch bilbotiks_controller kontrola.py
```