# Simulación de datos de una MPU6050 en Rviz usando una ESP32

Este trabajo consiste en transmitir datos de un MPU6050 usando una ESP32 usando microROS para luego simular los datos recibidos en Rviz. Para esto se creó un nodo que publica en ```/imu/raw_data``` en la ESP32 que armara el mensaje en estandar de sensor Imu para ROS2 que luego es transmitido por puerto serial a través de un

Todos los archivos personalizados se encuentran comentados para el entendimiento del usuario.
Este trabajo fue realizado en ROS2 Iron.

![a](images/apariencia_3dof.png)
## Setup
Para lograr ejecutar el programa se deben cambiar algunas dirección de archivos con respecto donde se ubique el repositorio. 

## Simulación en Rviz
Para simularlo en Rviz se debe ubicar en la carpeta donde se haya clonado el repositorio. Luego se ejecutan los siguientes comandos:

Inicializar ROS.

``source /opt/ros/iron/setup.bash``

Se instala el paquete 3dof.

``colcon build``

``source install/setup.bash``

Se inicia la simulación en Rviz.

``ros2 launch 3dof visualization.launch.py``

El video de demostración se encuentra en [este archivo descargable](images/video/Rviz.webm)
![a](images/demostracion_Rviz.png)
