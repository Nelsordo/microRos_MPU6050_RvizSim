# 3dof
3dof es un brazo robótico articulado con 3 grados de libertad con una cámara adjuntada en el extremo del brazo. En este repositorio se mostrará la simulación de este brazo en Rviz, con ventana de controles, y en Gazebo, siendo controlado por comandos.
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
