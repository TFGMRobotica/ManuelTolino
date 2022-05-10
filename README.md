# Entorno de investigación de sistemas de robótica aérea
## Requisitos generales
El entoro de desarrollo de este repositorio se basa en la plataforma [Dronecode](https://www.dronecode.org/). 
El desarrollo principalmente se trabaja sobre el stack del autopiloto PX4. Es necesario disponer de todo el 'Toolchain' de desarrollo de la plataforma para poder trabajar con este repositorio. Se encuentra en su carpeta como submódulo la última versión a fecha de 22/03/2022 del stack autopilot PX4, con modificaciones propias. Para poder desarrollar y utilizar este repositorio, se deben seguir las instrucciones proporcionadas en la web de desarrollo de PX4. Se puede trabajar con Ubuntu 18 o Ubuntu 20. Como las versiones de ROS dependen según la versión de Ubuntu. En un primer lugar sólo se va a trabajar con Ubuntu 20. Si hay alguna funcionalidad requerida por la versión anterior, se creará una documentación a parte partiendo desde cero.
*El desarrollo se está realizando sobre Ubuntu 20.4 actualmente*
1. Instalar ROS Noetic
2. Seguir los pasos para la instalación de PX4: 
[Developer Toolchain - Ubuntu 20](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html#rosgazebo)
[Developer Toolchain - Ubuntu 18 y ROS Melodic sólo](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html#rosgazebo) 
3. Instalar OpenCV 4+
4. Comprobar el funcionamiento correcto de Gazebo
Abrir una instancia de QGroundControl y dejar en el background.
Lanzar el simulador junto el autopiloto:
```
cd PX4-Autopilot
make px4_sitl gazebo
```

## Simulación de control autónomo ([Offboard](https://docs.px4.io/master/en/flight_modes/offboard.html)) en PX4
```
Lanzar script de python offboardscript.py
```
## Simulación de detección de marcadores [ArUco](https://www.uco.es/investiga/grupos/ava/node/26)
# Programa de prueba con simulación en Gazebo y Quadrotor IRIS
Para realizar una prueba de funcionamiento con Gazebo, PX4, y la detección de marcadores ArUco con OpenCV 4 utilizar el nodo **roscpptestonly**
1. Lanzar QGroundControl en el background
2. Lanzar PX4 en SITL Gazebo con:
```
cd PX4-Autopilot
make px4_sitl gazebo
```
(El Gazebo World por defecto contiene el Iris con una cámara frontal simulada y un marcador ArUco para hacer la prueba, si no es así, pruebe a añadir un marcador ArUco a mano, y utilice la simulación del Typhoon con *make px4_sitl gazebo_typhoon_h480* pues este ya incluye una cámara simulada por defecto)
3. Lanzar el nodo *roscpptestonly*
```
source /opt/ros/noetic/setup.bash
cd Noetic_ws
catkin_make
source devel/setup.bash
rosrun roscpptestonly roscpptestonly_node
```
4. Debe aparecer una ventana con el input de vídeo desde el drone de Gazebo. Y si el marcador está enfocado, deben aparecer superpuestos sus contornos de detección.
Nota: Ruta por defecto de los headers de OpenCV 4 - para VSCode por ejemplo:
```
/usr/include/opencv4/
```

## Chuleta Github:

Ver branch y estado:
```
git status
```
Seleccionar rama (en los submódulos debería funcionar actualizándolo a la última rama de su repo original)
```
git checkout [e.g master]
```
Para clonar el repo y los submódulos correctamente:
```
git clone --recurse-submodules [LINK DE GITHUB]
```
Para actualizar submódulo a la última versión que se encuentre en su repo original:
```
git submodule update --remote (op. nombrar ruta y/o carpeta)
```
Para añadir un nuevo submódulo al repo:
```
git submodule add [LINK DE GITHUB] [Ruta y carpeta final renombrante]
```
Link recomendado: https://chrisjean.com/git-submodules-adding-using-removing-and-updating/

# ROS 2 + RTPS + PX4
Para compilar el workspace del puente ROS 2 - PX4:
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELWITHDEBINFO --symlink-install --packages-skip ros1_bridge
```
Pero una vez compilado por primera vez, se puede omitir el paquete px4_msgs:
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELWITHDEBINFO --symlink-install --packages-skip px4_msgs ros1_bridge
```
Tras instalar ROS 2, puede aparecer una advertencia sobre el contexto DDS. Instalar el siguiente paquete si es necesario:
```
sudo apt-get install ros-foxy-rmw-connext-cpp
```

Para comprobar el funcionamiento correcto del puente de comunicaciones entre PX4 y ROS2:
Navegar a la carpeta del repositorio de PX4 y compilar la versión de simulador que incluye el cliente RTPS:
```
cd PX4-Autopilot
make px4_sitl_rtps gazebo
```
En otra terminal, ejecutamos la aplicación de prueba "listerner" que escucha mensajes del topic **sensor_combined** y imprime los valores en pantalla:
```
source ~/px4_ros_com_ros2/install/setup.bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```
Y ahora solo falta activar la instancia del puente PX4-ROS 2 mediante RTPS, el 'daemon':
```
source ~/px4_ros_com_ros2/install/setup.bash
micrortps_agent -t UDP
```
Al activar este programa, se abre el canal de comunicación, y la aplicación "listerner" debería comenzar a recibir los mensajes desde PX4 y mostrarlos en pantalla.

Vamos a probar ahora a enviar a PX4 mensajes desde ROS 2.

Se inicia el simulador con capacidad RTPS como antes:
```
cd PX4-Autopilot
make px4_sitl_rtps gazebo
```
Ejecutamos el nodo de ejemplo
```
ros2 run px4_ros_com debug_vect_advertiser
```
Este nodo enviará un vector de prueba a PX4 con los valores 0, 3 y 4. Una vez lo ejecutamos, podemos abrir QGroundControl