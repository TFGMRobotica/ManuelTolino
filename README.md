# Entorno de investigación de sistemas de robótica aérea
## Requisitos generales
El entoro de desarrollo de este repositorio se basa en la plataforma [Dronecode](https://www.dronecode.org/). En concreto el desarrollo principalmente es sobre el stack del autopiloto PX4. Es necesario disponer de todo el 'Toolchain' de desarrollo de la plataforma para poder trabajar con este repositorio. Se encuentra en su carpeta como submódulo la última versión a fecha de 22/03/2022 del stack autopilot PX4. Para poder desarrollar y utilizar este repositorio, se deben seguir las instrucciones proporcionadas en la web de desarrollo de PX4. Se puede trabajar con Ubuntu 18 o Ubuntu 20. Como las versiones de ROS dependen según la versión de Ubuntu. En un primer lugar sólo se va a trabajar con Ubuntu 20. Si hay alguna funcionalidad requerida por la versión anterior, se creará una documentación a parte partiendo desde cero.

1. Instalar ROS Noetic
2. Seguir los pasos para la instalación de la [Developer Toolchain](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html#rosgazebo) de PX4. 

Abrir una instancia de QGroundControl y dejar en el background.
Lanzar el simulador junto el autopiloto:
```
make px4_sitl gazebo
```
```
source /opt/ros/noetic/setup.bash
cd Noetic_ws
catkin_make
source devel/setup.bash
rosrun roscpptestonly roscpptestonly_node
```
```
Lanzar script de python offboardscript.py
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
git submodule update --remote
```
Para añadir un nuevo submódulo al repo:
```
git submodule add [LINK DE GITHUB] [Ruta y carpeta final renombrante]
```
Link recomendado: https://chrisjean.com/git-submodules-adding-using-removing-and-updating/