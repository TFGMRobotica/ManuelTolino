# Onboard computer RPi 4
## Prueba simple HITL
Para comprobar que el entorno de ROS 2 + OpenCV + microRTPS + PX4 msgs está instalado y programado correctamente, se propone una prueba en la cual:
1. El software PX4 (el autopiloto) se ejecutará en la simulación de un MAV IRIS en Gazebo, en un PC Linux Ubuntu 20.04 junto al microRTPS client.

```
make px4_sitl_rtps gazebo
```
Por defecto, el simulador con el cliente microRTPS estará configurado para transmitir en la dirección local (localhost) "127.0.0.1" según se puede localizar en el código fuente. Para poder conectarlo con otro sistema con otra IP en la misma red local, se debe parar y lanzar de nuevo mediante:
```
micrortps_client start -t UDP -i "ip"
```
En una primera prueba no ha funcionado, siendo necesario editar la IP en el código fuente. Esto puede hacerse fácilmente editando el archivo:
```
/PX4-Autopilot/src/modules/micrortps_bridge/micrortps_client/microRTPS_client.h
```
2. El MAV simulado publicará todos sus parámetros al entorno ROS2, y transmitirá la imágen de camara simulada en una 'pipeline' gstreamer mediante UDP.
Para lograr esto es importante configurar el módelo de cámara de gazebo correctamente. En primer lugar se puede añadir un nuevo emisor de imágen gstreamer editando el archivo:
```
/PX4-Autopilot/Tools/sitl_gazebo/models/fpv_cam/fpv_cam.sdf
```
Se tendría que añadir un parámetro adicional para especificar la dirección IP de la RPi 4:
```
<udpHost>"192.168.0.100"</udpHost>
```
Este archivo hace referencia a los scripts Gazebo donde se programa la pipeline gstreamer. Se puede editar el código fuente directamente de estos scripts en:
```
/PX4-Autopilot/Tools/sitl_gazebo/src
```

3. El PC con el simulador y la RPi se encontrarán en la misma red LAN.
4. En la RPi 4, se debe ejecutar el programa agente microRTPS:
```
$ ./micrortps_agent [options]
  -t <transport>          [UART|UDP] Default UART.
  -d <device>             UART device. Default /dev/ttyACM0.
  -w <sleep_time_us>      Time in us for which each iteration sleep. Default 1ms.
  -b <baudrate>           UART device baudrate. Default 460800.
  -p <poll_ms>            Time in ms to poll over UART. Default 1ms.
  -r <reception port>     UDP port for receiving. Default 2019.
  -s <sending port>       UDP port for sending. Default 2020.
  -n <set namespace>      Set a namespace for the micrortps_agent.
  ```
<span style="color:blue">⚠️ No se ha conseguido recibir transmisión de vídeo mediante gstreamer y openCV en la raspberry Pi 4B+. Esto puede ser debido a que OpenCV no ha sido compilado en la RPI4 con la opción activada de usar Gstreamer. Habría que probar recompilandolo pero este proceso lleva horas. Se va a tratar de usar el transporte de imágen mediante ROS 2 para esta prueba en su lugar.</span>
