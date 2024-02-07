Repositorio de la camara ZED2i empleado en los vehiculos autonomos de supreficie (ASV)

Hay dos dockerfiles:
- Docker file with ZED SDK and the api to use it : dockerfile_Zed_api
- Docker with Ros2 wrapper : dockerfile_Zed2i_Wrapper_Eloquent

Los archivos que estan en la carpeta zed-ros2-wrapper-eloquent son necesarios para el dockerfile wrapper , asi como del entropoint_jetson.

Se ha modificado para incluir un archivo launch para la camara Zed2i y una configuracion para la camara Zed2i

Antes de lanzar el docker realizar el siguiente comando para permitir al usuario usar el X server. 

xhost +si:localhost:root

Para lanzar el docker realizarlo con el comando siguiente

docker run -it --gpus all  --privileged --net host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix <image name>


Si se quiere persistir con un volumen externo podemos hacerlo con 

docker run -it --gpus all  --privileged --net host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v 
<path folder in hot>:<path folder destination> <image name>

En el wrapper lanzar el launch con el comando 

ros2 launch zed_wrapper zed2i.launch.py

Recordatorio abrir otro termianl con y cambiar con ros2 run el centro de la camara. Esto nos permite usar la camara desde el centro de la camara, situado a 6cm a la derecha de la lente ZIQ (Recordatorio la camara tiene como ejes Z foward, Y down and X rigth)

docker exec -it <image ps> /bin/bash

ros2 run tf2_ros static_transform_publisher 0 0.06 0 0 0 0 base_link zed2i_camera_center


Para lanzar el servicio de empezar a grabar con la camara utilizar el siguiente comando

ros2 service call /zed2i/zed_node/start_svo_rec zed_interfaces/srv/StartSvoRec "{svo_filename: '/root/<folder name>/<name_file>.svo', compression_mode: '2', target_framerate: '30', bitrate: '6000'}"

- Compresion_mode 0-2, 0: LOSSLESS (PNG/ZSTD), 1: H264 (AVCHD) ,2: H265 (HEVC)
- Target_framerate: 0,15,30, 60 or 100. Si empleamos HD720 limitado a 30 max
- Bitrate: 0 or [1000,60000]. 
- Transcode: true or flase . Transforma en otro formato de video ( Preferible no , debido a que la camaara trabaja con el formato svo, se puede convertir luego con la api a otro formatos)

Para pausar la grabacion se puede llamar al servicio con el siguiente comando

ros2 service call /zed2i/zed_node/stop_svo_rec std_srvs/srv/Triggers 

Recordatorio la camara tiene como ejes Z foward, Y down and X rigth sobre la lente IZQ cambiar mediante una matriz hacia el centro de la camara
Los procesos, asi como ejemplo en la carpeta examples

