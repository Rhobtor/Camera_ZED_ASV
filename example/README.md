Para subscribirse a un topic de la camara necesitamos un quality of service(QOS)
example:
depth_qos=rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST
            )

Si modificamos algun parametro de la camara , modificamos el QOS. Si no son iguales se recibe nada del toopic susbcrito.

La matriz se ha calculado desde el punto central de la imagen obtenido con la magen deapth y la localizacion del punto central. El centro de coordenadas de la camara al obtener una imagen se encuantra en la esquina superior IZQ, siendo Y hacia arriba y X hacia la IZQ.
El nuevo sistema de coordenada sigue el sistema de referencia utlizado en ROS, Z arriba, X hacia adelante e Y hacia la IZQ (mano derecha). Con esto ademas coincide con el sistema empleado en el wrapper para la odometria y posicion de la camara que emple este sistema de referencia. 