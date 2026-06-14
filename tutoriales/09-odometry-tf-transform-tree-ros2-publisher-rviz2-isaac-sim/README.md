# Tutorial 09: Odometry & TF in Isaac Sim - ROS2 TF Publisher & RViz2

Fuente oficial de referencia:

- [ROS2 Transform Trees and Odometry, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_tf.html).
- [Isaac Compute Odometry Node, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.nodes/docs/ogn/OgnIsaacComputeOdometry.html).
- [ROS2 Publish Odometry, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.ros2.bridge/docs/ogn/OgnROS2PublishOdometry.html).
- [ROS2 Publish Transform Tree, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.ros2.bridge/docs/ogn/OgnROS2PublishTransformTree.html).
- [ROS2 Publish Raw Transform Tree, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.ros2.bridge/docs/ogn/OgnROS2PublishRawTransformTree.html).

## Objetivo

El objetivo de esta parte es publicar odometria y un arbol TF para Carter v1 desde Isaac Sim hacia ROS2, y visualizarlo en RViz2.

El flujo de trabajo:

- `Isaac Compute Odometry` calcula la pose de Carter respecto a su posicion inicial.
- `ROS2 Publish Odometry` publica `nav_msgs/msg/Odometry`.
- `ROS2 Publish Raw Transform Tree` publica el transform dinamico `odom -> base_link`.
- `ROS2 Publish Transform Tree` publica transforms de links y sensores.
- RViz2 usa `/tf`, `/tf_static` y `/carter/odom` para visualizar el movimiento.

## Parte 1: Preparar la escena

1. Abre Isaac Sim.
2. Crea o abre una escena con Carter v1.
3. Verifica que Carter tenga `Articulation Root`.
4. Confirma que el robot se mueve con el control de los tutoriales anteriores.
5. Si ya agregaste camara, IMU y LiDAR, deja esos sensores montados en el chasis.

La ruta de referencia del robot dentro de los assets de Isaac Sim es:

```text
Robots/NVIDIA/Carter/carter_v1.usd
```

Para este tutorial se usaran estas rutas de ejemplo:

```text
/World/Carter
/World/Carter/chassis_link
```

Si tu asset usa `Chassis_link` u otro nombre, usa el nombre exacto que aparezca en el `Stage`.

## Parte 2: Definir frames ROS2

Usaremos estos frames:

| Frame | Uso |
| --- | --- |
| `world` | Frame global opcional de la escena |
| `odom` | Frame de odometria |
| `base_link` | Frame principal del robot |
| `carter_front_camera_link` | Frame de la camara |
| `carter_imu_link` | Frame del IMU |
| `carter_lidar_2d_link` | Frame del LiDAR 2D |
| `carter_lidar_3d_link` | Frame del LiDAR 3D |

La relacion principal para navegacion queda:

```text
odom -> base_link -> sensores
```

La odometria no reemplaza localizacion global. En un robot real, paquetes como SLAM, AMCL o filtros de estado suelen publicar `map -> odom`.

## Parte 3: Crear el Action Graph

Abre:

```text
Window > Graph Editors > Action Graph
```

vamos a  usar los asistentes:

```text
Tools > Robotics > ROS 2 OmniGraphs > Odometry Publisher
``
Seleccionamos el articulation root
y en chasis link Prim seleccionamos /base_link


## Parte 8: Probar desde ROS2

Con la simulacion en `Play`, abre una terminal con ROS2 cargado y revisa:

```bash
ros2 topic list
```

Deben aparecer:

```text
/tf
/odom
```

Revisa la odometria:

```bash
ros2 topic info /odom
ros2 topic echo /odom --once
```

Revisa TF:

```bash
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
```

Genera un reporte de frames:

```bash
ros2 run tf2_tools view_frames
```

## Parte 9: Visualizar en RViz2

Abre RViz2:

```bash
rviz2
```

Configuracion recomendada:

1. En `Fixed Frame`, usa `base_link`.
2. Agrega un display `TF`.
3. Agrega un display `Odometry`.
4. En el display `Odometry`, selecciona `/odom`.


## Parte 10: Ver TF dentro de Isaac Sim

Isaac Sim tambien tiene un visualizador de TF.

Habilita la extension:

```text
isaacsim.ros2.tf_viewer
```

Luego abre:

```text
Window > TF Viewer
```

Con la simulacion en `Play`, selecciona el frame raiz, por ejemplo `world` u `odom`, y revisa que aparezcan los frames publicados.

