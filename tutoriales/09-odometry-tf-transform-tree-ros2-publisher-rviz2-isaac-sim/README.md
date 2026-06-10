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

Tambien puedes usar los asistentes:

```text
Tools > Robotics > ROS 2 OmniGraphs > Odometry Publisher
Tools > Robotics > ROS 2 OmniGraphs > TF Publisher
```

Para hacerlo a mano, crea un Action Graph nuevo y agrega estos nodos:

- `On Playback Tick`
- `ROS2 Context`
- `Isaac Read Simulation Time`
- `Isaac Compute Odometry`
- `ROS2 Publish Odometry`
- `ROS2 Publish Raw Transform Tree`
- `ROS2 Publish Transform Tree`

## Parte 4: Conectar odometria

Conecta las senales asi:

| Salida | Entrada |
| --- | --- |
| `On Playback Tick.tick` | `Isaac Compute Odometry.execIn` |
| `Isaac Compute Odometry.execOut` | `ROS2 Publish Odometry.execIn` |
| `Isaac Compute Odometry.position` | `ROS2 Publish Odometry.position` |
| `Isaac Compute Odometry.orientation` | `ROS2 Publish Odometry.orientation` |
| `Isaac Compute Odometry.linearVelocity` | `ROS2 Publish Odometry.linearVelocity` |
| `Isaac Compute Odometry.angularVelocity` | `ROS2 Publish Odometry.angularVelocity` |
| `Isaac Read Simulation Time.simulationTime` | `ROS2 Publish Odometry.timeStamp` |
| `ROS2 Context.context` | `ROS2 Publish Odometry.context` |

Configura `Isaac Compute Odometry`:

| Campo | Valor |
| --- | --- |
| `chassisPrim` | `/World/Carter` |

Configura `ROS2 Publish Odometry`:

| Campo | Valor |
| --- | --- |
| `topicName` | `odom` |
| `nodeNamespace` | `carter` |
| `odomFrameId` | `odom` |
| `chassisFrameId` | `base_link` |
| `robotFront` | `1.0, 0.0, 0.0` |
| `queueSize` | `10` |

Con esta configuracion, el topico queda:

```text
/carter/odom
```

El tipo de mensaje es:

```text
nav_msgs/msg/Odometry
```

## Parte 5: Publicar odom -> base_link

Agrega y configura `ROS2 Publish Raw Transform Tree`.

Conecta:

| Salida | Entrada |
| --- | --- |
| `On Playback Tick.tick` | `ROS2 Publish Raw Transform Tree.execIn` |
| `Isaac Compute Odometry.position` | `ROS2 Publish Raw Transform Tree.translation` |
| `Isaac Compute Odometry.orientation` | `ROS2 Publish Raw Transform Tree.rotation` |
| `Isaac Read Simulation Time.simulationTime` | `ROS2 Publish Raw Transform Tree.timeStamp` |
| `ROS2 Context.context` | `ROS2 Publish Raw Transform Tree.context` |

Configura el nodo:

| Campo | Valor |
| --- | --- |
| `topicName` | `tf` |
| `parentFrameId` | `odom` |
| `childFrameId` | `base_link` |
| `queueSize` | `10` |

No agregues `nodeNamespace` a los publishers de TF si quieres publicar en el topico estandar:

```text
/tf
```

## Parte 6: Publicar transforms de sensores

Agrega un nodo `ROS2 Publish Transform Tree`.

Conecta:

| Salida | Entrada |
| --- | --- |
| `On Playback Tick.tick` | `ROS2 Publish Transform Tree.execIn` |
| `Isaac Read Simulation Time.simulationTime` | `ROS2 Publish Transform Tree.timeStamp` |
| `ROS2 Context.context` | `ROS2 Publish Transform Tree.context` |

Configura:

| Campo | Valor |
| --- | --- |
| `topicName` | `tf` |
| `parentPrim` | `/World/Carter/chassis_link` |
| `targetPrims` | sensores montados en el chasis |
| `staticPublisher` | `True` |
| `queueSize` | `10` |

Ejemplos de `targetPrims`:

```text
/World/Carter/chassis_link/camera_mount/RGB_Sensor
/World/Carter/chassis_link/IMU_Sensor
/World/Carter/chassis_link/lidar_2d_mount/Lidar_2D
/World/Carter/chassis_link/lidar_3d_mount/Lidar_3D
```

Si alguno de esos sensores no existe en tu escena, no lo agregues al arreglo.

`staticPublisher = True` es adecuado para sensores fijos al chasis. Si el sensor se mueve con un joint o mecanismo, usa `False`.

## Parte 7: Publicar world -> odom opcional

Si quieres que RViz2 tenga un frame global `world`, agrega otro `ROS2 Publish Raw Transform Tree`.

Configura:

| Campo | Valor |
| --- | --- |
| `topicName` | `tf_static` |
| `parentFrameId` | `world` |
| `childFrameId` | `odom` |
| `translation` | `0.0, 0.0, 0.0` |
| `rotation` | `0.0, 0.0, 0.0, 1.0` |

Este transform representa que el frame `odom` inicia alineado con `world`.

## Parte 8: Probar desde ROS2

Con la simulacion en `Play`, abre una terminal con ROS2 cargado y revisa:

```bash
ros2 topic list
```

Deben aparecer:

```text
/tf
/tf_static
/carter/odom
```

Revisa la odometria:

```bash
ros2 topic info /carter/odom
ros2 topic echo /carter/odom --once
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

Esto genera un PDF con el arbol TF publicado. Instala la herramienta si no esta disponible:

```bash
sudo apt install ros-$ROS_DISTRO-tf2-tools
```

## Parte 9: Visualizar en RViz2

Abre RViz2:

```bash
rviz2
```

Configuracion recomendada:

1. En `Fixed Frame`, usa `odom`.
2. Agrega un display `TF`.
3. Agrega un display `Odometry`.
4. En el display `Odometry`, selecciona `/carter/odom`.
5. Si tienes LiDAR, agrega `/carter/lidar_2d/scan` o `/carter/lidar_3d/points`.

Mueve Carter y revisa:

- El frame `base_link` debe moverse bajo `odom`.
- Los frames de sensores deben mantenerse fijos respecto a `base_link`.
- La odometria debe cambiar mientras el robot se desplaza.

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

## Errores comunes

- `/carter/odom` no aparece: revisa que `ROS2 Publish Odometry` tenga `execIn`, `context` y `timeStamp`.
- `/tf` no aparece: revisa que `ROS2 Publish Raw Transform Tree` este conectado al tick.
- RViz2 no muestra frames: usa `odom` como `Fixed Frame` y revisa `ros2 topic echo /tf --once`.
- El arbol TF queda separado: revisa que `childFrameId = base_link` coincida con el frame usado por sensores.
- La odometria no cambia: revisa que `chassisPrim` apunte al prim correcto de Carter.
- Los sensores no aparecen bajo `base_link`: revisa `parentPrim` y `targetPrims` en `ROS2 Publish Transform Tree`.
- Aparecen nombres de frames inesperados: revisa los nombres de prims o usa atributos `NameOverride` si necesitas nombres ROS2 especificos.
