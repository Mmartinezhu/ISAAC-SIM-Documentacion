# Tutorial 08: 3D LiDAR Sensor in Isaac Sim - Action Graph + ROS2 Bridge

Fuente oficial de referencia:

- [RTX Lidar Sensors, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html).
- [RTX Lidar Sensor, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_rtx_lidar.html).
- [ROS2 RTX Lidar Helper, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.ros2.bridge/docs/ogn/OgnROS2RtxLidarHelper.html).

## Objetivo

El objetivo de esta parte es montar un LiDAR RTX 3D sobre Carter v1 y publicar sus datos hacia ROS2 como `sensor_msgs/msg/PointCloud2`.

El flujo de trabajo:

- Isaac Sim crea un sensor `RTX Lidar` de tipo 3D.
- El sensor se monta sobre el chasis de Carter.
- `Isaac Create Render Product` crea el render product del LiDAR.
- `ROS2 RTX Lidar Helper` publica la nube de puntos hacia ROS2.
- RViz2 visualiza el resultado como `PointCloud2`.

## Parte 1: Preparar la escena

1. Abre Isaac Sim.
2. Crea o abre una escena con Carter v1.
3. Agrega objetos alrededor del robot para generar puntos visibles.
4. Presiona `Play` brevemente para confirmar que el robot y la escena son estables.

La ruta de referencia del robot dentro de los assets de Isaac Sim es:

```text
Robots/NVIDIA/Carter/carter_v1.usd
```

Para este tutorial se usara esta jerarquia de ejemplo:

```text
/World/Carter/chassis_link/camera_mount
```

## Parte 2: Crear y montar el LiDAR 3D

Crea el sensor desde el menu:

```text
Create > Sensors > RTX Lidar > NVIDIA > Example Rotary
```

Renombra el prim como:

```text
Lidar_3D
```

Arrastra `Lidar_3D` dentro del chasis de Carter o dentro de un mount del chasis.

La jerarquia debe quedar parecida a:

```text
/World/Carter/chassis_link/camera_mount/Lidar_3D
```

Si no existe `lidar_3d_mount`, puedes dejar el sensor directamente dentro de:

```text
/World/Carter/chassis_link/Lidar_3D
```

Configura una pose local inicial:

| Campo | Valor |
| --- | --- |
| `Translate X` | `0.0` |
| `Translate Y` | `0.0` |
| `Translate Z` | `0.45` |
| `Rotate X` | `0.0` |
| `Rotate Y` | `0.0` |
| `Rotate Z` | `0.0` |

El LiDAR 3D debe quedar lo suficientemente alto para no quedar dentro de la geometria del robot.

## Parte 3: Crear el Action Graph

Abre:

```text
Window > Graph Editors > Action Graph
```

Tambien puedes usar el asistente:

```text
Tools > Robotics > ROS 2 OmniGraphs > RTX Lidar
```

Para hacerlo a mano, crea un Action Graph nuevo y agrega estos nodos:

- `On Playback Tick`
- `ROS2 Context`
- `Isaac Run One Simulation Frame`
- `Isaac Create Render Product`
- `ROS2 RTX Lidar Helper`

Conecta las senales asi:

| Salida | Entrada |
| --- | --- |
| `On Playback Tick.tick` | `Isaac Run One Simulation Frame.execIn` |
| `Isaac Run One Simulation Frame.stepOut` | `Isaac Create Render Product.execIn` |
| `Isaac Create Render Product.execOut` | `ROS2 RTX Lidar Helper.execIn` |
| `Isaac Create Render Product.renderProductPath` | `ROS2 RTX Lidar Helper.renderProductPath` |
| `ROS2 Context.context` | `ROS2 RTX Lidar Helper.context` |

El flujo completo queda:

```text
On Playback Tick
  -> Isaac Create Render Product
  -> ROS2 RTX Lidar Helper
```
El action Graph queda algo asi: 
<img width="1570" height="574" alt="image" src="https://github.com/user-attachments/assets/455402dd-9624-434c-af75-e90fdb7e0c7c" />


## Parte 4: Configurar Isaac Create Render Product

Selecciona el nodo `Isaac Create Render Product` y configura:

| Campo | Valor |
| --- | --- |
| `cameraPrim` | `/World/Carter/chassis_link/camera_mount/Lidar_3D` |
| `enabled` | `True` |

Si tu sensor quedo en otra ruta, usa esa ruta exacta. El campo `cameraPrim` tambien acepta sensores RTX LiDAR.

## Parte 5: Configurar ROS2 RTX Lidar Helper

Selecciona el nodo `ROS2 RTX Lidar Helper` y configura:

| Campo | Valor |
| --- | --- |
| `type` | `point_cloud` |
| `topicName` | `point_cloud` |
| `nodeNamespace` | `carter/lidar_3d` |
| `frameId` | `sim_lidar` |
| `queueSize` | `10` |
| `frameSkipCount` | `0` |
| `fullScan` | `False` |
| `enabled` | `True` |

Con esta configuracion, el topico queda:

```text
/point_cloud
```

El tipo de mensaje es:

```text
sensor_msgs/msg/PointCloud2
```

Notas:

- `fullScan = False` publica puntos parciales con menor latencia.
- `fullScan = True` espera a acumular un escaneo completo antes de publicar.


## Parte 6: Probar desde ROS2

Con la simulacion en `Play`, abre una terminal con ROS2 cargado y revisa:

```bash
ros2 topic list
```

Debe aparecer:

```text
/point_cloud
```

Revisa el tipo:

```bash
ros2 topic info /point_cloud
```

Revisa la frecuencia:

```bash
ros2 topic hz /point_cloud
```

Imprime un mensaje para confirmar que hay datos:

```bash
ros2 topic echo /point_cloud --once
```

## Parte 7: Visualizar en RViz2

Abre RViz2:

```bash
rviz2
```

En RViz2:

1. En `Fixed Frame`, prueba primero con `sim_lidar`.
2. En `Displays`, oprime `Add`.
3. Selecciona `By topic`.
4. Agrega `/point_cloud` como `PointCloud2`.
5. Ajusta `Size (m)` a 0.1  si los puntos se ven muy pequenos.

Si ya tienes TF publicado para Carter, puedes usar `base_link` u `odom` como `Fixed Frame`.

