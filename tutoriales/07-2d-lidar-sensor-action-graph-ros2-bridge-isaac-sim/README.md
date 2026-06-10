# Tutorial 07: 2D LiDAR Sensor in Isaac Sim - Action Graph + ROS2 Bridge

Fuente oficial de referencia:

- [RTX Lidar Sensors, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html).
- [RTX Lidar Sensor, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_rtx_lidar.html).
- [ROS2 RTX Lidar Helper, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.ros2.bridge/docs/ogn/OgnROS2RtxLidarHelper.html).

## Objetivo

El objetivo de esta parte es montar un LiDAR RTX 2D sobre Carter v1 y publicar sus datos hacia ROS2 como `sensor_msgs/msg/LaserScan`.

El flujo de trabajo:

- Isaac Sim crea un sensor `RTX Lidar` de tipo 2D.
- El LiDAR se monta como hijo del chasis para que se mueva junto con Carter.
- `Isaac Create Render Product` crea el render product del sensor.
- `ROS2 RTX Lidar Helper` publica el escaneo 2D en ROS2.
- ROS2 permite visualizar el escaneo en RViz2 como `LaserScan`.

## Parte 1: Preparar la escena

1. Abre Isaac Sim.
2. Crea o abre una escena con Carter v1.
3. Revisa que Carter quede estable al presionar `Play`.
4. Agrega algunos objetos frente al robot para que el LiDAR tenga superficies que detectar.

La ruta de referencia del robot dentro de los assets de Isaac Sim es:

```text
Robots/NVIDIA/Carter/carter_v1.usd
```

Para este tutorial se usara esta jerarquia de ejemplo:

```text
/World/Carter/chassis_link
```

Si tu asset usa `Chassis_link` u otro nombre, usa el nombre exacto que aparezca en el `Stage`.

## Parte 2: Crear y montar el LiDAR 2D

Crea el sensor desde el menu:

```text
Create > Sensors > RTX Lidar > NVIDIA > Example Rotary 2D
```

Renombra el prim como:

```text
Lidar_2D
```

Arrastra `Lidar_2D` dentro del chasis de Carter o dentro de un mount del chasis.

La jerarquia debe quedar parecida a:

```text
/World/Carter/chassis_link/lidar_2d_mount/Lidar_2D
```

Si no existe `lidar_2d_mount`, puedes dejar el sensor directamente dentro de:

```text
/World/Carter/chassis_link/Lidar_2D
```

Configura una pose local inicial:

| Campo | Valor |
| --- | --- |
| `Translate X` | `0.2` |
| `Translate Y` | `0.0` |
| `Translate Z` | `0.35` |
| `Rotate X` | `0.0` |
| `Rotate Y` | `0.0` |
| `Rotate Z` | `0.0` |

Presiona `Play` y confirma que el LiDAR se desplaza junto con Carter. Si el sensor se queda quieto, quedo fuera del prim del robot.

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
- `Isaac Create Render Product`
- `ROS2 RTX Lidar Helper`

Conecta las senales asi:

| Salida | Entrada |
| --- | --- |
| `On Playback Tick.tick` | `Isaac Create Render Product.execIn` |
| `Isaac Create Render Product.execOut` | `ROS2 RTX Lidar Helper.execIn` |
| `Isaac Create Render Product.renderProductPath` | `ROS2 RTX Lidar Helper.renderProductPath` |
| `ROS2 Context.context` | `ROS2 RTX Lidar Helper.context` |

El flujo completo queda:

```text
On Playback Tick
  -> Isaac Create Render Product
  -> ROS2 RTX Lidar Helper
```

## Parte 4: Configurar Isaac Create Render Product

Selecciona el nodo `Isaac Create Render Product` y configura:

| Campo | Valor |
| --- | --- |
| `cameraPrim` | `/World/Carter/chassis_link/lidar_2d_mount/Lidar_2D` |
| `enabled` | `True` |

Si tu sensor quedo en otra ruta, usa esa ruta exacta. Aunque el campo se llame `cameraPrim`, tambien se usa para sensores RTX como LiDAR.

## Parte 5: Configurar ROS2 RTX Lidar Helper

Selecciona el nodo `ROS2 RTX Lidar Helper` y configura:

| Campo | Valor |
| --- | --- |
| `type` | `laser_scan` |
| `topicName` | `scan` |
| `nodeNamespace` | `carter/lidar_2d` |
| `frameId` | `carter_lidar_2d_link` |
| `queueSize` | `10` |
| `frameSkipCount` | `0` |
| `enabled` | `True` |

Con esta configuracion, el topico queda:

```text
/carter/lidar_2d/scan
```

El tipo de mensaje es:

```text
sensor_msgs/msg/LaserScan
```

Cuando `type = laser_scan`, el mensaje se publica cuando el LiDAR completa un escaneo. En un sensor rotatorio, eso puede tardar varios frames segun la velocidad de rotacion y el timestep de simulacion.

## Parte 6: Probar desde ROS2

Con la simulacion en `Play`, abre una terminal con ROS2 cargado y revisa:

```bash
ros2 topic list
```

Debe aparecer:

```text
/carter/lidar_2d/scan
```

Revisa el tipo:

```bash
ros2 topic info /carter/lidar_2d/scan
```

Revisa la frecuencia:

```bash
ros2 topic hz /carter/lidar_2d/scan
```

Imprime un mensaje:

```bash
ros2 topic echo /carter/lidar_2d/scan --once
```

Mueve Carter o coloca objetos alrededor y revisa que cambien los rangos del `LaserScan`.

## Parte 7: Visualizar en RViz2

Abre RViz2:

```bash
rviz2
```

En RViz2:

1. En `Fixed Frame`, prueba primero con `carter_lidar_2d_link`.
2. En `Displays`, oprime `Add`.
3. Selecciona `By topic`.
4. Agrega `/carter/lidar_2d/scan` como `LaserScan`.

Si ya tienes TF publicado para Carter, puedes usar `base_link` u `odom` como `Fixed Frame`. Si todavia no tienes TF, usar el frame del LiDAR permite validar el topico sin configurar el arbol completo de transforms.

## Errores comunes

- No aparece `/carter/lidar_2d/scan`: revisa que ROS2 Bridge este habilitado y que la simulacion este en `Play`.
- El topico aparece pero no publica: revisa que `Isaac Create Render Product` apunte al prim del LiDAR.
- RViz2 no muestra el escaneo: revisa `Fixed Frame` y usa `carter_lidar_2d_link` para una primera prueba.
- El LiDAR no sigue al robot: revisa que `Lidar_2D` sea hijo de `chassis_link`.
- El escaneo sale vacio: agrega objetos dentro del rango del LiDAR y revisa que el sensor no este dentro de la geometria del robot.
- La frecuencia es baja: recuerda que `LaserScan` se publica cuando se completa un escaneo completo.
