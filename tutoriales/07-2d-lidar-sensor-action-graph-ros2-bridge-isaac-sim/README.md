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

1. Abre Isaac Sim con:

   ```bash
   talos@IsaacUN:~/isaac-sim$ ./launch_isaacsim.bash
   ```

2. Crea o abre una escena con Carter v1.
3. Revisa que Carter quede estable al presionar `Play`.
4. Agrega algunos objetos frente al robot para que el LiDAR tenga superficies que detectar.

La ruta de referencia del robot dentro de los assets de Isaac Sim es:

```text
Robots/NVIDIA/Carter/carter_v1.usd
```

Para este tutorial se usara esta jerarquia de ejemplo:

```text
/World/Carter/chassis_link/camera_mount/XT_32_10hz 
```

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
/World/Carter/chassis_link/camera_mount/XT_32_10hz /Lidar_2D
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
- `Isaac Read Simulation Time`
- `Isaac Read Lidar Beams Node`
- `ROS2 Publish Laser Scan`

Conecta las senales asi:

| Salida | Entrada |
| --- | --- |
| Origen                                             | Destino                                        |
| -------------------------------------------------- | ---------------------------------------------- |
| `On Playback Tick.tick`                            | `Isaac Read Lidar Beams Node.execIn`           |
| `Isaac Read Lidar Beams Node.execOut`              | `ROS2 Publish Laser Scan.execIn`               |
| `ROS2 Context.context`                             | `ROS2 Publish Laser Scan.context`              |
| `Isaac Read Simulation Time.simulationTime`        | `ROS2 Publish Laser Scan.timestamp`            |
| `Isaac Read Lidar Beams Node.azimuthRange`         | `ROS2 Publish Laser Scan.azimuthRange`         |
| `Isaac Read Lidar Beams Node.depthRange`           | `ROS2 Publish Laser Scan.depthRange`           |
| `Isaac Read Lidar Beams Node.horizontalFov`        | `ROS2 Publish Laser Scan.horizontalFov`        |
| `Isaac Read Lidar Beams Node.horizontalResolution` | `ROS2 Publish Laser Scan.horizontalResolution` |
| `Isaac Read Lidar Beams Node.intensities`          | `ROS2 Publish Laser Scan.intensitiesData`      |
| `Isaac Read Lidar Beams Node.linearDepthData`      | `ROS2 Publish Laser Scan.linearDepthData`      |
| `Isaac Read Lidar Beams Node.numCols`              | `ROS2 Publish Laser Scan.numCols`              |
| `Isaac Read Lidar Beams Node.numRows`              | `ROS2 Publish Laser Scan.numRows`              |
| `Isaac Read Lidar Beams Node.rotationRate`         | `ROS2 Publish Laser Scan.rotationRate`         |


El flujo completo queda:

```text
On Playback Tick
  -> Isaac Create Render Product
  -> ROS2 RTX Lidar Helper
```
El Action Graph queda algo asi: 
<img width="891" height="557" alt="image" src="https://github.com/user-attachments/assets/4f727b3d-a54b-4990-9f67-945c592c7796" />

## Parte 4: Configurar Isaac Create Render Product

Selecciona el nodo `Isaac Create Render Product` y configura:

| Campo | Valor |
| --- | --- |
| `cameraPrim` | `/World/Carter/chassis_link/camera_mount/XT_32_10hz /Lidar_2D` |
| `enabled` | `True` |

Si tu sensor quedo en otra ruta, usa esa ruta exacta. Aunque el campo se llame `cameraPrim`, tambien se usa para sensores RTX como LiDAR.

## Parte 5: Ros2 Publish Laser Scan

Selecciona el nodo `ROS2 Publish Laser Scan` y configura:

| Campo | Valor |
| --- | --- |
| `topicName` | `scan` |
| `frameId` | `sim_lidar` |
| `queueSize` | `10` |
| `frameSkipCount` | `0` |
| `enabled` | `True` |

Con esta configuracion, el topico queda:

```text
/scan
```

El tipo de mensaje es:

```text
sensor_msgs/msg/LaserScan
```

Cuando `type = laser_scan`, el mensaje se publica cuando el LiDAR completa un escaneo. En un sensor rotatorio, eso puede tardar varios frames segun la velocidad de rotacion y el timestep de simulacion.

## Parte 6: Probar desde ROS2

Antes de ejecutar cualquier comando ROS, abre una terminal ROS con:

```bash
talos@IsaacUN:~/isaac-sim$ ./terminal_b.bash
```

Con la simulacion en `Play`, ejecuta los comandos siguientes dentro de esa terminal.

```bash
ros2 topic list
```

Debe aparecer:

```text
/scan
```

Revisa el tipo:

```bash
/scan
```

Revisa la frecuencia:

```bash
ros2 topic hz /scan
```

Imprime un mensaje:

```bash
ros2 topic echo /scan --once
```

Mueve Carter o coloca objetos alrededor y revisa que cambien los rangos del `LaserScan`.

## Parte 7: Visualizar en RViz2

Usa la misma terminal ROS o abre otra con:

```bash
talos@IsaacUN:~/isaac-sim$ ./terminal_b.bash
```

Abre RViz2:

```bash
rviz2
```

En RViz2:

1. En `Fixed Frame`, cambialo a  `sim_lidar`.
2. En `Displays`, oprime `Add`.
3. Selecciona `By topic`.
4. Agrega `/scan` como `LaserScan`.
5. En LaserScan cambia Size(m)  a 0.1

Se deberia ver una pestaña parecida a: 
<img width="1661" height="733" alt="image" src="https://github.com/user-attachments/assets/145df5c3-0040-47e2-9a1a-ea9a63a421b0" />



