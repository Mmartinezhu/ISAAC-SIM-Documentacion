# Tutorial 06: IMU Sensor in Isaac Sim - Action Graph + ROS2 Bridge

Fuente oficial de referencia:

- [IMU Sensor, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physics_imu.html).
- [Isaac Read IMU Node, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.sensors.physics/docs/ogn/OgnIsaacReadIMU.html).
- [ROS2 Publish Imu, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.ros2.bridge/docs/ogn/OgnROS2PublishImu.html).

## Objetivo

El objetivo de esta parte es montar un sensor IMU sobre un robot tipo Carter v1 en Isaac Sim y publicar sus datos hacia ROS2 usando ROS2 Bridge y Action Graph.

El flujo de trabajo:

- Isaac Sim usa un prim `Imu_Sensor` agregado sobre un rigid body del robot.
- El IMU se monta como hijo del chasis para que mida el movimiento de Carter.
- `Isaac Read IMU` lee orientacion, velocidad angular y aceleracion lineal.
- `ROS2 Publish Imu` publica esos datos como `sensor_msgs/msg/Imu`.
- ROS2 permite validar el topico con `ros2 topic echo`, `ros2 topic hz` y RViz2.

## Parte 1: Preparar la escena

1. Abre Isaac Sim con:

   ```bash
   talos@IsaacUN:~/isaac-sim$ ./launch_isaacsim.bash
   ```

2. Crea o abre una escena con Carter v1.
3. Verifica que el robot tenga un `PhysicsScene` en el `Stage`.
4. Revisa que Carter quede estable al presionar `Play`.

La ruta de referencia del robot dentro de los assets de Isaac Sim es:

```text
Robots/NVIDIA/Carter/carter_v1.usd
```

Para este tutorial se usara esta jerarquia de ejemplo:

```text
/World/Carter/com_offset/imu
```

## Parte 2: Crear y montar el IMU en Carter

Selecciona el link del chasis:

```text
/World/Carter/com_offset/imu
```

Crea el sensor desde el menu:

```text
Create > Sensors > Imu Sensor
```

Renombra el prim como:

```text
IMU_Sensor
```

La jerarquia debe quedar parecida a:

```text
/World/Carter/chassis_link/IMU_Sensor
```

En general, los sensores IMU deben agregarse sobre prims que sean rigid bodies o hijos de un rigid body. Por eso se recomienda montarlo en `/World/Carter/com_offset/imu`.

Configura una pose local simple:

| Campo | Valor |
| --- | --- |
| `Translate X` | `0.0` |
| `Translate Y` | `0.0` |
| `Translate Z` | `0.25` |
| `Rotate X` | `0.0` |
| `Rotate Y` | `0.0` |
| `Rotate Z` | `0.0` |

La orientacion del IMU define sus ejes locales. Para empezar, deja la rotacion en cero y verifica el signo de las lecturas cuando Carter avance o gire.

## Parte 3: Revisar propiedades del sensor

Selecciona `IMU_Sensor` y abre sus propiedades.

Valores iniciales recomendados:

| Campo | Valor |
| --- | --- |
| `enabled` | `True` |
| `sensorPeriod` | `0.0` |
| `angularVelocityFilterWidth` | `1` |
| `linearAccelerationFilterWidth` | `1` |
| `orientationFilterWidth` | `1` |

Notas:

- `sensorPeriod = 0.0` publica con la frecuencia de la simulacion.
- Si el sensor se ve muy ruidoso, sube los `FilterWidth`.
- Si necesitas una frecuencia fija, usa `sensorPeriod`; por ejemplo `0.01` equivale aproximadamente a 100 Hz.

## Parte 4: Crear el Action Graph

Abre:

```text
Window > Graph Editors > Action Graph
```

Crea un Action Graph nuevo y agrega estos nodos:

- `On Playback Tick`
- `ROS2 Context`
- `Isaac Read Simulation Time`
- `Isaac Read IMU Node`
- `ROS2 Publish Imu`
- `ROS2 QoS Profile`

Conecta las senales asi:

| Salida | Entrada |
| --- | --- |
| `On Playback Tick.tick` | `Isaac Read IMU.execIn` |
| `Isaac Read IMU Node.execOut` | `ROS2 Publish Imu.execIn` |
| `Isaac Read IMU Node.orientation` | `ROS2 Publish Imu.orientation` |
| `Isaac Read IMU Node.angularVelocity` | `ROS2 Publish Imu.angularVelocity` |
| `Isaac Read IMU Node.linearAcceleration` | `ROS2 Publish Imu.linearAcceleration` |
| `Isaac Read Simulation Time.simulationTime` | `ROS2 Publish Imu.timeStamp` |
| `ROS2 Context.context` | `ROS2 Publish Imu.context` |
| `ROS2 QoS Profile.QoS Profile  ` | `ROS2 Publish Imu.QoS Profile` |

El Action Graph queda asi: 
<img width="948" height="627" alt="image" src="https://github.com/user-attachments/assets/4dffa82f-3407-41ec-851c-751eb1d935f0" />



El flujo completo queda:

```text
On Playback Tick
  -> Isaac Read IMU
  -> ROS2 Publish Imu
```

## Parte 5: Configurar Isaac Read IMU

Selecciona el nodo `Isaac Read IMU` y configura:

| Campo | Valor |
| --- | --- |
| `imuPrim` | `/World/Carter/com_offset/imu` |
| `readGravity` | `True` |

`readGravity = True` hace que la aceleracion lineal incluya el efecto de la gravedad, similar a lo que esperarias de una IMU real. Si quieres analizar solo aceleraciones de movimiento, prueba tambien con `False`.

## Parte 6: Configurar ROS2 Publish Imu

Selecciona el nodo `ROS2 Publish Imu` y configura:

| Campo | Valor |
| --- | --- |
| `topicName` | `imu` |
| `queueSize` | `10` |
| `publishOrientation` | `True` |
| `publishAngularVelocity` | `True` |
| `publishLinearAcceleration` | `True` |

Con esta configuracion, el topico queda:

```text
/imu
```

El tipo de mensaje es:

```text
sensor_msgs/msg/Imu
```

## Parte 7: Probar desde ROS2

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
/imu
```

Revisa el tipo del topico:

```bash
ros2 topic info /imu
```

Revisa la frecuencia:

```bash
ros2 topic hz /imu
```

Imprime un mensaje:

```bash
ros2 topic echo /imu --once
```

Mueve Carter y revisa que cambien:

- `orientation`
- `angular_velocity`
- `linear_acceleration`

## Parte 8: Visualizar en RViz2

Usa la misma terminal ROS o abre otra con:

```bash
talos@IsaacUN:~/isaac-sim$ ./terminal_b.bash
```

Abre RViz2:

```bash
rviz2
```

Para una prueba rapida:

1. En `Displays`, agrega `By topic`.
2. Selecciona `/imu`.
3. Si RViz2 no muestra el IMU por falta de TF, valida primero con `ros2 topic echo`.

