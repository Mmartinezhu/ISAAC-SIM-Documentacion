# Tutorial 04: Ackermann Drive Action Graph + ROS2 Bridge en Isaac Sim

Fuente oficial de referencia: [ROS 2 Ackermann Controller, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_ackermann_controller.html).

## Objetivo

El objetivo de esta parte es controlar un robot con direccion Ackermann en Isaac Sim usando ROS2 Bridge y un Action Graph.

El flujo va a ser:

- ROS2 publica comandos `ackermann_msgs/msg/AckermannDriveStamped`.
- Isaac Sim recibe el topico con un nodo `ROS2 Subscribe AckermannDrive`.
- Un `Ackermann Controller` calcula angulos de direccion y velocidades de rueda.
- Dos `Articulation Controller` aplican esos comandos a los joints del robot: uno para direccion y otro para traccion.


## Parte 1: Preparar la escena

1. Abre Isaac Sim con:

   ```bash
   talos@IsaacUN:~/isaac-sim$ ./launch_isaacsim.bash
   ```

2. Crea y prepara una escena nueva, en Window > Browser > Isaac Sim assets. Hay diferentes Assets que pueden ayuda a tener una escena mas personalizada, por ejemplo importando jettracer_track_solid.
4. En el `Content Browser`, busca:

```text
Isaac Sim Assets > ROBOTS > NVIDIA > Leatherback
```

5. Arrastra `leatherback.usd` al `Stage` y verifica que no esta en contacto con el piso.
6. Presiona `Play` brevemente para confirmar que el robot queda estable sobre el piso.


## Parte 2: Crear el Action Graph

Abre:

```text
Window > Graph Editors > Action Graph
```

Crea un Action Graph nuevo y agrega estos nodos:

- `On Playback Tick`
- `ROS2 Context`
- `ROS2 QoS Profile`
- `ROS2 Subscribe AckermannDrive`
- `Ackermann Controller`
- `Articulation Controller` para los joints de direccion
- `Articulation Controller_01` para los joints de las ruedas

La estructura general del grafo es:

- `On Playback Tick` ejecuta el grafo en cada frame de simulacion.
- `ROS2 Context` entrega el contexto ROS2 al subscriber.
- `ROS2 QoS Profile` define la politica QoS usada por el subscriber.
- `ROS2 Subscribe AckermannDrive` recibe el comando ROS2.
- `Ackermann Controller` convierte `speed` y `steeringAngle` en comandos por rueda.
- Los `Articulation Controller` aplican posicion a la direccion y velocidad a las ruedas.

Conecta las senales principales asi:

| Salida | Entrada |
| --- | --- |
| `On Playback Tick.tick` | `ROS2 Subscribe AckermannDrive.execIn` |
| `ROS2 Context.context` | `ROS2 Subscribe AckermannDrive.context` |
| `ROS2 QoS Profile.qosProfile` | `ROS2 Subscribe AckermannDrive.qosProfile` |
| `ROS2 Subscribe AckermannDrive.speed` | `Ackermann Controller.speed` |
| `ROS2 Subscribe AckermannDrive.acceleration` | `Ackermann Controller.acceleration` |
| `ROS2 Subscribe AckermannDrive.steeringAngle` | `Ackermann Controller.steeringAngle` |
| `ROS2 Subscribe AckermannDrive.steeringAngleVelocity` | `Ackermann Controller.steeringAngleVelocity` |
| `Ackermann Controller.wheelAngles` | `Articulation Controller.positionCommand` |
| `Ackermann Controller.wheelRotationVelocity` | `Articulation Controller_01.velocityCommand` |

El action graph deberia quedar como la siguiente imagen: 
<img width="1184" height="534" alt="image" src="https://github.com/user-attachments/assets/859be362-294f-406c-9e72-cb3b9a4f6c6e" />


## Parte 3: Configurar ROS2 Subscribe AckermannDrive

Selecciona el nodo `ROS2 Subscribe AckermannDrive` y configura (normalmente ya tendra ese nombre):

```text
topicName = ackermann_cmd
```

El mensaje recibido es:

```text
ackermann_msgs/msg/AckermannDriveStamped
```

El subscriber entrega estos datos al grafo:

- `speed`: velocidad hacia adelante en `m/s`.
- `acceleration`: aceleracion en `m/s^2`.
- `steeringAngle`: angulo virtual de direccion en radianes.
- `steeringAngleVelocity`: velocidad de cambio del angulo de direccion en `rad/s`.

Para este tutorial, conecta `speed`, `acceleration`, `steeringAngle` y `steeringAngleVelocity` al `Ackermann Controller`.

## Parte 4: Configurar Ackermann Controller

Selecciona el nodo `Ackermann Controller` y usa estos valores iniciales para Leatherback:

| Campo | Valor |
| --- | --- |
| `backWheelRadius` | `0.052` |
| `frontWheelRadius` | `0.052` |
| `maxWheelRotation` | `0.7854` |
| `maxWheelVelocity` | `20.0` |
| `trackWidth` | `0.24` |
| `wheelBase` | `0.32` |
| `maxAcceleration` | `1.0` |
| `maxSteeringAngleVelocity` | `1.0` |

Estos parametros describen la geometria y limites del robot:

- `wheelBase`: distancia entre eje delantero y eje trasero.
- `trackWidth`: distancia lateral entre ruedas izquierda y derecha.
- `frontWheelRadius` y `backWheelRadius`: radio de las ruedas.
- `maxWheelRotation`: limite del angulo de direccion.
- `maxWheelVelocity`: limite de velocidad angular de las ruedas.

El nodo genera dos salidas principales:

- `wheelAngles`: arreglo con los angulos de direccion en orden izquierda, derecha.
- `wheelRotationVelocity`: arreglo con velocidades en orden rueda delantera izquierda, delantera derecha, trasera izquierda, trasera derecha.

## Parte 5: Configurar Articulation Controller

Se usan dos `Articulation Controller` porque el robot mezcla control de posicion y control de velocidad.

### Direccion

En el primer `Articulation Controller`:

1. Agrega como `targetPrim` el prim del Leatherback, normalmente:

```text
/Leatherback
```

2. En `jointNames`, agrega estos joints:

```text
Knuckle__Upright__Front_Left
Knuckle__Upright__Front_Right
```

3. Conecta:

```text
Ackermann Controller.wheelAngles -> Articulation Controller.positionCommand
```

Este controlador mueve los nudillos delanteros de direccion.

### Ruedas

En el segundo `Articulation Controller`, llamado normalmente `Articulation Controller_01`:

1. Usa el mismo `targetPrim`:

```text
/Leatherback
```

2. En `jointNames`, agrega estos joints en este orden:

```text
Wheel__Knuckle__Front_Left
Wheel__Knuckle__Front_Right
Wheel__Upright__Rear_Left
Wheel__Upright__Rear_Right
```

3. Conecta:

```text
Ackermann Controller.wheelRotationVelocity -> Articulation Controller_01.velocityCommand
```

Este controlador hace girar las ruedas.

Si el robot no responde, revisa que el prim seleccionado tenga el `Articulation Root` correcto y que los nombres de joints coincidan exactamente con los del asset.

## Parte 6: Probar desde ROS2

Antes de ejecutar cualquier comando ROS, abre una terminal ROS con:

```bash
talos@IsaacUN:~/isaac-sim$ ./terminal_b.bash
```

Con la simulacion en `Play`, ejecuta los comandos siguientes dentro de esa terminal y verifica que el topico exista:

```bash
ros2 topic list
```

Debe aparecer:

```text
/ackermann_cmd
```

Publica un comando simple hacia adelante:

```bash
ros2 topic pub /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped "{drive: {steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 0.8, acceleration: 0.5, jerk: 0.0}}"
```

Publica un comando con giro suave:

```bash
ros2 topic pub /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped "{drive: {steering_angle: 0.25, steering_angle_velocity: 0.5, speed: 0.8, acceleration: 0.5, jerk: 0.0}}"
```

Para detener el robot:

```bash
ros2 topic pub /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped "{drive: {steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 0.0, acceleration: 0.0, jerk: 0.0}}"
```

Tambien puedes usar el publisher oficial del paquete `isaac_tutorials`:

```bash
ros2 run isaac_tutorials ros2_ackermann_publisher.py
```

## Parte 7: Controlar con Twist y teclado ( o control)

El tutorial oficial tambien muestra una forma de controlar Leatherback desde comandos `Twist`.

La idea es:

- `teleop_twist_keyboard` publica `geometry_msgs/msg/Twist`.
- `cmdvel_to_ackermann` convierte `cmd_vel` a `ackermann_msgs/msg/AckermannDriveStamped`.
- Isaac Sim recibe el resultado en `ackermann_cmd`.

Abre la escena preconfigurada:

```text
Isaac Sim > Sample > ROS2 > Scenario > leatherback_ackermann
```

Presiona `Play`. Para este flujo usa una terminal ROS abierta con:

```bash
talos@IsaacUN:~/isaac-sim$ ./terminal_b.bash
```

En esa terminal ejecuta:

```bash
ros2 launch cmdvel_to_ackermann cmdvel_to_ackermann.launch.py acceleration:=0.5 steering_velocity:=0.5
```

En otra terminal ROS, abierta tambien con `./terminal_b.bash`, ejecuta:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Controles basicos:

- `i`: adelante.
- `,`: atras.
- `u`: adelante izquierda.
- `o`: adelante derecha.
- `m`: atras izquierda.
- `.`: atras derecha.
- `k`: detener.

Tambien esta la opcion de utilizar un joystick, corriendo el script de python que esta en la carpeta del tutorial con el nombre de teleop_arckerman. 

Abre una terminal ROS con:

```bash
talos@IsaacUN:~/isaac-sim$ ./terminal_b.bash
```

```bash
python3 teleop_ackerman.py
```

