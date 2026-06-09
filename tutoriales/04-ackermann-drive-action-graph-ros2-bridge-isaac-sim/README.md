# Tutorial 04: Ackermann Drive Action Graph + ROS2 Bridge en Isaac Sim

Fuente oficial de referencia: [ROS 2 Ackermann Controller, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_ackermann_controller.html).

## Objetivo

El objetivo de esta parte es controlar un robot con direccion Ackermann en Isaac Sim usando ROS2 Bridge y un Action Graph.

La idea central es:

- ROS2 publica comandos `ackermann_msgs/msg/AckermannDriveStamped`.
- Isaac Sim recibe el topico con un nodo `ROS2 Subscribe AckermannDrive`.
- Un `Ackermann Controller` calcula angulos de direccion y velocidades de rueda.
- Dos `Articulation Controller` aplican esos comandos a los joints del robot: uno para direccion y otro para traccion.

## Requisitos

- Tener ROS2 instalado y con el entorno cargado antes de abrir Isaac Sim.
- Tener habilitada la extension de ROS2 Bridge en Isaac Sim.
- Tener instalado el paquete `ackermann_msgs`.
- Tener disponibles los paquetes `isaac_tutorials` y `cmdvel_to_ackermann` si vas a usar los nodos de prueba oficiales.
- Confirmar que Isaac Sim y ROS2 usan el mismo `ROS_DOMAIN_ID`.
- Tener una escena con un robot compatible con direccion Ackermann, por ejemplo Leatherback.

Para instalar `ackermann_msgs`:

```bash
sudo apt install ros-$ROS_DISTRO-ackermann-msgs
```

Los paquetes `isaac_tutorials` y `cmdvel_to_ackermann` vienen del workspace oficial `IsaacSim-ros_workspaces`.

## Parte 1: Preparar la escena

1. Abre Isaac Sim desde una terminal donde ya tengas cargado ROS2.
2. Crea una escena nueva.
3. Agrega un piso con:

```text
Create > Environments > Flat Grid
```

4. En el `Content Browser`, busca:

```text
Isaac Sim > ROBOTS > NVIDIA > Leatherback
```

5. Arrastra `leatherback.usd` al `Stage`.
6. Selecciona el prim del robot y deja su `Translate` en `0, 0, 0`.
7. Presiona `Play` brevemente para confirmar que el robot queda estable sobre el piso.

Si quieres partir desde una escena ya configurada por NVIDIA, revisa estas rutas en el `Content Browser`:

```text
Isaac Sim > Sample > ROS2 > Robots > Leatherback_ROS
Isaac Sim > Sample > ROS2 > Scenario > leatherback_ackermann
```

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

Si el nodo muestra los puertos como `Position Commands` o `Velocity Commands`, usa esos puertos equivalentes.

## Parte 3: Configurar ROS2 Subscribe AckermannDrive

Selecciona el nodo `ROS2 Subscribe AckermannDrive` y configura:

```text
topicName = ackermann_cmd
```

En ROS2 normalmente lo veras como:

```text
/ackermann_cmd
```

El mensaje recibido es:

```text
ackermann_msgs/msg/AckermannDriveStamped
```

El subscriber entrega estos campos importantes al grafo:

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

Con la simulacion en `Play`, abre una terminal con ROS2 cargado y verifica que el topico exista:

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

## Parte 7: Controlar con Twist y teclado

El tutorial oficial tambien muestra una forma de controlar Leatherback desde comandos `Twist`.

La idea es:

- `teleop_twist_keyboard` publica `geometry_msgs/msg/Twist`.
- `cmdvel_to_ackermann` convierte `cmd_vel` a `ackermann_msgs/msg/AckermannDriveStamped`.
- Isaac Sim recibe el resultado en `ackermann_cmd`.

Abre la escena preconfigurada:

```text
Isaac Sim > Sample > ROS2 > Scenario > leatherback_ackermann
```

Presiona `Play` y en una terminal ejecuta:

```bash
ros2 launch cmdvel_to_ackermann cmdvel_to_ackermann.launch.py acceleration:=0.5 steering_velocity:=0.5
```

En otra terminal ejecuta:

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

## Errores comunes

- El robot no se mueve: revisa que el `Articulation Controller` apunte al prim correcto.
- Los joints no responden: revisa los nombres de los joints de direccion y traccion.
- El topico no aparece: revisa que ROS2 Bridge este habilitado y que Isaac Sim se haya abierto desde una terminal con ROS2 cargado.
- Isaac Sim y ROS2 no se ven: revisa que ambos usen el mismo `ROS_DOMAIN_ID`.
- El robot gira al lado contrario: cambia el signo de `steeringAngle` o revisa `invertSteering`.
- La direccion se mueve pero el robot no avanza: revisa que `wheelRotationVelocity` este conectado a `Velocity Commands` del controlador de ruedas.
- El robot avanza pero no gira: revisa que `wheelAngles` este conectado a `Position Commands` del controlador de direccion.
