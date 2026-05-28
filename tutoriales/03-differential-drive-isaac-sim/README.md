# Tutorial 03: Differential Drive en Isaac Sim

Fuente oficial de referencia: [Driving TurtleBot using ROS 2 Messages, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_drive_turtlebot.html).

## Objetivo

El objetivo de esta parte es controlar un TurtleBot3 importado en Isaac Sim usando mensajes ROS2 `geometry_msgs/Twist` en el topico `/cmd_vel`.

La idea central es:

- ROS2 publica comandos `Twist`.
- Isaac Sim recibe `/cmd_vel` con un nodo `ROS2 Subscribe Twist`.
- Un `Differential Controller` convierte velocidad lineal y angular en velocidades de rueda.
- Un `Articulation Controller` aplica esas velocidades a los joints del robot.

## Archivos de apoyo

El script de joystick queda organizado en la misma carpeta del tutorial:

```text
tutoriales/03-differential-drive-isaac-sim/turtlebot_joy_combined.py
```

Este script publica mensajes `Twist` en `/cmd_vel` usando un joystick detectado con `pygame`.

## Requisitos

- Tener ROS2 instalado y con el entorno cargado antes de abrir Isaac Sim.
- Tener habilitada la extension de ROS2 Bridge en Isaac Sim.
- Tener un TurtleBot3 importado como USD desde URDF.
- Revisar que el robot tenga joints de ruedas manejables.
- Confirmar que Isaac Sim y ROS2 usan el mismo `ROS_DOMAIN_ID`.

## Parte 1: Preparar la escena

1. Abre Isaac Sim.
2. Carga la escena donde esta el TurtleBot3.
3. Verifica que el robot este sobre el piso y no atravesando la geometria.
4. Presiona `Play` brevemente para confirmar que la simulacion no explota ni lanza errores de fisica.

Si el robot no se mueve despues, primero revisa el `Articulation Root`. En TurtleBot3 suele ser mas facil dejar el root en el prim principal del robot.

## Parte 2: Crear el Action Graph

Abre:

```text
Window > Graph Editors > Action Graph
```

Crea un Action Graph nuevo y agrega estos nodos:

- `On Playback Tick`
- `ROS2 Context`
- `ROS2 Subscribe Twist`
- `Break 3-Vector`
- `Scale To/From Stage Unit`
- `Differential Controller`
- `Articulation Controller`
- `Constant Token` para `wheel_left_joint`
- `Constant Token` para `wheel_right_joint`
- `Make Array`

## Parte 3: Configurar ROS2 Subscribe Twist

Selecciona el nodo `ROS2 Subscribe Twist` y configura:

```text
topicName = /cmd_vel
```

Conecta el contexto ROS2 si estas usando el nodo `ROS2 Context`.

## Parte 4: Conectar velocidades

El mensaje `Twist` trae velocidad lineal y angular como vectores 3D.

Para differential drive normalmente se usa:

- `linear.x` como velocidad hacia adelante.
- `angular.z` como giro.

Usa `Break 3-Vector` para separar los componentes y conecta esos valores al `Differential Controller`.

## Parte 5: Configurar Differential Controller

Para TurtleBot3 Burger puedes empezar con estos valores:

| Campo | Valor |
| --- | --- |
| Max Angular Speed | `1.0` |
| Max Linear Speed | `0.22` |
| Wheel Distance | `0.16` |
| Wheel Radius | `0.025` |

Estos valores pueden cambiar si usas otro robot o si el modelo fue escalado.

## Parte 6: Configurar Articulation Controller

En el nodo `Articulation Controller`:

1. Agrega como target el prim principal del TurtleBot3.
2. Verifica que ese prim tenga el `Articulation Root`.
3. Crea un array de joints con `Make Array`.
4. Usa tokens, no strings, para los nombres:

```text
wheel_left_joint
wheel_right_joint
```

Conecta la salida de velocidades del `Differential Controller` a `Velocity Commands` del `Articulation Controller`.

## Parte 7: Probar desde ROS2

Con la simulacion en `Play`, abre una terminal con ROS2 cargado y revisa:

```bash
ros2 topic list
```

Debe aparecer:

```text
/cmd_vel
```

Prueba mover el robot hacia adelante:

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Para detenerlo:

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Parte 8: Probar con joystick

Instala dependencias en tu entorno ROS2:

```bash
pip install pygame
```

Ejecuta el script:

```bash
python3 tutoriales/03-differential-drive-isaac-sim/turtlebot_joy_combined.py
```

El script publica en `/cmd_vel`.

Mapeo inicial:

- Eje izquierdo vertical: velocidad lineal.
- Eje derecho horizontal: velocidad angular.

Si tu control usa otros ejes, ajusta estas lineas en el script:

```python
linear_axis = -self.joystick.get_axis(1)
angular_axis = self.joystick.get_axis(3)
```

## Errores comunes

- El robot no se mueve: revisa que el `Articulation Controller` apunte al prim correcto.
- Las ruedas giran al reves: intercambia los joints o cambia el signo de la velocidad.
- `/cmd_vel` no aparece: revisa que ROS2 Bridge este habilitado y que Isaac Sim se haya abierto desde una terminal con ROS2 cargado.
- Isaac Sim y ROS2 no se ven: revisa que ambos usen el mismo `ROS_DOMAIN_ID`.
- El robot salta o se cae: revisa colisiones, masa, posicion inicial y que no este atravesando el piso.
