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

El script de joystick se puede encontrar en los archivos del tutorial

```text
tutoriales/03-differential-drive-isaac-sim/turtlebot_joy_combined.py
```

Este script publica mensajes `Twist` en `/cmd_vel` usando un joystick detectado con `pygame`.


## Parte 1: Preparar la escena

1. Abre Isaac Sim con:

   ```bash
   talos@IsaacUN:~/isaac-sim$ ./launch_isaacsim.bash
   ```

2. Carga la escena del ultimo tutorial.
3. Verifica que el robot este sobre el piso y no atravesando la geometria.
4. Presiona `Play` brevemente para confirmar que la simulacion no explota ni lanza errores de fisica.



## Parte 2: Crear el Action Graph

Abre:

```text
Window > Graph Editors > Action Graph
```

Crea un Action Graph nuevo y agrega estos nodos:

- `On Playback Tick`
- `ROS2 Context`
- `ROS2 Subscribe Twist`
- `2 Break 3-Vector`
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

Conecta el contexto ROS2.

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


## Parte 6: Configurar Articulation Controller
En esta version del turtlebot hay que revisar el `Articulation Root`. En TurtleBot3 suele ser mas facil dejar el root en el prim principal del robot. 
Por lo que el paso 0 es cambiar el Articulation Root de a_namespace_base_footprint eliminando el api de articulation root y agregandoselo al prim principal turtlebot3_burguer dando click derecho ADD > Physics > Articulation root 

En el nodo `Articulation Controller`:

1. Agrega como target el prim principal (turtlebot3_burguer) del TurtleBot3.
2. Verifica que ese prim tenga el `Articulation Root`.
3. Crea un array de joints con `Make Array`.
4. Usa tokens, no strings, para los nombres:

```text
wheel_left_joint
wheel_right_joint
```

Conecta la salida de velocidades del `Differential Controller` a `Velocity Commands` del `Articulation Controller`.
Ejemplo del control 
<img width="1085" height="710" alt="image" src="https://github.com/user-attachments/assets/360f13be-254b-4460-8c37-1a9c52ac5f65" />

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

Para usar el joystick, abre una terminal ROS con:

```bash
talos@IsaacUN:~/isaac-sim$ ./terminal_b.bash
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

## Generalizar 
Se puede guardar el action Graph para cargarlo en otro entorno y solo cambiar los parametros especificos de cada robot.

