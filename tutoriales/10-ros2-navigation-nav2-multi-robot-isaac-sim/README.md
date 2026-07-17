# Tutorial 10: ROS2 Navigation con Nav2 y Multi-Robot en Isaac Sim

Fuentes oficiales de referencia:

- [ROS 2 Navigation, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html).
- [Multiple Robot ROS2 Navigation, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_multi_navigation.html).

## Objetivo

El objetivo de este tutorial es ejecutar Nav2 con Nova Carter en Isaac Sim y despues extender el flujo a varios robots usando namespaces.

El flujo de trabajo:

- Isaac Sim publica sensores, odometria, TF y reloj de simulacion.
- Nav2 recibe el mapa, la localizacion y los sensores.
- RViz2 permite enviar metas de navegacion.
- En multi-robot, cada Carter usa un namespace distinto: `/carter1`, `/carter2` y `/carter3`.

> Nota: las paginas oficiales de Isaac Sim 5.1.0 indican que navegacion con ROS2 esta soportada completamente en Linux. En Windows, RViz2 y Nav2 pueden fallar segun la configuracion.

## Requisitos

- Haber completado los tutoriales de sensores, odometria y TF.
- Tener ROS2, Nav2 y los paquetes de ejemplo de Isaac Sim disponibles en el workspace.
- Tener habilitada la extension `isaacsim.ros2.bridge`.
- Usar una terminal ROS abierta con `./terminal_b.bash` antes de cualquier comando `ros2`, `rviz2` o paquete ROS.

Abre Isaac Sim con:

```bash
talos@IsaacUN:~/isaac-sim$ ./launch_isaacsim.bash
```

Antes de ejecutar comandos ROS, abre una terminal ROS con:

```bash
talos@IsaacUN:~/isaac-sim$ ./terminal_b.bash
```

En esa terminal puedes verificar paquetes:

```bash
ros2 pkg list | grep -E "nav2_bringup|carter_navigation|iw_hub_navigation|isaac_ros_navigation_goal"
```

Si alguno no aparece, revisa que el workspace de Isaac Sim este cargado dentro de `terminal_b.bash`.

## Parte 1: Cargar Nova Carter para navegacion

En Isaac Sim, abre el navegador de ejemplos:

```text
Window > Examples > Robotics Examples
```

Carga la escena:

```text
ROS2 > Navigation > Nova Carter
```

Esta escena ya incluye un robot Nova Carter con los OmniGraphs necesarios para publicar datos hacia ROS2.

Antes de continuar:

1. Confirma que `isaacsim.ros2.bridge` este habilitada en `Window > Extensions`.
2. Revisa que Carter aparezca dentro del warehouse.
3. Presiona `Play` para iniciar la simulacion.
4. Deja Isaac Sim corriendo mientras ejecutas Nav2 desde la terminal ROS.

## Parte 2: Ejecutar Nav2 con Nova Carter

En la terminal ROS abierta con `./terminal_b.bash`, ejecuta:

```bash
ros2 launch carter_navigation carter_navigation.launch.py
```

RViz2 debe abrirse y cargar el mapa de ocupacion del warehouse. Si el mapa no aparece, detiene el launch con `Ctrl+C` y vuelve a ejecutarlo con la simulacion en `Play`.

En RViz2:

1. Verifica que `Fixed Frame` sea `map`.
2. Si Carter no aparece localizado correctamente, usa `2D Pose Estimate`.
3. Usa `Navigation2 Goal`.
4. Haz clic y arrastra sobre una zona libre del mapa para definir posicion y orientacion de la meta.

Resultado esperado: Nav2 calcula una trayectoria y Nova Carter empieza a moverse en Isaac Sim.

## Parte 3: Revisar topicos de navegacion

En la misma terminal ROS, puedes revisar:

```bash
ros2 topic list
```

Topicos importantes:

```text
/clock
/tf
/tf_static
/odom
/map
/scan
```

Tambien puedes validar algunos mensajes:

```bash
ros2 topic echo /clock --once
ros2 topic echo /tf --once
ros2 topic echo /odom --once
```

Si `/clock` no aparece, Nav2 puede quedar desincronizado con la simulacion.

## Parte 4: Enviar metas automaticamente

El paquete `isaac_ros_navigation_goal` puede mandar metas a Nav2 sin hacer clic en RViz2.

Con la escena de Nova Carter en `Play` y Nav2 corriendo, abre otra terminal ROS con:

```bash
talos@IsaacUN:~/isaac-sim$ ./terminal_b.bash
```

Ejecuta:

```bash
ros2 launch isaac_ros_navigation_goal isaac_ros_navigation_goal.launch.py
```

Parametros utiles dentro del launch:

| Parametro | Uso |
| --- | --- |
| `goal_generator_type` | `RandomGoalGenerator` para metas aleatorias o `GoalReader` para leer metas fijas |
| `map_yaml_path` | Archivo YAML del mapa usado por Nav2 |
| `iteration_count` | Numero de metas a enviar |
| `action_server_name` | Normalmente `navigate_to_pose` |
| `initial_pose` | Pose inicial usada antes de mandar metas |
| `goal_text_file_path` | Archivo con metas cuando se usa `GoalReader` |

Si modificas el launch o archivos del paquete, reconstruye y vuelve a cargar el workspace antes de repetir la prueba.

## Parte 5: Navegacion con iw.hub

Isaac Sim tambien incluye un escenario de warehouse con el robot `iw.hub`.

En Isaac Sim carga:

```text
Window > Examples > Robotics Examples > ROS2 > Navigation > iw_hub
```

Presiona `Play`. En una terminal ROS abierta con `./terminal_b.bash`, ejecuta:

```bash
ros2 launch iw_hub_navigation iw_hub_navigation.launch.py
```

RViz2 debe cargar el mapa correspondiente. Si la pose inicial ya esta definida por parametros, puedes enviar metas directamente con `Navigation2 Goal`.

## Parte 6: Preparar multi-robot

Para navegar con varios Carter a la vez se usan namespaces. Cada robot publica y recibe en su propio espacio de nombres:

| Robot | Namespace |
| --- | --- |
| Carter 1 | `/carter1` |
| Carter 2 | `/carter2` |
| Carter 3 | `/carter3` |

En Isaac Sim, los Action Graphs de cada `Nova_Carter_ROS_X` ya tienen configurado el nodo de namespace. En ROS2, los launch files de `carter_navigation` levantan Nav2 para cada robot con el mismo esquema de nombres.

## Parte 7: Ejecutar multi-robot en Hospital u Office

En Isaac Sim carga una de estas escenas:

```text
Window > Examples > Robotics Examples > ROS2 > Navigation > Multiple Robots > Hospital Scene
Window > Examples > Robotics Examples > ROS2 > Navigation > Multiple Robots > Office Scene
```

Presiona `Play`.

Para Hospital, en una terminal ROS abierta con `./terminal_b.bash`, ejecuta:

```bash
ros2 launch carter_navigation multiple_robot_carter_navigation_hospital.launch.py
```

Para Office:

```bash
ros2 launch carter_navigation multiple_robot_carter_navigation_office.launch.py
```

Se deben abrir tres ventanas de RViz2. Cada una corresponde a un namespace. En cada ventana:

1. Revisa el topico del display `Map` para identificar si corresponde a `/carter1`, `/carter2` o `/carter3`.
2. Confirma que el robot ya este localizado.
3. Usa `2D Nav Goal` en la ventana de cada robot.
4. Repite la prueba para los tres Carter.

## Parte 8: Metas automaticas para varios robots

Para mandar metas automaticas a varios robots, el launch de `isaac_ros_navigation_goal` debe crear un nodo por namespace.

Ejemplo conceptual:

```python
navigation_goal_node = Node(
    name="set_navigation_goal",
    package="isaac_ros_navigation_goal",
    executable="SetNavigationGoal",
    namespace="carter1",
    parameters=[{
        "goal_generator_type": "RandomGoalGenerator",
        "action_server_name": "navigate_to_pose",
        "iteration_count": 3,
        "initial_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    }],
    output="screen",
)
```

Duplica ese nodo para `carter2` y `carter3`, cambiando la variable del nodo y la `initial_pose` de cada robot. Si usas `GoalReader`, crea un archivo de metas separado para cada namespace.

Despues de modificar el launch, corre:

```bash
ros2 launch isaac_ros_navigation_goal isaac_ros_navigation_goal.launch.py
```

## Errores comunes

- RViz2 abre sin mapa: vuelve a lanzar Nav2 con Isaac Sim en `Play`.
- Carter no se mueve: revisa `/clock`, `/tf`, `/odom` y que el action server `navigate_to_pose` este activo.
- El robot aparece mal localizado: usa `2D Pose Estimate` o revisa la pose inicial del archivo de parametros.
- Multi-robot consume mucha CPU: cierra displays innecesarios en RViz2 y evita publicar imagenes si no las necesitas.
- Los robots chocan o ignoran comandos: revisa rendimiento, sincronizacion de sensores y que cada robot use su namespace correcto.
- No aparecen paquetes como `carter_navigation`: revisa que `terminal_b.bash` cargue el workspace correcto.
