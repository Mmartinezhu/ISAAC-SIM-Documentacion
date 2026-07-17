# Tutorial 11: ROS2 Navigation con Block World Generator en Isaac Sim

Fuente oficial de referencia:

- [ROS 2 Navigation with Block World Generator, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation_block_world.html).

## Objetivo

El objetivo de este tutorial es generar un mundo 3D desde un mapa de ocupacion 2D y navegarlo con Nova Carter usando Nav2.

El flujo de trabajo:

- `Block World Generator` convierte una imagen de mapa en paredes 3D con colisiones.
- Se agrega `Nova_Carter_ROS.usd` dentro del mundo generado.
- Se publica `/clock` para que Nav2 use tiempo de simulacion.
- Nav2 usa el mapa original para planear rutas y mover el robot en el mundo 3D.

## Requisitos

- Haber completado el Tutorial 10 de navegacion con Nav2.
- Tener `carter_navigation` e `isaac_ros_navigation_goal` en el workspace.
- Tener `isaacsim.ros2.bridge` habilitado.

Abre Isaac Sim con:

```bash
talos@IsaacUN:~/isaac-sim$ ./launch_isaacsim.bash
```

Antes de ejecutar comandos ROS, abre una terminal ROS con:

```bash
talos@IsaacUN:~/isaac-sim$ ./terminal_b.bash
```

## Parte 1: Localizar el mapa de Carter

El ejemplo usa el mapa:

```text
carter_navigation/maps/carter_warehouse_navigation.png
```

Para ubicar el paquete desde la terminal ROS, ejecuta:

```bash
ros2 pkg prefix carter_navigation
```

Busca el directorio `share/carter_navigation/maps` dentro de esa ruta. Alli deben estar el PNG del mapa y su YAML asociado.

## Parte 2: Generar el mundo 3D

En Isaac Sim abre:

```text
Tools > Robotics > Block World Generator
```

En la herramienta:

1. Presiona `Load Image`.
2. Selecciona `carter_warehouse_navigation.png`.
3. Revisa que la imagen se visualice correctamente.
4. Presiona `Generate`.

Isaac Sim creara paredes 3D a partir de los pixeles ocupados del mapa. La geometria generada incluye colisiones, por lo que Carter no deberia atravesar las paredes.

## Parte 3: Agregar Nova Carter

En el `Content Browser`, abre:

```text
Isaac Sim > Samples > ROS2 > Robots
```

Arrastra al Stage:

```text
Nova_Carter_ROS.usd
```

Ubica Carter dentro de una zona libre del mundo generado:

1. Usa una posicion sobre el suelo.
2. Evita dejarlo intersectando paredes.
3. Alinea su orientacion con un pasillo o area abierta.
4. Presiona `Play` brevemente y confirma que queda estable.

## Parte 4: Agregar reloj ROS2

Nav2 debe usar el tiempo de simulacion. Para eso la escena necesita publicar `/clock`.

Crea un Action Graph, por ejemplo:

```text
/World/ROS_Clock
```

Agrega estos nodos:

- `On Playback Tick`
- `Isaac Read Simulation Time`
- `ROS2 Context`
- `ROS2 Publish Clock`

Conecta:

| Salida | Entrada |
| --- | --- |
| `On Playback Tick.tick` | `ROS2 Publish Clock.execIn` |
| `Isaac Read Simulation Time.simulationTime` | `ROS2 Publish Clock.timeStamp` |
| `ROS2 Context.context` | `ROS2 Publish Clock.context` |

Con la simulacion en `Play`, valida desde la terminal ROS:

```bash
ros2 topic echo /clock --once
```

Si `/clock` no publica, corrige este grafo antes de iniciar Nav2.

## Parte 5: Ejecutar Nav2

Con Isaac Sim en `Play`, ejecuta en la terminal ROS:

```bash
ros2 launch carter_navigation carter_navigation.launch.py
```

RViz2 debe cargar el mapa usado para generar el mundo 3D.

En RViz2:

1. Usa `2D Pose Estimate` para ubicar a Carter aproximadamente donde lo pusiste en Isaac Sim.
2. Verifica que el robot quede orientado igual que en la escena.
3. Usa `Navigation2 Goal`.
4. Haz clic y arrastra en una zona libre del mapa.

Resultado esperado: Nav2 planea una ruta sobre el mapa 2D y Carter se mueve dentro del mundo 3D generado.

## Parte 6: Validar topicos

En la terminal ROS revisa:

```bash
ros2 topic list
```

Topicos esperados:

```text
/clock
/tf
/tf_static
/odom
/map
/scan
```

Revisa odometria y TF:

```bash
ros2 topic echo /odom --once
ros2 topic echo /tf --once
```

Si el robot no se mueve, confirma que el launch de Nav2 este usando `use_sim_time` y que `/clock` este activo.

## Parte 7: Ajustar el mundo generado

Si la navegacion falla, revisa:

- El mapa PNG debe coincidir con el YAML que carga Nav2.
- Carter debe iniciar en una zona libre del mapa.
- Las paredes generadas no deben invadir pasillos por escalado incorrecto.
- La pose inicial enviada desde RViz2 debe coincidir con la posicion real del robot en Isaac Sim.
- Si el mapa esta rotado o desplazado, vuelve a generar el mundo o ajusta la pose inicial.

## Errores comunes

- Nav2 abre sin mapa: revisa que `carter_navigation.launch.py` encuentre el YAML correcto.
- Carter aparece fuera del mapa: usa `2D Pose Estimate` antes de mandar una meta.
- Carter choca con paredes invisibles: revisa la geometria generada y las colisiones.
- El robot se queda quieto: valida `/clock`, `/tf`, `/odom` y `/scan`.
- La ruta pasa por paredes: el mundo 3D y el mapa 2D no coinciden; usa el mismo PNG/YAML para ambos.
- RViz2 no muestra datos: revisa que la terminal haya sido abierta con `./terminal_b.bash`.
