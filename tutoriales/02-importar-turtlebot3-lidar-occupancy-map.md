# Tutorial 02: TurtleBot3 Burger, Lidar y Occupancy Map

Fuentes oficiales de referencia:

- [URDF Import: Turtlebot, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_turtlebot.html)
- [PhysX SDK Lidar, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physx_lidar.html)
- [RTX Lidar Sensor, NVIDIA Isaac Sim latest](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_rtx_lidar.html)
- [Mapping / Occupancy Map Generator, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/digital_twin/ext_isaacsim_asset_generator_occupancy_map.html)

Esta guia esta escrita como material de estudio propio en espanol. Sigue el flujo general de la documentacion oficial, pero no es una traduccion literal.

## Objetivo

En este tutorial vas a:

- Importar el modelo `turtlebot3_burger.urdf` en Isaac Sim.
- Colocar el TurtleBot3 Burger dentro de un entorno simple.
- Agregar un sensor Lidar al robot.
- Hacer visibles las lineas de medicion del Lidar para depurar la escena.
- Generar un Occupancy Map 2D del entorno.
- Entender para que sirve ese mapa en navegacion robotica.

> Nota de nombre: el modelo oficial se llama `turtlebot3_burger`. A veces lo escribimos como `burguer` por costumbre en espanol, pero el archivo del paquete ROS usa `burger`.

## Conceptos clave

### URDF

URDF es un formato XML usado en ROS para describir un robot: enlaces, articulaciones, geometria visual, colisiones, masas y relaciones entre partes. Isaac Sim puede importar un URDF y convertirlo a USD para usarlo dentro del Stage.

### Lidar

Un Lidar emite rayos y mide distancias contra el entorno. En simulacion sirve para probar percepcion, navegacion, evasion de obstaculos y pipelines ROS sin depender de hardware real.

### Draw Lines

`drawLines` es una visualizacion de depuracion. Muestra los rayos del sensor en el viewport. No cambia la fisica ni crea el mapa por si mismo; solo te permite confirmar que el sensor esta bien ubicado, bien orientado y que esta detectando colisiones.

### Occupancy Map

Un Occupancy Map es una grilla 2D donde cada celda representa una parte del espacio:

- Libre: el robot puede pasar.
- Ocupada: hay una pared, obstaculo o geometria con colision.
- Desconocida: no hay informacion suficiente o queda fuera del area calculada.

En navegacion movil, este mapa se usa para planear rutas, localizar el robot y alimentar herramientas como Nav2 en ROS 2. En este tutorial el mapa se genera desde la geometria del Stage, no desde SLAM en tiempo real.

## Requisitos

- Isaac Sim 5.1.0 instalado y funcionando.
- ROS 2 instalado y configurado si vas a seguir el flujo oficial de TurtleBot3.
- Variable `ROS_DISTRO` definida en una terminal con ROS 2 sourceado.
- `xacro` instalado.
- URDF Importer habilitado en Isaac Sim.
- Un entorno simple, por ejemplo `Simple_Room`, o una escena con `GroundPlane`, `PhysicsScene`, luz y geometria con colisiones.

Instala `xacro` si hace falta:

```bash
sudo apt install ros-$ROS_DISTRO-xacro
```

## Parte 1: Preparar el URDF de TurtleBot3 Burger

Abre una terminal donde ROS 2 este sourceado. Por ejemplo:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

Clona el paquete de TurtleBot3 si todavia no lo tienes:

```bash
git clone -b $ROS_DISTRO https://github.com/ROBOTIS-GIT/turtlebot3.git turtlebot3
```

Entra a la carpeta donde esta el URDF:

```bash
cd turtlebot3/turtlebot3_description/urdf
```

Genera una version procesada del URDF usando `xacro`:

```bash
namespace=""
xacro ./turtlebot3_burger.urdf "namespace:=${namespace:+$namespace/}" > tb3_burger_processed.urdf
```

El archivo que importaremos en Isaac Sim sera:

```text
tb3_burger_processed.urdf
```

## Parte 2: Crear o cargar un entorno en Isaac Sim

Para este tutorial conviene usar una habitacion simple porque despues generaremos un mapa.

Opcion recomendada:

1. Abre Isaac Sim.
2. Crea una escena nueva con `File > New`.
3. En el Content Browser, busca un entorno simple, por ejemplo:

```text
Isaac Sim/Environments/Simple_Room/simple_room.usd
```

4. Arrastra el USD al Stage.
5. Asegurate de que el entorno quede cerca del origen.

Si prefieres una escena minima, crea estos elementos manualmente:

```text
Create > Physics > Physics Scene
Create > Physics > Ground Plane
Create > Lights > Distant Light
```

Para el Occupancy Map, cualquier obstaculo que quieras que aparezca en el mapa debe tener colision habilitada.

## Parte 3: Importar TurtleBot3 Burger

Si el importador URDF no aparece, habilitalo desde:

```text
Window > Extensions
```

Busca y activa:

```text
isaacsim.asset.importer.urdf
```

Luego importa el robot:

1. Ve a `File > Import`.
2. Selecciona `tb3_burger_processed.urdf`.
3. En la ventana de importacion, usa una configuracion pensada para robot movil:
   - Modelo referenciado si quieres mantener el USD como asset reutilizable.
   - Base movible en la seccion de links.
   - Joints de ruedas configurados para velocidad, especialmente `wheel_left_joint` y `wheel_right_joint`.
4. Define una carpeta de salida para el USD si quieres controlar donde queda guardado.
5. Presiona `Import`.

Despues de importar:

1. Selecciona el prim principal del TurtleBot3 en el Stage.
2. Muevelo para dejarlo sobre el piso, no sobre una mesa ni atravesando geometria.
3. Presiona `Play`.
4. Verifica que el robot cae o se estabiliza sobre el suelo.

Si el robot se comporta raro, revisa las propiedades de articulaciones. Para ruedas con control por velocidad, una regla practica es usar alta amortiguacion y rigidez cero en los drives de las ruedas.

## Parte 4: Agregar un Lidar con lineas visibles

Para este tutorial usaremos **PhysX SDK Lidar** porque tiene la propiedad `drawLines`, que es exactamente lo que queremos visualizar.

Crea el sensor desde el menu:

```text
Create > Sensors > PhysX Lidar > Rotating
```

Si tu version muestra los sensores bajo otro menu, busca una ruta parecida a:

```text
Create > Isaac > Sensors > PhysX Lidar > Rotating
```

Renombra el prim a algo claro:

```text
Lidar
```

Ahora fijalo al robot:

1. En el Stage, localiza el link del sensor del TurtleBot3. Normalmente sera algo como:

```text
/World/turtlebot3_burger/base_scan
```

2. Arrastra el prim `Lidar` para que quede como hijo de `base_scan`.
3. Selecciona `Lidar`.
4. En Transform, deja la traslacion local en cero si quieres que coincida con `base_scan`.
5. Si queda enterrado o desalineado, ajusta su posicion local ligeramente en Z.

Activa las lineas:

1. Selecciona el prim `Lidar`.
2. En el panel de propiedades, busca `Raw USD Properties` o las propiedades especificas del sensor.
3. Activa:

```text
drawLines
```

4. Ajusta `rotationRate`:
   - `1.0` para verlo girando a 1 Hz.
   - `0.0` para disparar rayos en todas las direcciones al mismo tiempo, util para depurar.

Presiona `Play`. Deberias ver lineas saliendo desde el Lidar hacia paredes, suelo u obstaculos. Si no ves nada, revisa que el entorno tenga colisiones y que el sensor no este dentro de otra geometria.

## Nota: PhysX Lidar vs RTX Lidar

En Isaac Sim 5.x tambien existe RTX Lidar. Es mas realista para ciertos casos porque usa el pipeline RTX y puede trabajar con materiales no visuales. Sin embargo, para ver rayos de forma sencilla en el viewport, el camino de `drawLines` pertenece al PhysX SDK Lidar.

Si mas adelante usamos RTX Lidar, la visualizacion se hace normalmente con Debug Draw o publicando datos a ROS 2 como `LaserScan` o `PointCloud2`.

## Parte 5: Verificar que el Lidar detecta la escena

Haz una prueba rapida:

1. Crea un cubo cerca del robot.
2. Agregale colision desde el panel de propiedades:

```text
Add > Physics > Collider
```

3. Presiona `Play`.
4. Observa si las lineas del Lidar chocan contra el cubo.
5. Mueve el cubo y confirma que las lineas cambian.

Esto confirma tres cosas:

- El Lidar esta activo.
- Las lineas estan visibles.
- La geometria tiene colision detectable.

## Parte 6: Generar un Occupancy Map

Abre la herramienta de mapa:

```text
Tools > Robotics > Occupancy Map
```

Si no aparece, habilita la extension:

```text
isaacsim.asset.gen.omap
```

La herramienta calcula un mapa 2D usando la geometria con colision del Stage. No necesita que el robot conduzca ni que el Lidar genere datos; el Lidar en este tutorial sirve para depurar y entender la percepcion.

### Configurar el mapa

En la ventana `Occupancy Map`:

1. Coloca el `Origin` en una zona libre, no dentro de una pared, mesa, robot u obstaculo.
2. Ajusta el eje Z del origen a una altura razonable para mapear el entorno. Un valor inicial util es:

```text
Z = 0.1
```

3. Define los limites del mapa con `Lower Bound` y `Upper Bound`, o selecciona el entorno y usa una opcion de bound/selection si esta disponible.
4. Elige `Cell Size`. Por ejemplo:

```text
0.05
```

Eso significa 5 cm por pixel. Menor valor da mas detalle, pero el mapa sera mas pesado.

5. Presiona `CALCULATE`.
6. Presiona `VISUALIZE IMAGE`.
7. Guarda la imagen del mapa desde la ventana de visualizacion.

Si quieres usarlo con ROS 2/Nav2, guarda tambien los parametros importantes:

- Resolucion o `cell size`.
- Origen.
- Orientacion/rotacion de la imagen.
- Colores o convencion de ocupado/libre.

La ventana de visualizacion puede mostrar informacion en formato util para mapas ROS. Si aparece una opcion tipo `ROS Occupancy Map Parameters`, usala para registrar esos datos junto con la imagen.

## Para que sirve el Occupancy Map

Un Occupancy Map convierte una escena 3D en una representacion 2D navegable. En robotica movil sirve para:

- Planificacion global: calcular una ruta desde A hasta B evitando paredes y obstaculos fijos.
- Localizacion: comparar sensores del robot contra un mapa conocido.
- Simulacion de navegacion: probar algoritmos antes de correrlos en un robot real.
- Validacion de entornos: revisar si una escena tiene pasillos, paredes y zonas libres bien definidas.
- Preparacion para Nav2: usar el mapa como entrada para navegacion autonoma en ROS 2.

La idea importante es esta:

```text
Lidar = sensor que mide el entorno.
Occupancy Map = representacion 2D del entorno para navegar.
```

En un robot real, muchas veces el mapa se construye con SLAM usando Lidar. En Isaac Sim, tambien puedes generar el mapa directamente desde la geometria del Stage para acelerar pruebas.

## Checklist de validacion

Antes de cerrar este tutorial, confirma que lograste:

- Importar `tb3_burger_processed.urdf`.
- Ver el TurtleBot3 Burger en el Stage.
- Ubicar el robot sobre el suelo.
- Crear un Lidar y hacerlo hijo de `base_scan`.
- Activar `drawLines`.
- Ver las lineas del Lidar durante la simulacion.
- Crear o cargar un entorno con colisiones.
- Abrir `Tools > Robotics > Occupancy Map`.
- Calcular y visualizar el mapa.
- Guardar la imagen del mapa.

## Errores comunes

### El importador URDF no aparece

Activa `isaacsim.asset.importer.urdf` desde `Window > Extensions`.

### `xacro` no existe

Instala el paquete con:

```bash
sudo apt install ros-$ROS_DISTRO-xacro
```

Asegurate tambien de haber sourceado ROS 2.

### El robot queda flotando o atravesando el piso

Mueve el prim principal del TurtleBot3 hasta dejarlo justo encima del suelo. Luego presiona `Play` para que la fisica lo estabilice.

### Las lineas del Lidar no aparecen

Revisa:

- Que estas usando PhysX SDK Lidar si quieres `drawLines`.
- Que `drawLines` esta activado.
- Que la simulacion esta corriendo.
- Que el sensor no esta dentro de una pared o dentro del robot.
- Que los objetos cercanos tienen colision.

### El Occupancy Map sale vacio

Revisa:

- La geometria debe tener colisiones habilitadas.
- El origen del mapa no puede estar ocupado.
- Los bounds deben cubrir la habitacion.
- La altura Z debe cortar una zona donde existan paredes u obstaculos.
- Puedes visualizar colisiones desde el icono de ojo del viewport y mostrar las mallas de fisica.

## Mini ejercicio

Agrega tres obstaculos simples dentro del cuarto:

1. Un cubo grande como pared baja.
2. Un cilindro como columna.
3. Una caja pequena cerca del TurtleBot3.

Activa colisiones en los tres. Luego:

1. Observa las lineas del Lidar.
2. Genera de nuevo el Occupancy Map.
3. Compara el mapa antes y despues de agregar obstaculos.

La meta es entender que el Lidar te ayuda a ver que detectaria el robot, mientras que el Occupancy Map te da una representacion util para planificacion.

## Siguiente paso

El siguiente tutorial natural es conectar el TurtleBot3 con ROS 2: publicar `LaserScan`, mover el robot con `cmd_vel` y visualizar datos en RViz2.
