# Tutorial 02: TurtleBot3 Burger, Lidar y Occupancy Map

Fuentes oficiales de referencia:

- [URDF Import: Turtlebot, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_turtlebot.html)
- [PhysX SDK Lidar, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physx_lidar.html)
- [RTX Lidar Sensor, NVIDIA Isaac Sim latest](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_rtx_lidar.html)
- [Mapping / Occupancy Map Generator, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/digital_twin/ext_isaacsim_asset_generator_occupancy_map.html)


## Objetivo

El objetivo de este tutorial es aprender a importar un modelo y agregar sensores. 

> Nota de nombre: el modelo oficial se llama `turtlebot3_burger`. A veces lo escribimos como `burguer` por costumbre en espanol, pero el archivo del paquete ROS usa `burger`.

## Conceptos importantes

### URDF

URDF es un formato XML usado en ROS para describir un robot: enlaces, articulaciones, geometria visual, colisiones, masas y relaciones entre partes. Isaac Sim puede importar un URDF y convertirlo a USD para usarlo dentro del Stage.
Para poder convertir los modelos desde FUSION360 a URDF puede revisar https://github.com/Mmartinezhu/FUSION_TO_URDF

### Lidar

Un Lidar emite rayos y mide distancias contra el entorno. En simulacion sirve para probar percepcion, navegacion, evasion de obstaculos sin depender de hardware real.

### Occupancy Map

Un Occupancy Map es una grilla 2D donde cada celda representa una parte del espacio:

- Libre: el robot puede pasar.
- Ocupada: hay una pared, obstaculo o geometria con colision.
- Desconocida: no hay informacion suficiente o queda fuera del area calculada.

En navegacion movil, este mapa se usa para planear rutas, localizar el robot y alimentar herramientas como Nav2 en ROS 2. En este tutorial el mapa se genera desde la geometria del Stage.

## Requisitos

- haber completado el tutorial anterior
- URDF Importer habilitado en Isaac Sim.

Abre Isaac Sim con:

```bash
talos@IsaacUN:~/isaac-sim$ ./launch_isaacsim.bash
```

## parte 0: Importar como ejemplo de isaac-sim (preferible no hacerlo)

Isaac-sim ofrece ejemplos y assets pre cargados que podemos importar de forma rapida desde 
window > browsers > Isaac sim assets Despues en el menu inferior ingresamos a Isaac sim assets > Robots > turtlebot3 > lo arrastramos al world. 


## Parte 1: Crear o cargar un entorno en Isaac Sim

Para este tutorial conviene usar una habitacion simple porque despues generaremos un mapa.

1. Agrega algunos objetos simples al mapa.
2. Agrega la fisica necesaria a los objetos.
3. Arrastra el USD al Stage.
4. Asegurate de que el entorno quede cerca del origen.

Para el Occupancy Map, cualquier obstaculo que quieras que aparezca en el mapa debe tener colision habilitada.

## Parte 2: Importar TurtleBot3 Burger

Si el importador URDF no aparece, habilitalo desde:

```text
Window > Extensions
```

Busca y activa:

```text
isaacsim.asset.importer.urdf
```
Activa tambien el AUTOLOAD

Luego importa el robot:

1. Ve a `File > Import`.
2. Selecciona `turtlebot3_burger.urdf`.
3. En la ventana de importacion, usa una configuracion pensada para robot movil:
   - Modelo referenciado si quieres mantener el USD como asset reutilizable.
   - Base movible en la seccion de links.
   - Joints de ruedas configurados para velocidad, especialmente `wheel_left_joint` y `wheel_right_joint`.
4. Define una carpeta de salida para el USD si quieres controlar donde queda guardado.
5. Presiona `Import`.

Despues de importar:

1. Selecciona el prim principal del TurtleBot3 en el Stage.
2. Muevelo para dejarlo sobre el piso,ligeramente mas alto que el piso ya que puede generar errores que antes de iniciar la simulacion una parte choque con otra. 
3. Presiona `Play`.
4. Verifica que el robot cae o se estabiliza sobre el suelo.

## Parte 4: Agregar un Lidar con lineas visibles

Para este tutorial usaremos **PhysX SDK Lidar** porque tiene la propiedad `drawLines`, que es exactamente lo que queremos visualizar.

Crea el sensor desde el menu:

```text
Create > Sensors > PhysX Lidar > Rotating
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
2. En el panel de propiedades, busca `Raw USD Properties` en la parte del final.
3. Activa:

```text
drawLines
```

4. Ajusta `rotationRate`:
   - `1.0` para verlo girando a 1 Hz.
   - `0.0` para disparar rayos en todas las direcciones al mismo tiempo, util para depurar.

Presiona `Play`. Deberias ver lineas saliendo desde el Lidar hacia paredes, suelo u obstaculos. Si no ves nada, revisa que el entorno tenga colisiones y que el sensor no este dentro de otra geometria.

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

## Parte 6: Generar un Occupancy Map

Abre la herramienta de mapa:

```text
Tools > Robotics > Occupancy Map
```
La herramienta calcula un mapa 2D usando la geometria con colision del Stage. No necesita que el robot conduzca ni que el Lidar genere datos.

### Configurar el mapa

En la ventana `Occupancy Map`:

1. Coloca el `Origin` en una zona libre, no dentro de una pared, mesa, robot u obstaculo.
2. Ajusta el eje Z del origen a una altura razonable para mapear el entorno. Preferiblemente la altura del Lidar  :
3. Preciona Bound selection
4. Presiona `CALCULATE`.
5. Presiona `VISUALIZE IMAGE`.
6. Guarda la imagen del mapa desde la ventana de visualizacion.

## Para que sirve el Occupancy Map

Un Occupancy Map convierte una escena 3D en una representacion 2D navegable. En robotica sirve para:

- Planificacion global: calcular una ruta desde A hasta B evitando paredes y obstaculos fijos.
- Localizacion: comparar sensores del robot contra un mapa conocido.
- Simulacion de navegacion: probar algoritmos antes de correrlos en un robot real.
- Validacion de entornos: revisar si una escena tiene pasillos, paredes y zonas libres bien definidas.
- Preparacion para Nav2: usar el mapa como entrada para navegacion autonoma en ROS 2.

En un robot real, muchas veces el mapa se construye con SLAM usando Lidar. En Isaac Sim, tambien puedes generar el mapa directamente desde la geometria del Stage para acelerar pruebas.

