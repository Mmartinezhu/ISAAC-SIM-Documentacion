# Tutorial 05: Camera Sensor en Carter v1 - Action Graph + ROS2 Bridge

Fuente oficial de referencia:

- [ROS 2 Cameras, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera.html).
- [Camera Sensors, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_camera.html).
- [Robot Assets, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html).
- [Adding Sensors to Carter, NVIDIA Physical AI Learning](https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-sim/latest/ingesting-robot-assets-and-simulating-your-robot-in-isaac-sim/03-adding-sensors.html).

## Objetivo

El objetivo de esta parte es montar una camara RGB sobre un robot tipo Carter v1 en Isaac Sim y publicar sus datos hacia ROS2 usando ROS2 Bridge y Action Graph.

La idea central es:

- Isaac Sim usa un prim USD de tipo `Camera`.
- La camara se agrega como hija del link del chasis del robot para que se mueva junto con Carter.
- `Isaac Create Render Product` crea el render product asociado a esa camara.
- `ROS2 Camera Helper` publica imagenes como mensajes ROS2.
- `ROS2 Camera Info Helper` publica la informacion intrinseca de la camara.
- ROS2 puede visualizar la imagen con `rqt_image_view` o `rviz2`.

## Requisitos

- Tener ROS2 instalado y con el entorno cargado antes de abrir Isaac Sim.
- Tener habilitada la extension de ROS2 Bridge en Isaac Sim.
- Tener disponible el asset Carter v1 de Isaac Sim.
- Confirmar que Isaac Sim y ROS2 usan el mismo `ROS_DOMAIN_ID`.
- Tener herramientas de visualizacion de imagenes en ROS2.

Para instalar herramientas utiles:

```bash
sudo apt install ros-$ROS_DISTRO-rqt-image-view ros-$ROS_DISTRO-image-tools
```

## Parte 1: Preparar la escena

1. Abre Isaac Sim desde una terminal donde ya tengas cargado ROS2.
2. Crea una escena nueva.
3. Agrega un piso con:

```text
Create > Environments > Flat Grid
```

4. Agrega Carter v1 desde el `Content Browser`.

Puedes buscar el asset por nombre:

```text
carter_v1.usd
```

La ruta de referencia dentro de los assets de Isaac Sim es:

```text
Robots/NVIDIA/Carter/carter_v1.usd
```

5. Arrastra el robot al `Stage`.
6. Renombra el prim raiz como:

```text
Carter
```

7. Deja el robot sobre el piso, por ejemplo cerca de:

```text
Translate = 0, 0, 0
```

8. Agrega objetos visibles frente al robot para probar la camara:

```text
Create > Shape > Cube
```

9. Mueve el cubo frente a Carter, por ejemplo:

```text
Translate = 2.0, 0.0, 0.5
Scale = 0.4, 0.4, 0.4
```

10. Presiona `Play` brevemente para confirmar que Carter queda estable sobre el piso.

No uses la camara `Perspective` del viewport como si fuera un sensor. Para publicar datos por ROS2 conviene crear un prim `Camera` dentro del `Stage`.

## Parte 2: Crear y montar la camara en Carter

Crea una camara desde el menu:

```text
Create > Camera
```

Renombra el prim como:

```text
RGB_Sensor
```

En el `Stage`, arrastra `RGB_Sensor` dentro del link del chasis de Carter.

La jerarquia debe quedar parecida a:

```text
/World/Carter/chassis_link/RGB_Sensor
```

En algunos assets el nombre del link puede aparecer como `Chassis_link` o con otra capitalizacion. Usa el nombre exacto que veas en el `Stage`.

Montar la camara como hija de `chassis_link` es lo que hace que se mueva junto con el robot durante la simulacion.

Configura una pose local inicial parecida a esta:

| Campo | Valor |
| --- | --- |
| `Translate X` | `0.1` |
| `Translate Y` | `0.0` |
| `Translate Z` | `0.33` |
| `Rotate X` | `90.0` |
| `Rotate Y` | `-90.0` |
| `Rotate Z` | `90.0` |

Estos valores ubican la camara en la parte frontal superior de Carter. Ajustalos si tu robot tiene otra orientacion o si el encuadre queda invertido.

Para revisar lo que ve la camara:

1. En el viewport, abre el menu de camaras.
2. Selecciona `RGB_Sensor`.
3. Ajusta posicion y rotacion hasta que el cubo quede dentro del encuadre.
4. Presiona `Play` y mueve Carter si ya tienes control configurado.
5. Confirma que la camara se desplaza junto con el robot.

Si la camara se queda quieta mientras Carter se mueve, probablemente quedo fuera de `chassis_link` o dentro de otro prim que no sigue al robot.

## Parte 3: Crear el Action Graph para RGB

Abre:

```text
Window > Graph Editors > Action Graph
```

Crea un Action Graph nuevo y agrega estos nodos:

- `On Playback Tick`
- `ROS2 Context`
- `Isaac Create Render Product`
- `ROS2 Camera Helper`

Conecta las senales asi:

| Salida | Entrada |
| --- | --- |
| `On Playback Tick.tick` | `Isaac Create Render Product.execIn` |
| `Isaac Create Render Product.execOut` | `ROS2 Camera Helper.execIn` |
| `Isaac Create Render Product.renderProductPath` | `ROS2 Camera Helper.renderProductPath` |
| `ROS2 Context.context` | `ROS2 Camera Helper.context` |

El flujo completo queda:

```text
On Playback Tick
  -> Isaac Create Render Product
  -> ROS2 Camera Helper
```

El render product es el objeto interno que conecta la camara USD con el pipeline de render. Sin ese render product, el helper ROS2 no tiene imagen que publicar.

## Parte 4: Configurar Isaac Create Render Product

Selecciona el nodo `Isaac Create Render Product` y configura:

| Campo | Valor |
| --- | --- |
| `cameraPrim` | `/World/Carter/chassis_link/RGB_Sensor` |
| `enabled` | `True` |
| `width` | `640` |
| `height` | `480` |

Puedes usar `1280 x 720`, pero para pruebas iniciales `640 x 480` reduce carga de GPU y ancho de banda ROS2.

Si tu jerarquia usa otro nombre, por ejemplo `/World/Carter/Chassis_link/RGB_Sensor`, usa esa ruta exacta en `cameraPrim`.

## Parte 5: Configurar ROS2 Camera Helper para RGB

Selecciona el nodo `ROS2 Camera Helper` y configura:

| Campo | Valor |
| --- | --- |
| `type` | `rgb` |
| `topicName` | `rgb` |
| `nodeNamespace` | `carter/front_camera` |
| `frameId` | `carter_front_camera_link` |
| `queueSize` | `10` |
| `frameSkipCount` | `0` |
| `enabled` | `True` |

Con esta configuracion, el topico publicado queda:

```text
/carter/front_camera/rgb
```

El tipo de mensaje es:

```text
sensor_msgs/msg/Image
```

Si no usas `nodeNamespace`, el topico quedara como:

```text
/rgb
```

## Parte 6: Publicar camera_info

Para que ROS2 conozca los parametros intrinsecos de la camara, agrega otro nodo:

```text
ROS2 Camera Info Helper
```

Conecta:

| Salida | Entrada |
| --- | --- |
| `Isaac Create Render Product.execOut` | `ROS2 Camera Info Helper.execIn` |
| `Isaac Create Render Product.renderProductPath` | `ROS2 Camera Info Helper.renderProductPath` |
| `ROS2 Context.context` | `ROS2 Camera Info Helper.context` |

Configura el nodo:

| Campo | Valor |
| --- | --- |
| `topicName` | `camera_info` |
| `nodeNamespace` | `carter/front_camera` |
| `frameId` | `carter_front_camera_link` |
| `queueSize` | `10` |
| `frameSkipCount` | `0` |
| `enabled` | `True` |

El topico publicado queda:

```text
/carter/front_camera/camera_info
```

El tipo de mensaje es:

```text
sensor_msgs/msg/CameraInfo
```

## Parte 7: Publicar depth o point cloud

Para publicar profundidad, agrega un segundo `ROS2 Camera Helper` conectado al mismo render product.

Configuralo asi:

| Campo | Valor |
| --- | --- |
| `type` | `depth` |
| `topicName` | `depth` |
| `nodeNamespace` | `carter/front_camera` |
| `frameId` | `carter_front_camera_link` |

El topico queda:

```text
/carter/front_camera/depth
```

Para nube de puntos, usa otro helper con:

| Campo | Valor |
| --- | --- |
| `type` | `depth_pcl` |
| `topicName` | `points` |
| `nodeNamespace` | `carter/front_camera` |
| `frameId` | `carter_front_camera_link` |

El topico queda:

```text
/carter/front_camera/points
```

El `ROS2 Camera Helper` solo publica un tipo de dato por nodo. Si ya ejecutaste la simulacion y cambiaste el campo `type`, puede que el pipeline interno no se regenere correctamente. En ese caso crea un helper nuevo o recarga la escena.

## Parte 8: Probar desde ROS2

Con la simulacion en `Play`, abre una terminal con ROS2 cargado y revisa:

```bash
ros2 topic list
```

Deben aparecer, segun los nodos que hayas creado:

```text
/carter/front_camera/rgb
/carter/front_camera/camera_info
/carter/front_camera/depth
/carter/front_camera/points
```

Revisa el tipo de los topicos:

```bash
ros2 topic info /carter/front_camera/rgb
ros2 topic info /carter/front_camera/camera_info
```

Revisa la frecuencia de publicacion:

```bash
ros2 topic hz /carter/front_camera/rgb
```

Visualiza la imagen RGB:

```bash
ros2 run rqt_image_view rqt_image_view /carter/front_camera/rgb
```

Visualiza la profundidad:

```bash
ros2 run rqt_image_view rqt_image_view /carter/front_camera/depth
```

Tambien puedes usar RViz2:

```bash
rviz2
```

En RViz2:

1. Agrega un display de tipo `Image`.
2. Selecciona `/carter/front_camera/rgb`.
3. Agrega otro display de tipo `Camera` si tambien estas publicando `/carter/front_camera/camera_info`.

Si RViz2 pide un frame fijo y no tienes TF publicado, usa primero `rqt_image_view` para validar la imagen. El campo `frameId` del mensaje no publica automaticamente un arbol TF.

## Parte 9: Crear el grafo con el atajo de Isaac Sim

Isaac Sim tambien puede crear estos nodos desde un asistente:

```text
Tools > Robotics > ROS 2 OmniGraphs > Camera
```

El asistente pide:

- `Graph Path`
- `Camera Prim`
- `frameId`
- `Node Namespace`
- tipos de datos a publicar, por ejemplo RGB, depth o camera info

Si no aparece el menu de ROS2, revisa que la extension de ROS2 Bridge este habilitada.

Este atajo es util para comparar tu Action Graph manual con un grafo generado automaticamente.

## Errores comunes

- No aparecen topicos: revisa que Isaac Sim se haya abierto desde una terminal con ROS2 cargado.
- No aparecen topicos: confirma que ROS2 Bridge este habilitado y que la simulacion este en `Play`.
- Isaac Sim y ROS2 no se ven: revisa que ambos usen el mismo `ROS_DOMAIN_ID`.
- La imagen sale negra: revisa que la camara apunte a objetos visibles y que la escena tenga iluminacion.
- La camara no sigue al robot: revisa que `RGB_Sensor` sea hijo de `chassis_link`.
- La imagen sale congelada: revisa que `On Playback Tick` este conectado y que el render product este habilitado.
- `/carter/front_camera/camera_info` no aparece: agrega `ROS2 Camera Info Helper`; el helper RGB no reemplaza ese nodo.
- Cambiaste `type` y no cambio el topico: crea un nuevo `ROS2 Camera Helper` o recarga la escena.
- La profundidad se ve casi blanca o negra: ajusta la pose de la camara para limitar el rango visible de profundidad.
- La publicacion es lenta: baja `width` y `height`, aumenta `frameSkipCount` o publica menos tipos de datos.
- RViz2 no muestra la imagen: prueba primero con `rqt_image_view` y revisa el `frameId`.
