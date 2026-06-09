# Tutorial 05: Camera Sensor in Isaac Sim - Action Graph + ROS2 Bridge

Fuente oficial de referencia:

- [ROS 2 Cameras, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera.html).
- [Camera Sensors, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_camera.html).

## Objetivo

El objetivo de esta parte es crear una camara en Isaac Sim y publicar sus datos hacia ROS2 usando ROS2 Bridge y Action Graph.

La idea central es:

- Isaac Sim usa un prim USD de tipo `Camera`.
- `Isaac Create Render Product` crea el render product asociado a esa camara.
- `ROS2 Camera Helper` publica imagenes como mensajes ROS2.
- `ROS2 Camera Info Helper` publica la informacion intrinseca de la camara.
- ROS2 puede visualizar la imagen con `rqt_image_view` o `rviz2`.

## Requisitos

- Tener ROS2 instalado y con el entorno cargado antes de abrir Isaac Sim.
- Tener habilitada la extension de ROS2 Bridge en Isaac Sim.
- Tener una escena simple con objetos visibles o un robot ya importado.
- Confirmar que Isaac Sim y ROS2 usan el mismo `ROS_DOMAIN_ID`.
- Tener herramientas de visualizacion de imagenes en ROS2.

Para instalar herramientas utiles:

```bash
sudo apt install ros-$ROS_DISTRO-rqt-image-view ros-$ROS_DISTRO-image-tools
```

## Parte 1: Preparar la escena

1. Abre Isaac Sim desde una terminal donde ya tengas cargado ROS2.
2. Crea una escena nueva o abre una escena existente del repositorio.
3. Agrega un piso con:

```text
Create > Environments > Flat Grid
```

4. Agrega un objeto visible para probar la camara:

```text
Create > Shape > Cube
```

5. Mueve el cubo frente a la zona donde quedara la camara.
6. Presiona `Play` brevemente para confirmar que la escena no lanza errores.

No uses la camara `Perspective` del viewport como si fuera un sensor. Para publicar datos por ROS2 conviene crear un prim `Camera` dentro del `Stage`.

## Parte 2: Crear y ubicar la camara

Crea una camara desde el menu:

```text
Create > Camera
```

Renombra el prim como:

```text
Camera_1
```

Una ruta simple para este tutorial es:

```text
/World/Camera_1
```

Configura una pose inicial parecida a esta:

| Campo | Valor |
| --- | --- |
| `Translate X` | `1.5` |
| `Translate Y` | `0.0` |
| `Translate Z` | `1.0` |
| `Rotate X` | `60.0` |
| `Rotate Y` | `0.0` |
| `Rotate Z` | `90.0` |

Los valores exactos dependen de tu escena. Lo importante es que la camara apunte a objetos visibles.

Para revisar lo que ve la camara:

1. En el viewport, abre el menu de camaras.
2. Selecciona `Camera_1`.
3. Ajusta posicion y rotacion hasta que el cubo o el robot queden dentro del encuadre.

Si quieres montar la camara sobre un robot, arrastra `Camera_1` dentro del prim del robot o del link donde deba quedar fijada. Luego revisa que la ruta final del prim sea la que uses en el Action Graph.

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
| `cameraPrim` | `/World/Camera_1` |
| `enabled` | `True` |
| `width` | `640` |
| `height` | `480` |

Puedes usar `1280 x 720`, pero para pruebas iniciales `640 x 480` reduce carga de GPU y ancho de banda ROS2.

## Parte 5: Configurar ROS2 Camera Helper para RGB

Selecciona el nodo `ROS2 Camera Helper` y configura:

| Campo | Valor |
| --- | --- |
| `type` | `rgb` |
| `topicName` | `rgb` |
| `nodeNamespace` | `camera` |
| `frameId` | `camera_link` |
| `queueSize` | `10` |
| `frameSkipCount` | `0` |
| `enabled` | `True` |

Con esta configuracion, el topico publicado queda:

```text
/camera/rgb
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
| `nodeNamespace` | `camera` |
| `frameId` | `camera_link` |
| `queueSize` | `10` |
| `frameSkipCount` | `0` |
| `enabled` | `True` |

El topico publicado queda:

```text
/camera/camera_info
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
| `nodeNamespace` | `camera` |
| `frameId` | `camera_link` |

El topico queda:

```text
/camera/depth
```

Para nube de puntos, usa otro helper con:

| Campo | Valor |
| --- | --- |
| `type` | `depth_pcl` |
| `topicName` | `points` |
| `nodeNamespace` | `camera` |
| `frameId` | `camera_link` |

El topico queda:

```text
/camera/points
```

El `ROS2 Camera Helper` solo publica un tipo de dato por nodo. Si ya ejecutaste la simulacion y cambiaste el campo `type`, puede que el pipeline interno no se regenere correctamente. En ese caso crea un helper nuevo o recarga la escena.

## Parte 8: Probar desde ROS2

Con la simulacion en `Play`, abre una terminal con ROS2 cargado y revisa:

```bash
ros2 topic list
```

Deben aparecer, segun los nodos que hayas creado:

```text
/camera/rgb
/camera/camera_info
/camera/depth
/camera/points
```

Revisa el tipo de los topicos:

```bash
ros2 topic info /camera/rgb
ros2 topic info /camera/camera_info
```

Revisa la frecuencia de publicacion:

```bash
ros2 topic hz /camera/rgb
```

Visualiza la imagen RGB:

```bash
ros2 run rqt_image_view rqt_image_view /camera/rgb
```

Visualiza la profundidad:

```bash
ros2 run rqt_image_view rqt_image_view /camera/depth
```

Tambien puedes usar RViz2:

```bash
rviz2
```

En RViz2:

1. Agrega un display de tipo `Image`.
2. Selecciona `/camera/rgb`.
3. Agrega otro display de tipo `Camera` si tambien estas publicando `/camera/camera_info`.

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
- La imagen sale congelada: revisa que `On Playback Tick` este conectado y que el render product este habilitado.
- `/camera/camera_info` no aparece: agrega `ROS2 Camera Info Helper`; el helper RGB no reemplaza ese nodo.
- Cambiaste `type` y no cambio el topico: crea un nuevo `ROS2 Camera Helper` o recarga la escena.
- La profundidad se ve casi blanca o negra: ajusta la pose de la camara para limitar el rango visible de profundidad.
- La publicacion es lenta: baja `width` y `height`, aumenta `frameSkipCount` o publica menos tipos de datos.
- RViz2 no muestra la imagen: prueba primero con `rqt_image_view` y revisa el `frameId`.
