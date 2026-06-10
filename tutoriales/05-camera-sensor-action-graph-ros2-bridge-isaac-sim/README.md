# Tutorial 05: Camera Sensor en Carter v1 - Action Graph + ROS2 Bridge

Fuente oficial de referencia:

- [ROS 2 Cameras, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera.html).
- [Camera Sensors, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_camera.html).
- [Robot Assets, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html).
- [Adding Sensors to Carter, NVIDIA Physical AI Learning](https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-sim/latest/ingesting-robot-assets-and-simulating-your-robot-in-isaac-sim/03-adding-sensors.html).

## Objetivo

El objetivo de esta parte es montar una camara RGB sobre un robot tipo Carter v1 en Isaac Sim y publicar sus datos hacia ROS2 usando ROS2 Bridge y Action Graph.

El flujo de trabajo:

- Isaac Sim usa un prim USD de tipo `Camera`.
- La camara se agrega como hija del link del chasis del robot para que se mueva junto con Carter.
- `Isaac Create Render Product` crea el render product asociado a esa camara.
- `ROS2 Camera Helper` publica imagenes como mensajes ROS2.
- `ROS2 Camera Info Helper` publica la informacion intrinseca de la camara.
- ROS2 puede visualizar la imagen con `rqt_image_view` o `rviz2`.



## Parte 1: Preparar la escena

1. Abre Isaac Sim.
2. Crea  prepara una escena.
3. Agrega Carter v1.

La ruta de referencia dentro de los assets de Isaac Sim es:

```text
Robots/NVIDIA/Carter/carter_v1.usd
```
4. Presiona `Play` brevemente para confirmar que Carter queda estable sobre el piso.


## Parte 2: Crear y montar la camara en Carter

Crea una camara desde el menu:

```text
Create > Camera
```

Renombra el prim como:

```text
RGB_Sensor
```

En el `Stage`, arrastra `RGB_Sensor` dentro del link del chasis y luego dentro de camera_mount.

La jerarquia debe quedar parecida a:

```text
/World/Carter/chassis_link/camera_mount/RGB_Sensor
```
Montar la camara como hija de `chassis_link` es lo que hace que se mueva junto con el robot durante la simulacion.

Para revisar lo que ve la camara:

1. En el viewport, abre el menu de camaras.
2. Selecciona `RGB_Sensor`.
3. Presiona `Play` y mueve Carter si ya tienes control configurado.
4. Confirma que la camara se desplaza junto con el robot.

## Parte 3: Crear el Action Graph para RGB

Abre:

```text
Tools > robotics > Ros2 Omnigraphs > camera
```
Selecciona add en camera prim y agrega la camara deseada. 
Deberia tener esta configuracion incial. 
<img width="497" height="598" alt="image" src="https://github.com/user-attachments/assets/df559a63-5316-4601-a033-adf79bed8f57" />

Esto agregara un Action Graph. Tambien se puede seguir el camino de hacerlo a mano creando el Action Graph como se muestra a continuacion.

Crea un Action Graph nuevo y agrega estos nodos:

- `On Playback Tick`
- `ROS2 Context`
- `Isaac Create Render Product`
- `ROS2 Camera Helper`
- `ROS2 Camera Info Helper`

Conecta las senales asi:

| Salida | Entrada |
| --- | --- |
| `On Playback Tick.tick` | `Isaac Create Render Product.execIn` |
| `Isaac Create Render Product.execOut` | `ROS2 Camera Helper.execIn` |
| `Isaac Create Render Product.execOut` | `ROS2 Camera Info Helper.execIn` |
| `Isaac Create Render Product.renderProductPath` | `ROS2 Camera Helper.renderProductPath` |
| `Isaac Create Render Product.renderProductPath` | `ROS2 Camera Info Helper.renderProductPath` |
| `ROS2 Context.context` | `ROS2 Camera Helper.context` |
| `ROS2 Context.context` | `ROS2 Camera Info Helper.context` |


El flujo completo queda:

```text
On Playback Tick
  -> Isaac Create Render Product
  -> ROS2 Camera Helper
```

El render product es el objeto interno que conecta la camara USD con el pipeline de render. Sin ese render product, el helper ROS2 no tiene imagen que publicar.
Imagen de ejemplo del action Graph 
<img width="1031" height="480" alt="image" src="https://github.com/user-attachments/assets/adfec0ac-f7cb-47fe-9758-3a2d682295a9" /> <img width="706" height="485" alt="image" src="https://github.com/user-attachments/assets/43462968-dadb-45e4-a41b-b36f0254022d" />


## Parte 4: Configurar ROS2 Camera Helper para RGB

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

## Parte 5: Publicar camera_info

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

## Parte 8: Probar desde ROS2

Con la simulacion en `Play`, abre una terminal con ROS2 cargado y revisa:

```bash
ros2 topic list
```

Deben aparecer, segun los nodos que hayas creado:

```text
/rgb
/camera_info
```

Revisa el tipo de los topicos:

```bash
ros2 topic /rgb
ros2 topic /camera_info
```

Revisa la frecuencia de publicacion:

```bash
ros2 topic hz /rgb
```

Visualiza la imagen RGB:
en rviz2

```bash
rviz2
```
En Displays en la aprte izquierda abajo oprime en Add > By topic > /rgb > image 
Ahora puedes visualizar la camara de Isaac Sim en rviz2.

Ó puedes usar. 

```bash
ros2 run rqt_image_view rqt_image_view /carter/front_camera/rgb
```
- La profundidad se ve casi blanca o negra: ajusta la pose de la camara para limitar el rango visible de profundidad.
- La publicacion es lenta: baja `width` y `height`, aumenta `frameSkipCount` o publica menos tipos de datos.
- RViz2 no muestra la imagen: prueba primero con `rqt_image_view` y revisa el `frameId`.
