# Tutorial 01: Uso basico de Isaac Sim

Fuente oficial de referencia: [Isaac Sim Basic Usage Tutorial, NVIDIA Isaac Sim 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/introduction/quickstart_isaacsim.html).


## Objetivo

El objetivo de este tutorial es dar una introudcción sencilla a el programa isaac-sim y a su menu principal asi como algunas opciones del mismo. 

## Idea central

Isaac Sim permite construir una simulacion de varias formas. En este primer recorrido vamos a hacer lo mismo por tres caminos:

- GUI: usando menus y paneles visuales.
- Script Editor: ejecutando Python dentro de Isaac Sim.
- Standalone Python: corriendo un script desde la carpeta de instalacion.

Los tres caminos apuntan al mismo resultado: una escena simple con suelo, luz y objetos, donde uno de los cubos puede caer por gravedad e interactuar con el plano.

## Flujo 1: GUI

### 1. Abrir Isaac Sim

Desde la carpeta de instalacion, abre el App Selector.

Linux:

```bash
cd ~/isaacsim
./isaac-sim.selector.sh
```

Crea una escena limpia desde `File > New`. 

### 2. Agregar un plano de suelo

El plano sirve como superficie para que los objetos con fisica tengan donde caer o colisionar.

En la barra superior, usa:

```text
Create > Physics > Ground Plane
```

Despues de crearlo, deberias verlo en el Stage como un objeto dentro de `/World`.

### 3. Agregar una luz

Una escena puede existir sin luz, pero es dificil verla. Para iluminarla, crea una luz direccional:

```text
Create > Lights > Distant Light
```

revisa que haya objetos visibles en la escena y que la vista este apuntando hacia ellos.

### 4. Agrega lo que quieras

Crea lo que quieras desde:

```text
Create > Shape > elije
```

Presiona `Play`. lo que creaste no deberia caer todavia. Esto es normal: algo creado como geometria visual no tiene masa, cuerpo rigido ni colision configurada automaticamente.

### 5. Mover, rotar y escalar el cubo

Selecciona el cubo y prueba los gizmos de transformacion:

- `W`: mover.
- `E`: rotar.
- `R`: escalar.
- `Esc`: quitar seleccion.

Tambien puedes cambiar valores exactos desde el panel de propiedades. Eso es util cuando quieres repetir una posicion o dejar una escena ordenada.

### 6. Agregar fisica y colisiones

Para que el cubo responda a la gravedad y choque con el suelo:

1. Selecciona el cubo en el Stage, normalmente `/World/lo_que_crearon`.
2. En el panel de propiedades, busca `Add`.
3. En las opciones de fisica, agrega un preset de cuerpo rigido con colisionadores.
4. Presiona `Play`.

Resultado esperado: el cubo cae por gravedad y se detiene al tocar el plano.

## Flujo 2: Script Editor

Este camino hace la misma escena, pero usando Python dentro de Isaac Sim. Abre:

```text
Window > Script Editor
```

Ejecuta los bloques en pestanas separadas o limpia la escena antes de probar de nuevo.

### 1. Crear el plano

```python
from isaacsim.core.api.objects.ground_plane import GroundPlane

GroundPlane(prim_path="/World/GroundPlane", z_position=0)
```

### 2. Crear una luz direccional

```python
import omni.usd
from pxr import Sdf, UsdLux

stage = omni.usd.get_context().get_stage()
light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight"))
light.CreateIntensityAttr(300)
```

### 3. Crear cubos desde Python

Un cubo visual solo se renderiza; un cubo dinamico ya esta preparado para participar en la simulacion fisica.

```python
import numpy as np
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid

VisualCuboid(
    prim_path="/World/visual_cube",
    name="visual_cube",
    position=np.array([0, 0.5, 0.5]),
    size=0.3,
    color=np.array([255, 255, 0]),
)

DynamicCuboid(
    prim_path="/World/dynamic_cube",
    name="dynamic_cube",
    position=np.array([0, -0.5, 1.0]),
    scale=np.array([0.6, 0.5, 0.2]),
    size=1.0,
    color=np.array([255, 0, 0]),
)
```

Presiona `Play`. El cubo visual permanecera quieto; el cubo dinamico deberia caer y colisionar con el suelo.

### 4. Convertir un objeto existente en objeto con fisica

Si ya tienes un cubo visual y quieres darle comportamiento fisico, puedes agregarle propiedades de cuerpo rigido y colision.

```python
from isaacsim.core.prims import GeometryPrim, RigidPrim

RigidPrim("/World/visual_cube")
prim = GeometryPrim("/World/visual_cube")
prim.apply_collision_apis()
```

Despues de esto, al ejecutar la simulacion, ese cubo tambien deberia responder a la fisica.

### 5. Transformar objetos por codigo

Para ubicar objetos con precision puedes modificar posicion, orientacion o escala desde la API.

```python
import numpy as np
from isaacsim.core.prims import XFormPrim

cube = XFormPrim(prim_paths_expr="/World/dynamic_cube")
cube.set_world_poses(
    positions=np.array([[1.5, 1.2, 1.0]]),
    orientations=np.array([[0.7, 0.7, 0.0, 1.0]]),
)
cube.set_local_scales(np.array([[1.0, 1.5, 0.2]]))
```

Si tu version de Isaac Sim cambia el nombre de alguna clase o metodo, consulta la API instalada o la documentacion oficial de la version que estes usando.

## Flujo 3: Standalone Python

Isaac Sim tambien incluye scripts que se ejecutan desde terminal usando el Python que viene con la instalacion.

Desde la raiz de Isaac Sim:

```bash
./python.sh standalone_examples/tutorials/getting_started.py
```
Este script crea una escena similar usando Python fuera del Script Editor. La ventaja de este flujo es que se parece mas a un proyecto automatizado: puedes versionar scripts, repetir experimentos y conectar tu simulacion con herramientas externas.

## Comparacion rapida de workflows

| Workflow | Cuando usarlo | Ventaja principal |
| --- | --- | --- |
| GUI | Exploracion inicial y aprendizaje visual | Ves de inmediato que cambia en la escena |
| Script Editor | Pruebas rapidas con Python dentro de Isaac Sim | Iteracion rapida sin cerrar el simulador |
| Standalone Python | Automatizacion, experimentos repetibles y proyectos | Mejor para pipelines y codigo versionado |

