# Parte 1: Simulacion del rover ROBERT en Isaac Sim

De un modelo de Fusion 360 a un USD que se puede teleoperar por ROS2 y que sirve de base para entrenar con Isaac Lab.

Scripts de apoyo en [scripts/](scripts/). Todos se ejecutan en el Script Editor de Isaac Sim (`Window > Script Editor`) con la simulacion en **Stop**, salvo que se indique lo contrario.

## Objetivo

Tener el rover simulado con fidelidad suficiente para que lo que se aprenda en simulacion sirva en el robot real. Eso significa:

- Suspension rocker-bogie funcional, con su lazo cerrado.
- Motores con el par y la velocidad reales.
- Ruedas con el material y la geometria de contacto reales.
- Control de direccion independiente por rueda, igual que el que llevara el robot fisico.

## Requisitos

- Fusion 360 con el modelo del rover.
- Exportador URDF para Fusion (`URDF_Exporter`, basado en `fusion2urdf`).
- Isaac Sim 5.x con el bridge ROS2 activado.
- Terminal ROS2 (`./terminal_b.bash`).

## Resumen del flujo

```text
Fusion 360
   |  exportador URDF (ball joints -> revolute)
   v
URDF + mallas STL
   |  URDF Importer de Isaac Sim
   v
USD con joints revolute
   |  scripts en Isaac Sim:
   |   - convertir a esfericos y cerrar el lazo
   |   - drives, masas, material, colision
   |   - corregir piezas giradas
   v
USD del robot configurado
   |  Action Graph con control ICR
   v
robert_teleop.usd (con graph)    robert_RL.usd (sin graph, para Isaac Lab)
```

Se guardan dos USD al final porque tienen usos distintos. El de teleoperacion lleva el Action Graph y los materiales de escena; el de RL debe contener **solo el robot**, porque Isaac Lab lo replica miles de veces y cualquier cosa extra (suelo, cubos, grafos ROS2) se replica con el.

## Parte 1: Exportar desde Fusion 360

### 1.1 Joints esfericos

URDF no tiene joints esfericos. El exportador original falla o genera `type="Ball"`, que no es valido. Hay dos opciones:

1. **Exportar los joints de bola como revolute** desde Fusion y convertirlos a esfericos despues en Isaac Sim. Es lo que se hizo aqui y es lo recomendado, porque Isaac Sim tiene `PhysicsSphericalJoint` nativo.
2. Usar el exportador modificado, que descompone cada bola en tres revolute (roll, pitch, yaw) con dos eslabones ficticios sin masa. Sirve para RViz y Gazebo, que no tienen esfericos, pero en Isaac Sim mete seis joints donde bastan dos.

### 1.2 Lazo cerrado

URDF exige un arbol: cada eslabon tiene exactamente un padre. El rocker-bogie tiene dos lazos, asi que uno de los joints de cada lazo **no puede ir en el URDF**. Se exporta el arbol y el joint que falta se crea a mano en Isaac Sim (Parte 3.2).

Al exportar, hay que decidir cual de los joints del lazo se omite. Conviene omitir el que une el tirante con el hombro (`union_sus_*_01`), de modo que el tirante quede colgando del diferencial en el arbol.

### 1.3 Componentes temporales

Si una exportacion anterior fallo a medias, el diseño puede quedar con componentes llamados `old_component*`. El exportador los detecta y se niega a continuar. Deshacer esa ejecucion o reabrir una copia limpia del diseño.

## Parte 2: Importar el URDF en Isaac Sim

Abrir `Isaac Utils > Robot Importer > URDF` y cargar el archivo. Opciones relevantes:

| Opcion | Valor | Motivo |
| --- | --- | --- |
| Fix Base Link | desactivado | El rover es base flotante |
| Create Physics Scene | activado | |
| Collision From Visuals | activado | Las mallas de colision se generan desde las visuales |
| Instanceable Assets | (ver nota) | |

Nota sobre *instanceable*: el importador puede marcar las mallas como instanciables para ahorrar memoria. Eso las vuelve **de solo lectura** y no se les puede asignar material ni cambiar la aproximacion de colision. Mas adelante (Parte 8) se desinstancian solo las llantas.

Tras importar, el robot queda en `/World/cuerpo_suspension` con los joints en `/World/cuerpo_suspension/joints/`.

## Parte 3: Joints esfericos y cierre del lazo

### 3.1 Convertir revolute a esferico

Los joints que en Fusion eran de bola llegan como `PhysicsRevoluteJoint`. Se cambia el tipo del prim:

```python
from pxr import Usd
stage = omni.usd.get_context().get_stage()
joint_path = "/World/cuerpo_suspension/joints/union_sus_der"
joint_prim = stage.GetPrimAtPath(joint_path)
if joint_prim:
    joint_prim.SetTypeName("PhysicsSphericalJoint")
    print("cambiado a Spherical Joint")
```

Repetir para cada joint de bola. Los prims conservan sus `body0`, `body1` y frames locales; solo cambia el tipo.

### 3.2 Crear los joints que cierran el lazo

Los joints que no cupieron en el URDF se crean a mano. La forma mas fiable en la interfaz es:

1. Seleccionar un joint esferico que ya funcione.
2. `Ctrl+D` para duplicarlo.
3. En el Property Panel, cambiar `physics:body0` y `physics:body1` a los dos cuerpos que debe unir.
4. Ajustar `Local Position 0/1` para que el pivote quede donde corresponde. Los esfericos se dibujan como una esfera roja; sirve para comprobar la alineacion.

Por script tambien se puede, pero al crear el prim desde cero hay que definir los frames locales a mano:

```python
from pxr import Usd
stage = omni.usd.get_context().get_stage()
joint_path = "/World/cuerpo_suspension/joints/union_sus_der_01"
joint_prim = stage.DefinePrim(joint_path, "PhysicsSphericalJoint")
joint_prim.CreateRelationship("physics:body0").AddTarget("/World/cuerpo_suspension/hombro_der_1")
joint_prim.CreateRelationship("physics:body1").AddTarget("/World/cuerpo_suspension/union_der_sus_1")
```

Despues de esto el modelo tiene sus dos lazos y **Isaac Sim se queja al dar Play**:

```text
[Error] [omni.physx.plugin] RigidBody (...) appears to be a part of a closed articulation, which is not supported, please exclude one of the joints from the articulation
```

### 3.3 Excluir un joint por lazo de la articulacion

Esta es la parte conceptual mas importante de todo el proyecto.

PhysX simula el robot como una **articulacion** (reduced coordinates): un arbol de cuerpos resuelto de forma exacta y estable. Un arbol no puede tener lazos. Pero PhysX tambien tiene joints normales (maximal coordinates) que se resuelven con el solver de cuerpos rigidos, y esos **si pueden cerrar lazos** sobre una articulacion. Es lo que documenta NVIDIA en [Rigging closed loop structures](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/rig_closed_loop_structures.html).

La operacion se llama **Exclude From Articulation**, y hay dos cosas que conviene tener claras:

- **No elimina el joint.** Sigue ahi, sigue transmitiendo fuerza y sigue cerrando el lazo. Solo cambia que lo resuelve el otro solver.
- **No es lo mismo que `purpose = guide`.** Un joint *guide* no transmite fuerza: se limita a seguir a los demas. En un rocker-bogie el lazo debe transmitir par entre ambos lados; con *guide* la suspension deja de funcionar. Esa confusion costo varias horas.

Cuantos joints excluir: uno por cada lazo independiente. Se cuenta con `lazos = joints - cuerpos + 1`. Aqui hay 6 cuerpos en el mecanismo de suspension y 7 joints, asi que dos lazos y dos joints a excluir.

Cual excluir: el que menos interfiera, sin limites, sin drive, que solo sirva de restriccion espacial. Los candidatos son los esfericos del tirante. Se eligen los del lado del hombro (`union_sus_der_01`, `union_sus_izq_01`) para que el tirante, que es ligero, quede colgando de la rama corta del diferencial y no de la rama larga y pesada del hombro.

Lo que **no** hay que hacer: excluir el joint central `suspension`. Esta en ambos lazos, pero excluirlo solo rompe uno de los dos (la cuenta baja de 2 a 1) y ademas es el pivote del diferencial, el joint con mas carga del mecanismo. Meterle error de solver ahi es lo peor que se puede hacer.

Script [scripts/01_configurar_joints.py](scripts/01_configurar_joints.py) lo aplica junto con todo lo demas de la Parte 5. Solo la exclusion:

```python
from pxr import UsdPhysics, Sdf
stage = omni.usd.get_context().get_stage()
for prim in stage.Traverse():
    if prim.GetName() in ["union_sus_der_01", "union_sus_izq_01"]:
        prim.CreateAttribute("physics:excludeFromArticulation", Sdf.ValueTypeNames.Bool).Set(True)
        print("excluido: " + prim.GetName())
```

### 3.4 Eje X en los esfericos

Al convertir un revolute en esferico se conserva su `physics:axis` (Y o Z), y PhysX avisa:

```text
Using USD spherical joints with any axis except x is not currently supported.
```

En un esferico el eje solo orienta el cono de limites. Como estos no tienen limites, se pone X sin mas consecuencia:

```python
for prim in stage.Traverse():
    if prim.GetName().startswith("union_sus"):
        a = prim.GetAttribute("physics:axis")
        if a:
            a.Set("X")
```

### 3.5 Verificar

Con la simulacion en Play, listar los grados de libertad de la articulacion:

```python
from isaacsim.core.prims import SingleArticulation
art = SingleArticulation("/World/cuerpo_suspension/base_link")
art.initialize()
for n in art.dof_names:
    print(repr(n))
```

Deben aparecer `union_sus_der:0/1/2` y `union_sus_izq:0/1/2` (los esfericos dentro de la articulacion, con tres grados cada uno) y **no** deben aparecer `union_sus_der_01` ni `union_sus_izq_01`. Que no esten en la lista es la prueba de que quedaron fuera de la articulacion y la resuelve el otro solver.

## Parte 4: Articulation Root

El importador pone el `ArticulationRootAPI` en `base_link`. NVIDIA recomienda ponerlo en el Xform raiz del robot para base flotante, pero eso importa cuando hay ambiguedad sobre cual es el eslabon raiz. Aqui `base_link` es inequivocamente la raiz (todo cuelga de el), y funciona igual.

Si se mueve al Xform, hay que actualizar el `Robot Path` de todos los `Articulation Controller`. No se movio.

## Parte 5: Drives de los joints

Un drive es el actuador simulado de cada joint. Los importados desde URDF traen valores automaticos que no corresponden a nada real y hay que sustituirlos. Hay tres grupos con configuraciones opuestas.

### 5.1 Joints pasivos: liberarlos

La suspension es pasiva en el robot real. Sus joints no deben tener stiffness (que los clavaria en una posicion), ni friccion, ni armature:

| Joint | stiffness | damping | friccion | armature |
| --- | --- | --- | --- | --- |
| `suspension` | 0 | 2 a 3 | 0 | 0 |
| `hombro_der`, `hombro_izq` | 0 | 5 a 8 | 0 | 0 |
| `codo_der`, `codo_izq` | 0 | 5 a 8 | 0 | 0 |
| `union_sus_*` (4 esfericos) | 0 | 0 | 0 | 0 |

El damping en hombros y codos no modela ninguna pieza fisica (un rocker-bogie no lleva amortiguadores), sirve solo para que la simulacion no oscile eternamente. Se calibra soltando el rover desde unos centimetros: un rebote y quieto es el punto. Cuando se bajo la masa de las ruedas hubo que bajarlo tambien, porque con menos peso la suspension tardaba en extenderse.

Los esfericos del tirante van a cero: son restriccion espacial, no absorben energia, y meterles damping solo añade ruido.

Un drive con `stiffness = 0`, `damping = D` y `targetVelocity = 0` es un amortiguador viscoso puro: no empuja hacia ninguna posicion, solo se resiste a la velocidad.

### 5.2 Ruedas: la fisica del motor

Se parte de dos datos del motor real y del rover:

| Dato | Valor |
| --- | --- |
| Par de bloqueo del motor | 45 kg·cm = 4.41 N·m |
| Velocidad maxima del rover | 10 m/min = 0.167 m/s |
| Radio de rueda | 0.0815 m |

De ahi salen los tres parametros del drive:

- `maxForce` = par de bloqueo = **4.41 N·m**.
- Velocidad angular maxima de la rueda = 0.167 / 0.0815 = **2.045 rad/s** (unas 19.5 rpm).
- `damping` = par de bloqueo / velocidad sin carga = 4.41 / 2.045 = **2.16**.

Esa ultima formula es la clave. En un drive de velocidad, el par entregado es `damping × (velocidad objetivo - velocidad actual)`, recortado por `maxForce`. Con `damping = 2.16` se obtiene exactamente la recta par-velocidad de un motor DC: par maximo parado, cero a la velocidad libre. Con un damping alto (por ejemplo 1000, un valor tipico de ejemplos) el drive vive saturado en `maxForce` y se comporta como un motor que da siempre su par maximo a cualquier velocidad, que no es como funciona ninguno.

| Joint | stiffness | damping | maxForce |
| --- | --- | --- | --- |
| `llanta_*` (6) | 0 | 2.16 | 4.41 |

`stiffness` tiene que ser cero: con stiffness el drive intenta mantener un angulo y la rueda vuelve atras como con un muelle.

### 5.3 Reductores (direccion): drive de posicion

Al reves que las ruedas: quieren mantener un angulo, asi que stiffness alta.

| Joint | stiffness | damping | maxForce |
| --- | --- | --- | --- |
| `reductor_*` (4) | 100000 | 10000 | 10000 |

Estos valores son generosos porque en Isaac Sim el reductor debe responder rapido; en Isaac Lab se usan valores mas bajos (Parte 2 del proyecto).

### 5.4 Limite de giro de los reductores

Los reductores llegan sin limites (`-inf .. inf`). Se limitan a ±90°, que cubre con margen los ±71° que necesita el giro sobre el propio eje. Los limites de USD Physics van en **grados**:

```python
from pxr import UsdPhysics
stage = omni.usd.get_context().get_stage()
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName().startswith("reductor"):
        j = UsdPhysics.RevoluteJoint(prim)
        j.CreateLowerLimitAttr().Set(-90.0)
        j.CreateUpperLimitAttr().Set(90.0)
```

### 5.5 Aplicarlo todo

El script [scripts/01_configurar_joints.py](scripts/01_configurar_joints.py) aplica de una vez: reversion de `purpose=guide` a `default`, exclusion de los dos joints del lazo, eje X en esfericos, liberacion y damping de los pasivos, drives de ruedas y reductores, y limites de ±90°.

Despues, comprobar el estado real con [scripts/04_diagnostico_joints.py](scripts/04_diagnostico_joints.py). Esto es obligatorio y no opcional: los scripts que usan listas de nombres literales fallan **en silencio** cuando un nombre no coincide (por ejemplo `llanta_der_m` en vez de `llanta_der_m_`), y en una ocasion las seis ruedas se quedaron sin configurar durante toda una sesion sin que nada avisara.

## Parte 6: Corregir piezas giradas

Alguna pieza puede llegar de Fusion mirando hacia el lado contrario (aqui, un alma con su llanta). Se corrige rotando el **frame del joint**, no el prim de la pieza: si se rota el Xform del cuerpo, la fisica lo sobrescribe al dar Play porque recalcula la pose desde el joint.

Rotar 180° el frame del lado hijo (`physics:localRot1`) hace que la nueva orientacion **sea el cero del joint**:

```python
from pxr import Gf
stage = omni.usd.get_context().get_stage()
JOINT = "/World/cuerpo_suspension/joints/reductor_izq_d"
EJE = Gf.Vec3f(0.0, 0.0, 1.0)
p = stage.GetPrimAtPath(JOINT)
a = p.GetAttribute("physics:localRot1")
a.Set(a.Get() * Gf.Quatf(0.0, EJE))
```

Un cuaternion de 180° siempre tiene `w = 0` y el eje en la parte imaginaria. Si el eje elegido no es el correcto, ejecutarlo otra vez lo deshace (180° + 180° = 360°) y se prueba con otro.

Dos cosas que pasan y son normales:

- **En Stop no se ve el cambio.** Los prims conservan su transform hasta que PhysX recoloca los cuerpos al dar Play.
- **El joint se dibuja en rojo** hasta que se da Play, porque sus dos frames ya no coinciden. Al arrancar la fisica los alinea.

Si el alma esta bien pero la llanta que cuelga de ella queda mal, se rota tambien el joint de la llanta; cada joint fija su propia orientacion respecto a su padre, no se componen entre si.

## Parte 7: Masas

Las masas vienen del CAD y dependen de los materiales asignados en Fusion. Se revisan con [scripts/04_diagnostico_joints.py](scripts/04_diagnostico_joints.py) (seccion de masas).

Las ruedas llegaron con 1.059 kg cada una, que corresponde a un TPU al 32 % de relleno macizo. Para una rueda impresa con radios lo real son 300 a 500 g. Se ajustaron a **0.3 kg**, escalando la inercia por el mismo factor:

```python
from pxr import UsdPhysics, Gf
stage = omni.usd.get_context().get_stage()
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.GetName().startswith("llanta"):
        m = UsdPhysics.MassAPI.Apply(p)
        vieja = m.GetMassAttr().Get() or 1.059
        f = 0.3 / vieja
        m.CreateMassAttr().Set(0.3)
        i = m.GetDiagonalInertiaAttr()
        if i and i.Get():
            v = i.Get()
            m.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(v[0] * f, v[1] * f, v[2] * f))
```

Escalar solo la masa y no la inercia deja una rueda que pesa poco pero se resiste a girar como si pesara un kilo.

Las ruedas son masa no suspendida, la que golpea los obstaculos, asi que su valor afecta directamente a como responde la suspension. Un aviso sobre lo que **no** hay que hacer: se probo subir la masa de los tirantes `union_*` de 43 g a 500 g para estabilizar la simulacion, y con una inercia 50 veces mayor que la original el mecanismo cambio por completo. Se revirtio. La estabilidad se consigue con el timestep y el solver, no inventando masas.

## Parte 8: Material de las ruedas y colision

### 8.1 Desinstanciar las llantas

Las mallas de colision viven en prototipos instanciados y `stage.Traverse()` no las ve (aparecen cero mallas aunque el rover se vea). Para poder tocarlas:

```python
stage = omni.usd.get_context().get_stage()
for p in stage.Traverse():
    path = str(p.GetPath())
    if p.IsInstanceable() and "llanta" in path and path.endswith("/collisions"):
        p.SetInstanceable(False)
```

Se desinstancian **solo** las colisiones de las llantas. El resto del rover se queda instanciado, que es mas barato.

### 8.2 Material TPU

Las ruedas son de TPU impreso. Valores usados:

| Propiedad | Valor |
| --- | --- |
| Friccion estatica | 0.9 |
| Friccion dinamica | 0.8 |
| Restitucion | 0.3 |
| Modo de combinacion | `multiply` |

Y para el suelo un material `Terreno` con 0.8 / 0.7 / 0.1.

Sobre el modo de combinacion: la friccion resultante depende de **ambos** materiales en contacto. Con `max` mandaria el 0.9 del TPU siempre; con `multiply` sale 0.9 × 0.8 = 0.72, mas realista y mas conservador. Para RL conviene errar hacia poco agarre: si la politica aprende con agarre irreal, patina en el robot de verdad.

### 8.3 Colision SDF en las ruedas

Al dar Play, PhysX avisa de que las mallas de colision no valen para cuerpos dinamicos y cambia a `convexHull`. Para el chasis y los brazos eso esta bien. Para las ruedas no, por dos razones:

- El casco convexo **rellena los huecos entre los tacos**: desaparece el dibujo que da agarre.
- Una rueda aproximada por casco queda ligeramente poligonal y salta al rodar.

Se usa `sdf` (Signed Distance Field), el unico modo de PhysX que soporta geometria concava en cuerpos dinamicos, con resolucion 256.

El script [scripts/02_material_tpu_colision.py](scripts/02_material_tpu_colision.py) hace las tres cosas: desinstancia, crea los dos materiales, los asigna y pone SDF en las llantas.

## Parte 9: Control de direccion por Action Graph

### 9.1 Por que control por ICR y no differential ni Ackermann

El tutorial 03 usa `Differential Controller` (dos ruedas). Con seis ruedas se puede replicar la salida a cada lado, pero eso es *skid steer*: las ruedas delanteras y traseras derrapan en cada giro. Ackermann (tutorial 04) usa las ruedas de direccion pero no permite girar sobre el propio eje.

Lo que usan los rovers de la NASA es **direccion independiente coordinada por ICR** (centro instantaneo de rotacion). Una sola formula produce los tres modos:

| Modo | v | w | Comportamiento |
| --- | --- | --- | --- |
| Ackermann | ≠ 0 | ≠ 0 | Giro en arco |
| Point turn | 0 | ≠ 0 | Gira sobre su eje sin desplazarse |
| Recto | ≠ 0 | 0 | Todas las ruedas rectas |

Para cada rueda en `(x, y)` respecto al centro del rover:

```text
vx = v - w * y
vy = w * x
angulo_direccion = atan2(vy, vx)
velocidad_rueda  = sqrt(vx² + vy²) / R
```

Es la velocidad del punto de contacto como solido rigido. Las ruedas medias tienen `x = 0` en un giro sobre el eje, asi que su angulo sale 0 y no necesitan direccion.

No hay un controlador estandar de ROS2 para esto; se implementa en un Script Node.

### 9.2 Medir la geometria

Las posiciones de las ruedas se miden con [scripts/05_medir_ruedas.py](scripts/05_medir_ruedas.py), que las expresa respecto a `base_link`. Los valores para este rover estan en el README del proyecto. Atencion a dos cosas:

- Medir con la simulacion en **Stop** y sin haber corrido fisica antes, para capturar la pose de diseño y no una postura caida.
- Si se acaba de corregir una pieza girada (Parte 6), la medida en Stop es la **anterior** a la correccion. Usar la posicion de su simetrica.

El radio de rueda sale de la mitad de la dimension mayor del bounding box: 0.0815 m.

### 9.3 Nodos y conexiones

Ocho nodos. Comparado con el tutorial 03, desaparecen `Differential Controller`, `Make Array` y los `Constant Token`; los nombres de joints se escriben directamente en cada `Articulation Controller`.

```text
On Playback Tick --> ROS2 Subscribe Twist --> Script Node --+--> ArtCtrl_Traccion
                            ^                              `--> ArtCtrl_Direccion
                      ROS2 Context
```

Conexiones de ejecucion (cables blancos):

```text
On Playback Tick.Tick          -> ROS2 Subscribe Twist.Exec In
ROS2 Subscribe Twist.Exec Out  -> Script Node.Exec In
Script Node.Exec Out           -> ArtCtrl_Traccion.Exec In
Script Node.Exec Out           -> ArtCtrl_Direccion.Exec In
```

Conexiones de datos:

```text
ROS2 Context.Context                   -> ROS2 Subscribe Twist.Context
ROS2 Subscribe Twist.Linear Velocity   -> Break3Vector_A.Vector
ROS2 Subscribe Twist.Angular Velocity  -> Break3Vector_B.Vector
Break3Vector_A.X                       -> Script Node.linear
Break3Vector_B.Z                       -> Script Node.angular
Script Node.wheelVel                   -> ArtCtrl_Traccion.Velocity Command
Script Node.steerPos                   -> ArtCtrl_Direccion.Position Command
```

Del `Twist` se usa `linear.x` (avance) y `angular.z` (giro). Confundir los ejes es el error mas comun.

Campos:

| Nodo | Campo | Valor |
| --- | --- | --- |
| ROS2 Subscribe Twist | Topic Name | `cmd_vel` |
| ArtCtrl_Traccion | Robot Path | `/World/cuerpo_suspension/base_link` |
| ArtCtrl_Traccion | Joint Names | `llanta_izq_d, llanta_izq_m, llanta_izq_d_, llanta_der_d_, llanta_der_m_, llanta_der_t` |
| ArtCtrl_Direccion | Robot Path | `/World/cuerpo_suspension/base_link` |
| ArtCtrl_Direccion | Joint Names | `reductor_izq_d, reductor_izq_t, reductor_der_d, reductor_der_t_` |

En los `Articulation Controller` se rellena `Robot Path` **o** `Target Prim`, nunca los dos. Si se dejan ambos vacios o ambos llenos, el nodo lanza un warning con el mensaje vacio (`OmniGraph Warning: "`) y no hace nada.

Un warning con solo un nombre entre comillas (`OmniGraph Warning: 'llanta_der_m'`) significa que ese joint **no existe** con ese nombre. Casi siempre es un guion bajo que falta.

### 9.4 Script Node

Puertos a declarar en el nodo (Property Panel, `Add Attribute`):

| Nombre | Tipo | Direccion |
| --- | --- | --- |
| `linear` | double | Input |
| `angular` | double | Input |
| `wheelVel` | double[] | Output |
| `steerPos` | double[] | Output |

Los nombres del codigo y de los puertos tienen que coincidir exactamente. El codigo completo esta en [scripts/script_node_icr.py](scripts/script_node_icr.py). Sus partes:

- `P`: las seis posiciones de rueda, en el orden `izq_d, izq_m, izq_t, der_d, der_m, der_t`. Ese mismo orden es el de `Joint Names` en `ArtCtrl_Traccion`.
- `V_MAX = 0.167`: satura el comando. Sin esto, un `cmd_vel` de 0.5 m/s pediria 6.13 rad/s a ruedas que llegan a 2.045, el drive saturaria y se perderia la curva del motor.
- La normalizacion a ±90°: si el angulo pedido supera 90°, en vez de girar la rueda media vuelta se invierte el sentido de giro. Sin esto, en maniobras cerradas las ruedas dan vueltas absurdas.
- `SIGNO_DIR` y `SIGNO_VEL`: un signo por rueda (seccion siguiente).
- `steerPos` toma solo los indices 0, 2, 3, 5 (las esquinas), en el mismo orden que `Joint Names` de `ArtCtrl_Direccion`.

### 9.5 Calibrar los signos

Al hacer el lado izquierdo como espejo del derecho en Fusion, los ejes de algunos joints quedan invertidos. Un mismo comando produce giros opuestos. Se corrige con un signo por rueda en el script, y **se calibra joint a joint**, no por lado: en este modelo la delantera izquierda va con `+1` pero la trasera izquierda necesita `-1`.

Procedimiento:

1. Publicar avance recto. Si alguna rueda rueda hacia atras, invertir su `SIGNO_VEL`.
2. Publicar giro sobre el eje (`v = 0, w = 0.5`). Las cuatro esquinas deben orientarse tangentes a una circunferencia. La que apunte al reves, invertir su `SIGNO_DIR`.

Valores que resultaron para este modelo (verificar en cada importacion nueva):

```python
SIGNO_DIR = [1.0, 1.0, -1.0, -1.0, -1.0, -1.0]
SIGNO_VEL = [1.0, 1.0, 1.0, 1.0, -1.0, 1.0]
```

En un giro sobre el eje es normal que los numeros de velocidad salgan con signo distinto entre lados (por la normalizacion a ±90°) y que las delanteras giren mas rapido que las traseras (estan mas lejos del centro de rotacion). Lo que hay que mirar es que el centro del rover se quede quieto.

### 9.6 Probar

Con la simulacion en Play y una terminal ROS2 (`./terminal_b.bash`):

```bash
ros2 topic info /cmd_vel
```

Debe decir `Subscription count: 1`. Si es 0, revisar que la extension `omni.isaac.ros2_bridge` este activa.

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.167}, angular: {z: 0.0}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.3}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Recto, arco y giro sobre el eje. Publicar con `-r 10` y dejarlo corriendo: si el publisher se cierra, el nodo se queda sin comando.

Para depurar, mirar los puertos del Script Node en el Property Panel: `Inputs` muestra lo que llega de ROS y `Outputs` lo que calcula. Si `wheelVel` aparece como `[]`, el nombre del puerto no coincide con el del codigo.

## Parte 10: Guardar los dos USD

1. Con el Action Graph montado y funcionando: `File > Save As > robert_teleop.usd`.
2. Para RL, borrar todo lo que no sea el robot y guardar aparte:

```python
import omni.usd
stage = omni.usd.get_context().get_stage()
for p in ["/World/GroundPlane", "/World/defaultGroundPlane", "/World/Cube",
          "/World/ActionGraph", "/World/CollisionGroup"]:
    if stage.GetPrimAtPath(p):
        stage.RemovePrim(p)
```

`File > Save As > robert_RL.usd`. Dejar `/World/PhysicsMaterials`: es ligero y las ruedas dependen de el.

Si se olvida este paso, Isaac Lab replica el suelo, el cubo y el Action Graph en cada uno de los miles de entornos, y falla con `Replication of this type is not supported` en el `CollisionGroup`.

**Guardar a menudo.** Isaac Sim se cerro de golpe dos veces durante este trabajo y hubo que rehacer la configuracion entera. Por eso existe el script `01_configurar_joints.py`: para restaurarlo todo de una vez.

## Parte 11: Prueba de pendiente maxima

Antes de entrenar conviene saber cuanto puede subir el rover por pura fisica. [scripts/rampa_anillos.py](scripts/rampa_anillos.py) crea una rampa de anillos concentricos: plataforma central, y anillos hacia fuera cada uno con mayor pendiente (5° a 35° de 5 en 5), con un tramo llano de 1 m entre rampas para que el rover se asiente antes de la siguiente. Los anillos van coloreados de verde a rojo y hay una luz lateral que marca las pendientes con sombreado.

Se coloca el rover en el centro, se da Play y se publica avance recto. [scripts/donde_esta.py](scripts/donde_esta.py) dice en que tramo esta.

Lo que predice la fisica con estos materiales: TPU 0.9 × terreno 0.8 = 0.72 de friccion estatica, que da patinaje a partir de `atan(0.72) ≈ 36°`; una vez patina, con 0.56 dinamica no recupera por encima de 29°. La traccion no limita: las seis ruedas dan 325 N y el rover pesa 160 N.

## Problemas resueltos

- **Gazebo no sirve para este robot.** No soporta joints esfericos (los convierte en fijos) ni lazos cerrados (rechaza cuerpos con dos padres). Se probo y se descarto; Isaac Sim soporta ambas cosas.
- **El robot caia rigido como un bloque.** Los drives importados tenian stiffness alta y clavaban cada joint en su posicion inicial. Se liberaron los pasivos.
- **Explotaba al arrancar (salia volando).** Combinacion de lazo cerrado con tirantes de 43 g entre piezas de kilos. No se arreglo con masas (eso lo empeoro) sino con timestep mas fino e iteraciones del solver, que se aplican en Isaac Lab.
- **Una llanta apuntaba hacia adentro.** Frame del joint girado desde Fusion. Se roto `localRot1` 180°.
- **Las llantas giraban en sentidos opuestos.** Modelo espejado. Signos por rueda en el Script Node.
- **`OmniGraph Warning: 'llanta_der_m'`.** El joint se llama `llanta_der_m_`. Todos los nombres se verificaron con `repr()`.
- **Dos Articulation Controller duplicados.** Uno se creo por accidente al recablear; sus warnings confundieron el diagnostico durante un rato.
- **El Script Editor duplica el primer caracter al pegar** (`ffrom pxr import ...`). Cuando hay `SyntaxError` en la linea 1, mirar ahi primero.
- **Las mallas de colision no aparecian en `Traverse()`.** Estaban instanciadas. Se desinstanciaron solo las llantas.
- **Isaac Sim se cerro y se perdio el trabajo (dos veces).** Guardar con `Save As` a un archivo propio en cuanto algo funcione, y tener el script de restauracion.

## Trabajo futuro

- Montar la ZED 2 en el USD (dos camaras con baseline de 120 mm) y validar que el height map que genera coincide con el del entrenamiento.
- Repetir la prueba de pendiente con la friccion del suelo real (estimada en 0.6) en vez de 0.8.
- Exportar el Action Graph para reutilizarlo con otros robots cambiando solo la tabla `P` y los signos.
