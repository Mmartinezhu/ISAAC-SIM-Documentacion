# Parte 2: Reinforcement Learning de ROBERT en Isaac Lab

Entrenamiento con PPO de una politica que mueve el rover por terreno irregular siguiendo comandos de velocidad, sin perder el rumbo al pasar obstaculos.

Codigo completo de la tarea base en [referencia-completa.md](referencia-completa.md); el de las fases 2 a 4 (terrenos propios, recompensa por altura, diagnosticos y scripts) en [referencia-fases-2-4.md](referencia-fases-2-4.md).

Las imagenes van en `imagenes/`; la lista de capturas pendientes esta en [imagenes/README.md](imagenes/README.md).

## Objetivo

Que el rover aprenda a superar obstaculos manteniendo la trayectoria que se le pide. La politica recibe un comando de velocidad y decide la velocidad de cada una de las seis ruedas y el angulo de los cuatro reductores de direccion. La suspension es pasiva y la politica no puede tocarla.

El resultado se mide en centimetros de escalon que supera, grados de pendiente que sube y nivel de terreno irregular que cruza.

## Requisitos

- Isaac Lab 2.x en `~/Github/IsaacLab`, con `rsl_rl` instalado.
- El USD limpio del robot generado en la Parte 1: `/home/talos/IsaacsimManuel/ROBERT/robert_RL.usd`.
- GPU con memoria suficiente: con 2048 entornos y colision SDF en las ruedas se usan unos 10 GB.

```bash
cd ~/Github/IsaacLab
./isaaclab.sh -p -c "print('Isaac Lab OK')"
```



## Las cuatro fases

| Fase | Tarea | Terreno | Resultado |
| --- | --- | --- | --- |
| 1 | `Isaac-Rover-Robert-v0` | Escalones de subida y bajada, obstaculos sueltos | Base: aprende a moverse, seguir comandos y bajar escalones |
| 2 | `Isaac-Rover-Robert-Escalera-v0` | Escalera progresiva de 5 a 18 cm | Techo practico 7-9 cm; aparcada |
| 3 | `Isaac-Rover-Robert-Pendiente-v0` | Rampas de 5 a 40 grados | Limite 22-24 grados; cerrada |
| 4 | `Isaac-Rover-Robert-Denso-v0` | Bloques, fosos, huecos y rugosidad | Candidata a politica final para Nav2 |

Esta guia describe la fase 1 con todo detalle, porque las otras heredan de ella y solo cambian lo que se indica en sus secciones.

## Parte 1: El USD limpio

Isaac Lab carga el USD del robot y lo replica en cada entorno. Todo lo que contenga el archivo se replica: si tiene el suelo, el cubo de pruebas o el Action Graph de ROS2, cada uno de los 2048 entornos tendra su propio suelo, su cubo y un grafo intentando suscribirse a `/cmd_vel`. Ademas los `CollisionGroup` no se pueden replicar y hace fallar la carga.

El USD para RL debe contener solo `/World/cuerpo_suspension` y `/World/PhysicsMaterials`. Como generarlo esta en la Parte 1 del proyecto, seccion 10.

## Parte 2: Estructura de archivos

La tarea va dentro de las tareas de locomocion de Isaac Lab, junto a Anymal, Spot y compañia:

```text
source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/rover/
|-- __init__.py                 registro de las tareas en gym
|-- rover_env_cfg.py            fase 1: entorno base
|-- terrenos_rover.py           fases 2 y 3: generadores de terreno propios
|-- rover_escalera_env_cfg.py   fase 2
|-- rover_pendiente_env_cfg.py  fase 3
|-- rover_denso_env_cfg.py      fase 4
`-- agents/
    |-- __init__.py
    `-- rsl_rl_ppo_cfg.py       hiperparametros de PPO
```

Crear la estructura:

```bash
cd ~/Github/IsaacLab
BASE=source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/rover
mkdir -p $BASE/agents
```

Los archivos se crean pegando su contenido con `cat > ruta << 'EOF'`; el contenido esta en [referencia-completa.md](referencia-completa.md).

Una advertencia no dejar copias de estos archivos en la raiz de Isaac Lab. La tarea registrada lee la copia del paquete. Si se edita otra copia, los cambios no llegan y se entrena con la configuracion vieja sin ningun aviso.

## Parte 3: Registrar la tarea

`__init__.py`:

```python
import gymnasium as gym

from . import agents
from .rover_env_cfg import RoverEnvCfg

gym.register(
    id="Isaac-Rover-Robert-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": RoverEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RoverPPORunnerCfg",
    },
)
```

Comprobar:

```bash
./isaaclab.sh -p scripts/environments/list_envs.py | grep -i rover
```

Por que registrar y no instanciar la config a mano en un script propio: el `train.py` oficial pasa la configuracion por un decorador de Hydra que la procesa antes de construir el runner. Sin ese paso, `rsl_rl` recibe un diccionario incompleto y falla con `KeyError: 'class_name'`.

## Parte 4: El entorno, bloque a bloque

El archivo `rover_env_cfg.py` define una clase `RoverEnvCfg` compuesta por bloques. Aqui va cada uno con los valores usados y, sobre todo, por que esos valores. Casi ninguno coincide con los ejemplos de Isaac Lab, y hay una razon: los ejemplos estan calibrados para cuadrupedos que van a 1-3 m/s, y este rover va a 0.167 m/s. 

### 4.1 Robot y actuadores

```python
ROVER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=USD_ROVER,
        activate_contact_sensors=True,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=32,
            solver_velocity_iteration_count=8,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.65)),
    actuators={...},
)
```

Los actuadores de Isaac Lab sobreescriben los drives del USD. Lo configurado en la Parte 1 sirve para Isaac Sim; aqui se vuelve a declarar:

| Grupo | Joints | effort_limit_sim | velocity_limit_sim | stiffness | damping |
| --- | --- | --- | --- | --- | --- |
| `traccion` | `llanta.*` | 4.41 | 2.045 | 0 | 2.16 |
| `direccion` | `reductor.*` | 20 | 3 | 100 | 10 |
| `suspension` | `suspension`, `hombro.*`, `codo.*`, `union_sus.*` | **0** | 10 | 0 | 2 |

1 cosas importante:


- **`effort_limit_sim = 0` en la suspension** es lo que la mantiene pasiva. Sin eso la politica descubre que puede usar hombros y codos como musculos y aprende a reptar, cosa que el robot real no puede hacer.


Sobre el solver: 32 iteraciones de posicion y 8 de velocidad, el doble de lo habitual, porque el lazo cerrado con tirantes de 43 g entre piezas de kilos es numericamente delicado. Con menos, la simulacion explotaba.

El spawn a `z = 0.65` es alto a proposito. Con escalones de hasta 30 cm, nacer a 0.45 hacia que algunos rovers aparecieran dentro de la geometria y PhysX los expulsara con velocidades infinitas.

### 4.2 Terreno y curriculum

![Terreno de la fase 1](imagenes/01-terreno-fase1.png)

*Terreno de la fase 1 visto desde arriba: columnas de escalones de bajada, de subida y de obstaculos; la dificultad crece por filas.*

```python
ROVER_TERRAIN = TerrainGeneratorCfg(
    size=(5.0, 5.0),
    num_rows=20,
    num_cols=20,
    curriculum=True,
    sub_terrains={
        "subir":     MeshPyramidStairsTerrainCfg(proportion=0.35, step_height_range=(0.03, 0.25), step_width=0.9, ...),
        "bajar":     MeshInvertedPyramidStairsTerrainCfg(proportion=0.35, step_height_range=(0.03, 0.25), step_width=0.9, ...),
        "obstaculos": HfDiscreteObstaclesTerrainCfg(proportion=0.30, obstacle_height_range=(0.03, 0.25), num_obstacles=20, ...),
    },
)
```

El terreno es una cuadricula de 20 filas por 20 columnas de parcelas de 5 m. Las **filas** son niveles de dificultad (la altura de escalon se interpola linealmente entre el minimo y el maximo a lo largo de ellas) y las **columnas** son variaciones del mismo nivel. Cada rover queda fijo en un tipo de terreno (columna) y cambia de fila segun lo bien que lo haga.

| Parametro | Valor | Por que |
| --- | --- | --- |
| `size` | 5 m | Promocionar exige recorrer parte del terreno. Con 8 m eran 4 m por episodio, inalcanzable a 0.167 m/s |
| `step_height_range` | 3 a 25 cm | 3 cm se sube casi rodando; hace falta un nivel 0 que se supere por accidente para que la exploracion descubra que trepar paga. 25 cm es 1.5 diametros de rueda, el limite teorico del rocker-bogie |
| `num_rows` | 20 | Con 10 el salto entre niveles era de 2.4 cm; con 20 es de 1.2. El nivel 1 pasa de 5.4 a 4.2 cm |
| `step_width` | 0.9 m | El rover mide 0.826 m de punta a punta. Con el valor por defecto (0.3 m) esta a caballo entre tres peldaños y choca con el siguiente antes de subir el anterior |
| `num_obstacles` | 20 | Con 6 en 25 m² un rover cruza la parcela sin tocar ninguno |

Un dato que salio de la geometria y explica mucho: el radio de rueda es 8.15 cm, y un escalon de esa altura es el limite fisico para subir por rodadura. Por debajo, el punto de contacto queda bajo el eje y la rueda trepa sola; en el radio, el contacto esta a la altura del eje y solo puede empujar contra la pared. Por encima solo se sube si las otras ruedas empujan de forma coordinada. 
### 4.3 Curriculum propio

El curriculum estandar de Isaac Lab (`terrain_levels_vel`) sube de nivel a quien recorre mas de medio terreno y baja a quien recorre menos de la mitad de lo comandado. Para un cuadrupedo eso funciona; para este rover no hay zona intermedia: cualquiera que no recorra 2.5 m en el episodio **baja** de nivel. El resultado era un curriculum que descendia sin parar aunque el rover mejorara.

Se sustituye por uno con banda neutra:

```python
def niveles_terreno_rover(env, env_ids, asset_cfg=SceneEntityCfg("robot")):
    distancia = norm(root_pos_xy - env_origin_xy)
    sube = distancia > 1.5
    baja = distancia < 0.5
    baja *= ~sube
    terrain.update_env_origins(env_ids, sube, baja)
    return mean(terrain.terrain_levels)
```

Por encima de 1.5 m promociona, por debajo de 0.5 m retrocede, y entre medias **se queda donde esta**. 1.5 m en 60 s son 0.025 m/s, un 15 % de la velocidad maxima: alcanzable aunque se pelee con un escalon.

Se añaden dos terminos que solo reportan, `nivel_max` y `nivel_min`, para ver la dispersion. Si la media sube pero el minimo se queda en 0, hay un grupo de rovers que nunca despega.

### 4.4 Sensor: height scan

![Height scan sobre el rover](imagenes/02-height-scan.png)

*Rejilla del height scan (debug_vis) sobre el rover: cada punto es la altura del terreno bajo el.*

```python
height_scanner = RayCasterCfg(
    prim_path="{ENV_REGEX_NS}/Robot/cuerpo_suspension/base_link",
    offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 1.0)),
    ray_alignment="yaw",
    pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[1.0, 0.85]),
    mesh_prim_paths=["/World/ground"],
)
```

Una rejilla de 21 × 18 rayos verticales (378 puntos) alrededor del rover que devuelve la altura del terreno bajo cada uno. Es la percepcion de la politica.

Por que un height scan y no la camara ZED 2 del robot real: renderizar es el cuello de botella del RL. Isaac Lab corre entornos sin camara a decenas de miles de pasos por segundo, y con camara cae a unos pocos miles. Los trabajos de locomocion en terreno irregular (ANYmal, los cuadrupedos de ETH) entrenan con height scan y en el robot real lo **generan** desde la nube de puntos de la camara. La politica ve lo mismo en simulacion y en realidad.

| Parametro | Valor | Por que |
| --- | --- | --- |
| `resolution` | 5 cm | Con 10 cm un obstaculo de 10 cm cae en un solo punto y parpadea al moverse el rover |
| `size` | 1.0 × 0.85 m | Cubre el rover y medio metro por delante |
| `ray_alignment` | `yaw` | La rejilla sigue la orientacion horizontal pero **no** el cabeceo ni el balanceo. Sin esto, al inclinarse sobre un obstaculo la politica veria el terreno moverse por su propio movimiento |
| `mesh_prim_paths` | `/World/ground` | Solo el terreno. Sin esto los rayos chocan con el propio rover |

El `prim_path` lleva `/cuerpo_suspension/` porque el USD tiene ese nivel intermedio. Sin el, el sensor no encuentra `base_link`.

Antes de montar esto en Isaac Lab se prototipo el height scan con raycasts en Isaac Sim, sobre un cubo, para calibrar tamaño y resolucion. Fue util: ahi se vio que con 10 cm el cubo aparecia en un unico punto.

### 4.5 Comandos

```python
base_velocity = mdp.UniformVelocityCommandCfg(
    resampling_time_range=(30.0, 60.0),
    heading_command=True,
    ranges=Ranges(lin_vel_x=(0.05, 0.167), lin_vel_y=(0.0, 0.0), ang_vel_z=(-0.3, 0.3), heading=(-pi, pi)),
)
```

El rover no tiene un destino. Recibe una velocidad y un rumbo, y su unica tarea es mantenerlos. Los escalones son estorbos que aparecen en su camino.

`resampling_time_range` es de 30 a 60 s, no los 10 s habituales. Con un rumbo nuevo cada 10 s, un episodio de 60 s tiene seis tramos en direcciones aleatorias: un paseo aleatorio que acaba cerca del origen aunque el rover se mueva todo el tiempo. Como el curriculum mide desplazamiento neto, nunca promocionaba.

`lin_vel_x` va de 0.05 a 0.167 m/s, la velocidad real del motor. Se descarto subirla a 1 m/s en simulacion: con la misma potencia (9 W por rueda) el par caeria a 0.74 N·m y el rover no subiria nada. Potencia es par por velocidad; inventar un rover mas rapido produce una politica para un robot que no existe.

### 4.6 Acciones

```python
traccion = mdp.JointVelocityActionCfg(joint_names=["llanta.*"], scale=2.045)
direccion = mdp.JointPositionActionCfg(joint_names=["reductor.*"], scale=pi/2, use_default_offset=True, clip={".*": (-pi/2, pi/2)})
```

Diez acciones en [-1, 1]. Las seis primeras se escalan a velocidad de rueda (±2.045 rad/s); las cuatro ultimas a angulo de direccion (±90°). El `clip` acota tras el escalado, porque la red gaussiana puede sacar valores fuera de [-1, 1] y pediria 135° a un joint limitado a 90°.

Aqui esta la diferencia con el control por ICR de la Parte 1: alli un script calcula las diez consignas desde `v` y `w`; aqui la politica las decide una a una. Puede dosificar el par rueda a rueda al trepar, que es lo que hace un rover real.

### 4.7 Observaciones

| Termino | Dimension | Ruido | Clip |
| --- | --- | --- | --- |
| `base_lin_vel` | 3 | ±0.1 | ±10 |
| `base_ang_vel` | 3 | ±0.2 | ±10 |
| `projected_gravity` | 3 | ±0.05 | |
| `velocity_commands` | 3 | | |
| `joint_pos` | 21 | ±0.01 | ±10 |
| `joint_vel` | 21 | ±0.5 | ±50 |
| `actions` | 10 | | |
| `height_scan` | 378 | ±0.02 | ±1 |

Total 442. Los 21 de joints son 15 revolute mas los dos esfericos dentro de la articulacion, que cuentan tres cada uno. El ruido hace que la politica tolere sensores imperfectos.

El `height_scan` pasa por una funcion propia, `height_scan_seguro`, que aplica `torch.nan_to_num`. Cuando un rayo no impacta nada, el sensor devuelve infinito; y `clip` acota infinitos pero **deja pasar los NaN intactos**. Eso costo un entrenamiento.

### 4.8 Eventos

| Evento | Que hace | Por que |
| --- | --- | --- |
| `reset_base` | Recoloca el rover con desplazamiento ±0.2 m y guiñada aleatoria | Con ±0.5 m caia dentro de escalones |
| `reset_joints` | Joints a su posicion por defecto | |
| `randomize_mass` | Masa del chasis ×0.8 a ×1.2 | Que la politica no se ajuste a 16.27 kg exactos |
| `randomize_friction` | Friccion de las ruedas 0.6 a 1.1 | Que tolere suelos distintos |

Los dos ultimos son lo que se llama *domain randomization*, y son la razon de que la politica pueda transferir a un robot que nunca pesa ni agarra exactamente lo que dice la simulacion.

### 4.9 Recompensas

| Termino | Peso | std | Que premia o castiga |
| --- | --- | --- | --- |
| `seguir_velocidad` | +3.0 | 0.05 | Ir a la velocidad comandada |
| `seguir_rumbo` | +1.5 | 0.1 | Girar a la velocidad angular comandada |
| `penaliza_balanceo` | -0.01 | | Velocidad angular de cabeceo y balanceo |
| `penaliza_par` | -2.5e-6 | | Par usado |
| `penaliza_aceleracion` | -2.5e-8 | | Aceleracion articular |
| `penaliza_cambio_accion` | -0.005 | | Cambios bruscos de accion |
| `penaliza_contacto_indebido` | -2.0 | | Contacto de cualquier parte que no sea una llanta |

**El `std = 0.05` es el parametro mas importante de toda la configuracion.** La recompensa de seguimiento es `exp(-error² / std²)`. Con el `std = 0.25` de los ejemplos y un rover cuyo error nunca supera 0.167 m/s, un rover **completamente parado** con el comando maximo cobra `exp(-0.167²/0.25²) = 0.64`, el 64 % de la recompensa. Con comandos pequeños llega al 96 %. La politica aprendio que quedarse quieto era optimo, y las graficas lo mostraban con claridad: `seguir_velocidad` al 96 % del maximo mientras las penalizaciones de suavidad mejoraban monotonamente, es decir, el rover haciendo cada vez menos. Con `std = 0.05` ese mismo rover parado cobra `exp(-11.2) ≈ 0`.

Lo que **no** se penaliza, y por que:

- **Inclinarse** (`flat_orientation_l2`): para trepar hay que inclinarse. Castigarlo es pedirle que no haga lo que necesita.
- **Velocidad vertical** (`lin_vel_z_l2`): en escalones el movimiento vertical *es* subir el obstaculo. Ese termino sirve para que un cuadrupedo no rebote en llano.

Lo que si se castiga con fuerza es que cualquier pieza que no sea una llanta toque algo: chasis, hombros, codos, almas, tirantes. Es lo que pidio el usuario y lo que de verdad limita al robot real.

Las penalizaciones de suavidad (par, aceleracion, cambio de accion) estan diez o veinte veces mas bajas que en los ejemplos. Sirven para pulir un movimiento que ya funciona, no para descubrirlo; altas desde el principio, la respuesta optima siempre es no hacer nada.

### 4.10 Terminaciones

| Termino | Condicion |
| --- | --- |
| `tiempo_agotado` | 60 s |
| `volcado` | Inclinacion mayor de 60° |
| `encallado` | El chasis (`base_link`) toca algo |
| `inestable` | Estado fisico invalido (ver abajo) |

Los 60° distinguen lo que importa: inclinarse 40° trepando es legitimo; pasar de 60° es que esta volcando. Sin la terminacion de volcado, la politica descubre que caerse y arrastrarse tambien avanza.

`inestable` es una funcion propia, `estado_invalido`, que corta el episodio si alguna velocidad del chasis o de los joints es infinita, NaN, o absurda (mas de 10 m/s en un rover que va a 0.167). Es una red de seguridad: un entorno que se desestabiliza se resetea limpio en vez de contaminar el batch.

### 4.11 Paso de simulacion

```python
self.decimation = 8
self.sim.dt = 0.0025
self.episode_length_s = 60.0
```

Fisica a 400 Hz y control a 50 Hz. El paso de control es el mismo que con `dt = 0.005` y `decimation = 4`, pero la fisica resuelve el doble de fino. Hizo falta por el lazo cerrado.

Episodios de 60 s en vez de 40: a 0.167 m/s, el rover necesita tiempo para atascarse en un escalon, resolverlo y aun asi recorrer los 1.5 m que exige promocionar.

## Parte 5: Configurar PPO

`agents/rsl_rl_ppo_cfg.py`, copiado del de AnymalC con tres cambios:

| Campo | Valor | Anymal | Por que se cambio |
| --- | --- | --- | --- |
| `actor_obs_normalization` | **True** | False | Sin ella la red divergia a las ~580 iteraciones (ver Problemas resueltos) |
| `critic_obs_normalization` | **True** | False | Idem |
| `learning_rate` | 5e-4 | 1e-3 | Mas conservador tras la divergencia |
| `max_grad_norm` | 0.5 | 1.0 | Idem |
| `max_iterations` | 10000 | 1500 | |
| `save_interval` | 250 | 50 | Para no llenar el disco |
| Red | [512, 256, 128] | [512, 256, 128] | |

El resto (clip 0.2, entropia 0.005, gamma 0.99, lambda 0.95, KL adaptativo 0.01) se deja igual: son valores robustos para locomocion.

Se eligio `rsl_rl` frente a `skrl` porque es la libreria con la que ETH entreno ANYmal en terreno irregular con height scan, el mismo problema, y su configuracion cabe en una pantalla.

## Parte 6: Probar antes de entrenar

Antes de un entrenamiento de horas, comprobar que el entorno arranca, que encuentra los diez joints y que el rover se mueve. El script `probar_rover.py` de [referencia-completa.md](referencia-completa.md) instancia el entorno con 16 rovers, manda velocidad fija a las ruedas y muestra:

```text
JOINTS ENCONTRADOS: ['hombro_der', ..., 'union_sus_der:0', 'union_sus_der:1', 'union_sus_der:2', ..., 'llanta_izq_d_']
DIM OBSERVACION: {'policy': (442,)}
DIM ACCION: 10
```

Dos cosas que verificar en esa lista:

- Que esten los seis `llanta*` y los cuatro `reductor*`.
- Que aparezcan `union_sus_der:0/1/2` y `union_sus_izq:0/1/2` pero **no** los `_01`. Los que faltan son los excluidos de la articulacion: es la confirmacion de que el lazo cerrado quedo bien resuelto.

## Parte 7: Entrenar

```bash
cd ~/Github/IsaacLab
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Rover-Robert-v0 --num_envs 2048 --headless
```

Empezar con 512 entornos y subir. La colision SDF de las seis ruedas es lo mas caro de la escena. En la RTX 5080: 4096 entornos a 2.88 s por iteracion, 6144 a 3.60 s (41 000 pasos por segundo). Se usan 6144.

Reanudar desde el ultimo checkpoint:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Rover-Robert-v0 --num_envs 2048 --headless --resume
```

`--resume` es un flag sin valor; `--resume True` hace que Hydra reciba `True` como override y falle.

Reanudar desde un run concreto con nombre propio:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Rover-Robert-v0 --num_envs 6144 --headless --resume --load_run '2026-09-10_12-42-31' --run_name escalera
```

Entrecomillar siempre `--load_run`: `.*` sin comillas lo expande bash. Y dar el nombre explicito, porque `'.*'` coge la carpeta mas nueva, que es la del propio run que se esta creando.

Cuando **si** hay que entrenar de cero y no reanudar: si cambia la dimension de observaciones o acciones, si se activa la normalizacion de observaciones (el checkpoint viejo no tiene esas estadisticas), o si las recompensas cambian de forma radical.

## Parte 8: Ver el entrenamiento

TensorBoard, en otra terminal:

```bash
./isaaclab.sh -p -m tensorboard.main --logdir logs/rsl_rl/rover_robert
```

Si el 6006 esta ocupado abre en el 6007; cerrar la instancia vieja para no mirar el run equivocado.

Reproducir el ultimo checkpoint:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Rover-Robert-v0 --num_envs 8 env.sim.render_interval=40 env.scene.terrain.max_init_terrain_level=8
```

- `render_interval=40`: dibuja un frame cada 40 pasos de fisica en vez de cada 8. La simulacion avanza seis veces mas rapido respecto al reloj; a 0.167 m/s el rover tarda 6 s por metro y sin esto es desesperante.
- `max_init_terrain_level=N`: en que nivel aparecen. El checkpoint no guarda el estado del curriculum, asi que sin esto salen en el nivel 0 y no se ve nada de lo aprendido. Ir subiendo N para encontrar donde falla.

Para seguir a un rover con la camara: seleccionarlo en el Stage y pulsar `F`.

Para verlo a velocidad real en vez de acelerado (util para juzgar como falla): `--real-time`. Y siempre con pocos entornos (`--num_envs 8` o `16`): si no se pasa `--num_envs`, `play.py` levanta los miles de rovers del entrenamiento y el viewport se arrastra. Si ademas se mira por NoMachine, parte del tiron de camara es la red, no la simulacion.
![play.py con 8 rovers](imagenes/03-play-viewport.png)

*play.py con 8 rovers en el nivel 8. La flecha verde es el comando de velocidad y la azul la velocidad real.*


## Parte 9: Interpretar las metricas

| Metrica | Que dice | Que se quiere ver |
| --- | --- | --- |
| `Train/mean_reward` | Desempeño global | Sube sostenidamente |
| `Train/mean_episode_length` | Cuanto sobreviven | Hacia los 60 s |
| `Episode_Reward/seguir_velocidad` | Si va a la velocidad pedida | Sube desde cerca de 0 (si arranca alto, el `std` esta mal) |
| `Episode_Reward/seguir_rumbo` | Si mantiene el rumbo | Sube |
| `Curriculum/terreno` | Nivel medio de dificultad | Sube |
| `Curriculum/nivel_max` y `nivel_min` | Dispersion | Que el minimo no se quede en 0 |
| `Episode_Termination/*` | Por que acaban los episodios | Que domine `tiempo_agotado` |

![Panel de TensorBoard](imagenes/04-tensorboard-metricas.png)

*Panel de TensorBoard de un run sano: Curriculum/terreno sube, seguir_velocidad sube desde cerca de 0, Episode_Termination dominado por tiempo_agotado.*

`Episode_Termination` es la mas diagnostica y la menos mirada. Si `inestable` es distinto de cero, la fisica esta explotando. Si `volcado` domina, la politica aun no sabe mantenerse. Si todo es `tiempo_agotado` pero el curriculum baja, el rover sobrevive sin avanzar.

Para convertir nivel en centimetros con 20 filas y rango 3-25 cm: `altura = 0.03 + nivel × 0.01158`. Nivel 4 son 7.6 cm, nivel 8 son 12.3 cm, nivel 19 son 25 cm.

## Parte 10: Resultados de la fase 1

Tras 10 000 iteraciones con 2048 entornos:

| Metrica | Valor |
| --- | --- |
| `Curriculum/terreno` (media) | 8.5 de 19 → escalones de **12.8 cm** |
| `Curriculum/nivel_max` | 19 → 25 cm |
| `Curriculum/nivel_min` | 0 |
| `Episode_Termination/tiempo_agotado` | 99.9 % |

Dos curriculos distintos (10 filas y 20 filas) convergieron al mismo 12.8 cm, lo que apuntaba a un limite real y no del curriculum. Un reentrenamiento desde cero con el clip de ±90° en la direccion (`clip90_cero`, 6144 entornos, 8000 iteraciones) dio el mismo resultado: el clip no cuesta rendimiento. Ese es el checkpoint base para las fases siguientes: `logs/rsl_rl/rover_robert/*_clip90_cero/model_8000.pt`.

![Curriculum de clip90_cero](imagenes/05-fase1-curriculum.png)

*clip90_cero: Curriculum/terreno se estabiliza en ~9 de 20, nivel_max llega al tope a las 2300 iteraciones y nivel_min no sale de 0 en las 8000.*

**El hallazgo que cambio el proyecto** salio al mirar el `play.py` con calma: los 12.8 cm venian de los rovers que **bajan** escalones. Los de la zona de subida nunca salieron del nivel 0. La politica no habia aprendido a subir un escalon vertical. Como cada rover esta fijo en su tipo de terreno y `Curriculum/terreno` es la media de los tres tipos, la curva escondia dos comportamientos opuestos.

## Parte 11: Fase 2, escalera progresiva

Para atacar la subida en aislamiento se creo un terreno propio (`EscaleraProgresivaCfg` en `terrenos_rover.py`): plataforma central y anillos cuadrados concentricos con escalones de 5, 7, 9, 11, 12, 13, 14, 15, 16, 17 y 18 cm, huella de 1 m (cabe el rover entero), sin curriculum de niveles. Tarea `Isaac-Rover-Robert-Escalera-v0`, entorno `RoverEscaleraEnvCfg(RoverEnvCfg)`.

![Escalera progresiva](imagenes/06-escalera-terreno.png)

*Escalera progresiva: plataforma central de 3 m y once anillos de 1 m con escalones de 5 a 18 cm. Las cuatro copias son la rejilla 2 x 2 del generador.*

Seis runs de afinado:

| Run | Cambio | Que paso |
| --- | --- | --- |
| `escalera` | Rumbos aleatorios cada 30-60 s | Subian y volvian a bajar al pozo; altura final ~0 |
| `escalera2` | Un rumbo fijo por episodio | Aprendio a **aparcar contra el primer escalon**: con `std=0.05` quedarse quieto cobra el 57-95 % del tracking y subir solo cuesta penalizaciones |
| `escalera3` | Recompensa `AlturaGanada` | Paga `(z - z_max)/dt` solo cuando el rover supera su record de altura del episodio, peso 200 (2 puntos por cm). Vale 0 en llano, no se puede farmear balanceandose, compatible con Nav2 |
| `escalera4` | Spawn repartido por toda la escalera, comando radial hacia fuera (±60°) | Con spawn central el rover gastaba el 80 % del episodio en escalones ya dominados. Tres bugs: el record se fijaba con el rover aun en el aire (arreglado con 1 s de espera tras el reset), la lista de escalones era vieja, y los de arriba bajaban por el pozo vecino |
| `escalera5` | Bugs corregidos, episodios de 40 s | Diagnostico por escalon: 5 cm 70-80 %, 7 cm 30-50 %, 9 cm 10-25 %, 11-12 cm ~5 %, 13 cm y mas 0 % |
| `escalera6` | Sin terminacion `encallado`, 3000 iteraciones mas | Perfil identico |

La caida gradual de exito con la altura apuntaba a un limite de politica. Pero la captura del `play.py` mostro otra cosa: la rueda delantera sube, y **la rueda media choca de frente con la cara del escalon** y no puede pivotar. Es una limitacion geometrica del rocker-bogie con ruedas de 8 cm de radio, no de la politica.

![Rueda media bloqueada](imagenes/07-escalera-rueda-media.png)

*Captura de play.py en escalera6: la rueda delantera ya esta sobre el escalon y la media choca de frente con la cara vertical. Con la trasera empujando y la delantera tirando no le alcanza para pivotar.*

![Exito por escalon](imagenes/08-escalera-exito.png)

*Curriculum/exito/escalon_Xcm en escalera6: caida gradual de ~65 % en 5 cm a ~5 % en 11-12 cm y cero a partir de 13 cm.*

**Techo practico: 7 a 9 cm de escalon.** La fase quedo aparcada.

Queda una prueba pendiente que aclararia si el limite es aun mas bajo: un test fisico con traccion maxima y sin politica no subio ni un escalon de 2 cm, y la sospecha es que la terminacion `encallado` reseteaba al rover al morder el borde. No se llego a confirmar.

## Parte 12: Fase 3, pendientes

Terreno `PendienteProgresivaCfg`: rampas cuadradas concentricas de 5 a 40 grados, de 1.5 m cada una. Tarea `Isaac-Rover-Robert-Pendiente-v0`, entorno `RoverPendienteEnvCfg(RoverEscaleraEnvCfg)`, con la misma recompensa `AlturaGanada`.

![Pendiente progresiva](imagenes/09-pendiente-terreno.png)

*Pendiente progresiva con color_scheme='height': cada banda de color es una rampa, de 20 a 34 grados de dentro hacia fuera. Los rovers nacen sobre la rampa mirando cuesta arriba.*

| Run | Cambio | Resultado |
| --- | --- | --- |
| `pend_diag` | 250 iteraciones desde `clip90_cero` (linea base) | 5-15° al 90-100 %, 20° al 45 %, 25° al 0 %. Aprende rapido |
| `pendiente1` | 2000 iteraciones | 10° 93 %, 15° 85 %, 20° 63 %, 25° 31 %, 30° 16 %, 35° 0 % |
| `pendiente2` | Reset **sobre la rampa** con el morro cuesta arriba; rampas finas de 20 a 34° de 2 en 2; 1500 iteraciones | En el pico: 22° 92 %, 24° 78 %, 26° 57 %, 28° 33 %, 30° 3 % |

El reset sobre la rampa vino de ver en `play.py` que los rovers nacian 70 cm en el aire con guiñada aleatoria y volcaban al aterrizar en 25-30°: fallos falsos que contaminaban la medida.

En `pend_diag` se descubrio un **defecto de medida** que afectaba tambien a la escalera: los terminos de curriculum registran solo el ultimo lote de resets de cada iteracion (uno o dos rovers volcados), pisando el lote grande del `time_out`. Las tasas de exito eran muestras minusculas y sesgadas. Se arreglo con `ExitoRampa`, una clase `ManagerTermBase` que acumula exitos y cuentas con olvido exponencial. Las tendencias anteriores eran validas; los numeros, no.

![Tasas por rampa](imagenes/10-pendiente-rampa.png)

*Curriculum/rampa/Xdeg y n_Xdeg en pendiente2: 22 grados al 92 % en el pico, 26 al 57 %, 30 al 3 %; entre 350 y 1400 rovers en ventana por rampa.*

**Limite practico con friccion 1.0: 22-24 grados. 26° a medias, 30° excepcional.** Mejor checkpoint: `pendiente2/model_11000.pt` (las rampas faciles empeoraron al final del run, posible sobreoptimizacion del premio por altura). Con la friccion real del suelo (estimada en 0.6) bajara unos grados. Fase cerrada.

## Parte 13: Fase 4, terreno denso

Es la tarea original (curriculum `terreno`, rumbos aleatorios, sin `AlturaGanada`) cambiando solo el generador. Tarea `Isaac-Rover-Robert-Denso-v0`, entorno `RoverDensoEnvCfg(RoverEnvCfg)`:

| Sub-terreno | Proporcion | Parametros |
| --- | --- | --- |
| `HfDiscreteObstacles` | 35 % | 120 bloques y fosos de ±3 a ±14 cm |
| `HfSteppingStones` | 25 % | Huecos de 5 a 22 cm, profundidad 10 cm |
| `MeshRandomGrid` | 25 % | Celdas de 45 cm, 2 a 10 cm |
| `HfRandomUniform` | 15 % | Rugosidad de 2 a 8 cm |

20 filas × 12 columnas de 8 m, friccion 0.8 en modo `max`.

![Terreno denso](imagenes/11-denso-terreno.png)

*Terreno denso: de izquierda a derecha, columnas de bloques y fosos, losas con huecos, rejilla y rugosidad; la dificultad crece hacia el fondo.*

`denso1`, 3000 iteraciones desde `clip90_cero`: `Curriculum/terreno` en 12 de 20 (bloques y fosos de ±9.6 cm, huecos de 15 cm) y aun subiendo despacio. Terminaciones: 99.8 % tiempo agotado, 0.1 % vuelco, 0.03 % encallado. El techo coincide con el limite de ~9 cm de escalon que impone la rueda media.

![denso1 en TensorBoard](imagenes/12-denso-tensorboard.png)

*denso1: Curriculum/terreno llega a 12 de 20 y sigue subiendo despacio; seguir_velocidad baja de 2.0 a 1.0 a medida que el terreno se endurece; las terminaciones siguen siendo 99.8 % tiempo agotado.*

**Es la candidata a politica final para Nav2**, porque es la unica de las cuatro que responde a comandos de velocidad y rumbo arbitrarios sobre terreno variado.

## Problemas resueltos

En orden cronologico. Cada uno costo al menos un entrenamiento.

- **`KeyError: 'class_name'` al construir el runner.** Se instanciaba la config a mano en un script propio, saltandose el decorador de Hydra de `train.py`. Registrar la tarea y usar los scripts oficiales.
- **`Failed to find a prim at path .../Robot/base_link`.** El USD tiene un nivel `cuerpo_suspension` intermedio. Añadirlo al `prim_path` del sensor y del contacto.
- **`velocity_limit` se ignoraba.** Nombre obsoleto. Cambiar a `velocity_limit_sim` y `effort_limit_sim`.
- **La politica aprendio a quedarse quieta.** `std = 0.25` en el tracking. Bajar a 0.05. Las graficas lo delataban: `seguir_velocidad` al 96 % del maximo con el rover inmovil.
- **No trepaba.** Se penalizaba inclinarse y moverse en vertical. Quitar ambos terminos; castigar contacto indebido en su lugar.
- **El curriculum bajaba sin parar.** `terrain_levels_vel` sin banda neutra. Curriculum propio; terreno de 8 a 5 m; episodios de 40 a 60 s.
- **El curriculum seguia sin subir.** Rumbos cada 10 s → paseo aleatorio. Subir a 30-60 s.
- **`NaN` en el `std` de la politica (`normal expects all elements of std >= 0.0`).** Tres intentos: spawn mas alto y reset mas corto (por si nacian dentro de escalones), `clip` en observaciones (no sirve: `clamp` deja pasar `NaN`), terminacion `inestable` y solver mas fino. Seguia fallando **en la misma iteracion** al reanudar, lo que revelo que era determinista: la red divergia por la escala dispar de las observaciones (de 0.1 a 50). Activar normalizacion de observaciones, bajar el learning rate y el grad norm, entrenar de cero.
- **Los cambios no se aplicaban.** Se editaba una copia del archivo en la raiz de Isaac Lab; la tarea leia la del paquete. Borrar las copias.
- **El nivel 0 era ya el limite fisico.** 8 cm de escalon con 8.15 cm de radio. Bajar el minimo a 3 cm y subir a 20 filas.
- **`Replication of this type is not supported`.** El USD contenia la escena entera. Limpiarlo.
- **`AlturaGanada` pagaba 0 todo el run.** Fijaba el record con el rover 65 cm en el aire tras el reset. Esperar 1 s.
- **Tasas de exito falsas.** Los terminos de curriculum solo veian el ultimo lote de resets. Acumular con `ManagerTermBase`.
- **Fallos falsos en pendientes.** Los rovers nacian en el aire y volcaban al caer en 25-30°. Reset sobre la rampa con el morro cuesta arriba.

- **`Missing key(s) in state_dict: obs_normalizer.*` al reanudar.** El run elegido se habia entrenado sin normalizacion de observaciones y la config actual la tiene. No sirve cargar con `strict=False` (la red veria entradas en otra escala): buscar el run correcto con `grep obs_normalization logs/rsl_rl/rover_robert/*/params/agent.yaml`, o entrenar de cero.
- **`LexerNoViableAltException: .git` de Hydra.** `--load_run .*` sin comillas: bash expandio `.*` a los archivos ocultos del directorio. Entrecomillar siempre. Y `'.*'` con comillas coge la carpeta mas nueva, que es la del run que se esta creando: dar el nombre.
- **`No runs present ... match: 'escalera'`.** `--load_run` usa `re.match`, anclado al principio del nombre. Hace falta `'.*_escalera'`.
- **Un script propio moria en silencio al crear la escena (`exit=0`, sin traceback).** Se guardaba solo `AppLauncher(args).app` y el objeto `AppLauncher` se destruia al instante. Guardar `launcher = AppLauncher(args)`.
- **Un script propio no imprimia nada aunque terminara bien.** `app.close()` sale con `os._exit` y el buffer de stdout se pierde. Escribir los resultados a archivo o usar `flush=True`.
- **Segfault de Kit a 0 ms de arranque.** Habia otro Isaac Sim abierto (un `play.py` con viewport) y el segundo se quedaba sin VRAM. Comprobar con `nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv` antes de lanzar.
- **La prueba con 60 iteraciones no mostraba resets por tiempo.** 60 iteraciones × 24 pasos × 0.02 s son 29 s, menos que un episodio de 40 s. Un diagnostico necesita 170-250 iteraciones.
- **La escalera no era la que se creia.** Un `sed` de una prueba anterior habia dejado una lista de escalones vieja y el de 18 cm no existia. `ver_parches.py` lo delato (maximo en 1.11 m en vez de 1.37). Comprobar la linea `[escalera_progresiva] ...` que imprime el terreno al arrancar.

## Trabajo futuro

- Reproducir `denso1` sobre el terreno original de la fase 1 y comprobar que no perdio la capacidad de bajar escalones.
- Afinar `denso1` con friccion aleatoria de 0.5 a 1.0 y masa ±20 %, para que la transferencia al robot real sea mas robusta.
- Repetir la evaluacion de pendientes con friccion 0.6, la estimada para el suelo real.
- Exportar la politica (`exported/policy.onnx`) y escribir el nodo ROS2 que la ejecute en el robot, con el height scan generado desde la ZED 2.
- Confirmar o descartar la sospecha sobre `encallado` en la prueba fisica de escalones, y si se retoma esa fase, sustituirla por una terminacion de "atascado" real (velocidad cero con comando distinto de cero durante N segundos).

## Donde esta cada cosa

| Que | Donde |
| --- | --- |
| Codigo completo de la fase 1 | [referencia-completa.md](referencia-completa.md) |
| Codigo completo de las fases 2 a 4, scripts de diagnostico, comandos y cuadro de runs | [referencia-fases-2-4.md](referencia-fases-2-4.md) |
| En la maquina Ubuntu | `~/Github/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/rover/` y `~/Github/IsaacLab/scripts/` |
| Checkpoints | `~/Github/IsaacLab/logs/rsl_rl/rover_robert/<fecha>_<run_name>/model_N.pt` |
| Politica exportada (la genera `play.py`) | `.../<run>/exported/policy.pt` y `policy.onnx` |

En la maquina, la fase 2 esta como parches sucesivos al final del archivo; la referencia de este repositorio es la version consolidada, equivalente.
