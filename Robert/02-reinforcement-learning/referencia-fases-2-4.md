# Referencia completa: fases 2, 3 y 4

Codigo de las tareas `Isaac-Rover-Robert-Escalera-v0`, `Isaac-Rover-Robert-Pendiente-v0` e `Isaac-Rover-Robert-Denso-v0`, mas los dos scripts de diagnostico. Todo hereda de la fase 1 ([referencia-completa.md](referencia-completa.md)) y solo cambia lo que se indica.

Los archivos se crean igual que en la fase 1:

```bash
cd ~/Github/IsaacLab
BASE=source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/rover
cat > $BASE/NOMBRE << 'EOF'
(contenido)
EOF
```

Nota sobre fidelidad: en la maquina Ubuntu la fase 2 se fue construyendo a base de parches sucesivos (un bloque de diagnostico añadido al final del archivo que redefine metodos de la clase). Aqui se presenta la **version consolidada**, equivalente en comportamiento y mas legible. Si se copia de la maquina, se vera la version con parches.

---

## 1. Terrenos propios: `terrenos_rover.py`

Dos sub-terrenos con la misma idea: el rover nace en el centro de un pozo y todo lo que hay alrededor es cada vez mas dificil. Asi el curriculum esta en la geometria y no hace falta el de niveles.

Isaac Lab construye cada sub-terreno con una funcion `f(difficulty, cfg) -> (lista de mallas trimesh, origen)`. La malla se expresa en un cuadro `[0, size] x [0, size]` y el origen es donde nacen los rovers. Con `curriculum=False` y `difficulty` ignorada, todos los sub-terrenos son identicos.

```python
"""Sub-terrenos propios del rover ROBERT: escalera y pendiente progresivas."""

from __future__ import annotations

import math

import numpy as np
import trimesh

from isaaclab.terrains import SubTerrainBaseCfg
from isaaclab.utils import configclass


# ---- escalera progresiva ----------------------------------------------------------------

def escalera_progresiva(difficulty: float, cfg: EscaleraProgresivaCfg) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Plataforma central a z=0 rodeada de anillos cuadrados; el anillo k sube ``cfg.subidas[k]``.

    Agotada la lista, el resto hasta el borde queda plano a la altura final. Cada anillo son
    cuatro cajas (norte, sur, este, oeste) que van desde z_base hasta su altura.
    """
    cx, cy = 0.5 * cfg.size[0], 0.5 * cfg.size[1]
    semilado = 0.5 * min(cfg.size)
    z_base = -cfg.grosor_base
    meshes: list[trimesh.Trimesh] = []

    def caja(x0, x1, y0, y1, z_top):
        dims = (x1 - x0, y1 - y0, z_top - z_base)
        centro = (cx + 0.5 * (x0 + x1), cy + 0.5 * (y0 + y1), 0.5 * (z_top + z_base))
        meshes.append(trimesh.creation.box(dims, trimesh.transformations.translation_matrix(centro)))

    def anillo(r_int, r_ext, z_top):
        caja(-r_ext, r_ext, r_int, r_ext, z_top)     # norte
        caja(-r_ext, r_ext, -r_ext, -r_int, z_top)   # sur
        caja(-r_ext, -r_int, -r_int, r_int, z_top)   # oeste
        caja(r_int, r_ext, -r_int, r_int, z_top)     # este

    r = 0.5 * cfg.platform_width
    caja(-r, r, -r, r, 0.0)  # plataforma central donde nace el rover

    h = 0.0
    for subida in cfg.subidas:
        if r >= semilado:
            print(f"[escalera_progresiva] AVISO: size={cfg.size} no da para todos los escalones")
            break
        h += subida
        r_ext = min(r + cfg.step_width, semilado)
        anillo(r, r_ext, h)
        r = r_ext
    if r < semilado:
        anillo(r, semilado, h)  # relleno plano hasta el borde

    print(f"[escalera_progresiva] {len(cfg.subidas)} escalones de {cfg.subidas[0] * 100:.0f} a "
          f"{cfg.subidas[-1] * 100:.0f} cm, altura total {h:.2f} m, radio util {r:.1f} m")
    return meshes, np.array([cx, cy, 0.0])


@configclass
class EscaleraProgresivaCfg(SubTerrainBaseCfg):
    function = escalera_progresiva
    # subida de cada escalon (m), del centro hacia fuera: de 2 en 2 hasta 11 (zona conocida)
    # y de 1 en 1 a partir de ahi, donde estaba el limite anterior
    subidas: tuple[float, ...] = (0.05, 0.07, 0.09, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18)
    step_width: float = 1.0        # huella: cabe el rover entero (0.66 m de batalla + margen)
    platform_width: float = 3.0    # plataforma central (m), debe cubrir el pose_range del reset
    grosor_base: float = 0.1       # cuanto baja la malla por debajo de z=0 para que no haya huecos


# ---- pendiente progresiva ----------------------------------------------------------------

def pendiente_progresiva(difficulty: float, cfg: PendienteProgresivaCfg) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Plataforma central a z=0 rodeada de anillos cuadrados en rampa, cada uno mas inclinado.

    Superficie de triangulos sin espesor (como el flat_terrain de Isaac Lab): cada anillo es
    un tronco de piramide cuadrada hecho de cuatro trapecios.
    """
    cx, cy = 0.5 * cfg.size[0], 0.5 * cfg.size[1]
    semilado = 0.5 * min(cfg.size)
    V: list[tuple[float, float, float]] = []
    F: list[list[int]] = []

    def cuad(p0, p1, p2, p3):  # antihorario visto desde arriba -> normal hacia +z
        i = len(V)
        V.extend([p0, p1, p2, p3])
        F.extend([[i, i + 1, i + 2], [i, i + 2, i + 3]])

    def banda(r0, h0, r1, h1):  # tronco de piramide cuadrada entre (r0,h0) y (r1,h1)
        cuad((-r0, r0, h0), (r0, r0, h0), (r1, r1, h1), (-r1, r1, h1))      # norte
        cuad((r0, r0, h0), (r0, -r0, h0), (r1, -r1, h1), (r1, r1, h1))      # este
        cuad((r0, -r0, h0), (-r0, -r0, h0), (-r1, -r1, h1), (r1, -r1, h1))  # sur
        cuad((-r0, -r0, h0), (-r0, r0, h0), (-r1, r1, h1), (-r1, -r1, h1))  # oeste

    r = 0.5 * cfg.platform_width
    cuad((-r, -r, 0.0), (r, -r, 0.0), (r, r, 0.0), (-r, r, 0.0))
    h, usados = 0.0, 0
    for ang in cfg.angulos_deg:
        if r >= semilado:
            print(f"[pendiente_progresiva] AVISO: size={cfg.size} no da para todas las rampas")
            break
        r1 = min(r + cfg.ancho, semilado)
        h1 = h + (r1 - r) * math.tan(math.radians(ang))
        banda(r, h, r1, h1)
        r, h, usados = r1, h1, usados + 1
    if r < semilado:
        banda(r, h, semilado, h)  # relleno plano hasta el borde

    verts = np.array(V, dtype=float)
    verts[:, 0] += cx
    verts[:, 1] += cy
    mesh = trimesh.Trimesh(vertices=verts, faces=np.array(F), process=False)
    print(f"[pendiente_progresiva] {usados} rampas de {cfg.angulos_deg[0]} a {cfg.angulos_deg[usados - 1]} grados, "
          f"altura total {h:.2f} m, radio util {r:.1f} m")
    return [mesh], np.array([cx, cy, 0.0])


@configclass
class PendienteProgresivaCfg(SubTerrainBaseCfg):
    function = pendiente_progresiva
    angulos_deg: tuple[float, ...] = (5, 10, 15, 20, 25, 30, 35, 40)
    ancho: float = 1.5             # ancho radial de cada rampa (m): cabe el rover entero
    platform_width: float = 3.0
```

Por que cajas en la escalera y triangulos sin espesor en la pendiente: las cajas dan caras verticales perfectas para los escalones; para rampas, una superficie de triangulos es lo mismo que hace `flat_terrain` de Isaac Lab y evita construir solidos inclinados. PhysX genera contactos contra mallas de triangulos sin importar el espesor, y el ray caster del height scan tambien.

---

## 2. Fase 2: `rover_escalera_env_cfg.py`

Hereda de `RoverEnvCfg` (fase 1) y cambia: terreno, comando, spawn, una recompensa, dos terminos de curriculum y una terminacion. Cada bloque lleva su porque.

```python
"""Fine-tuning del rover ROBERT en la escalera progresiva."""

import math
from collections.abc import Sequence

import torch

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.envs.mdp.commands.velocity_command import UniformVelocityCommand
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ManagerTermBase, RewardTermCfg, SceneEntityCfg
from isaaclab.terrains import FlatPatchSamplingCfg, TerrainGeneratorCfg
from isaaclab.utils import configclass

from .rover_env_cfg import RoverEnvCfg
from .terrenos_rover import EscaleraProgresivaCfg

# El root del USD queda 4.7 cm por debajo de la huella con las ruedas apoyadas: toda medida
# de altura del root hay que corregirla con esto.
_COTA_ROOT = 0.047
# Altura acumulada al inicio de cada huella (m) y subida del escalon que tiene delante (cm)
_HUELLAS = [0.0, 0.05, 0.12, 0.21, 0.32, 0.44, 0.57, 0.71, 0.86, 1.02, 1.19, 1.37]
_SUBIDA_CM = [5, 7, 9, 11, 12, 13, 14, 15, 16, 17, 18]


class ComandoRadial(UniformVelocityCommand):
    """Como UniformVelocityCommand, pero el rumbo apunta hacia fuera del pozo (+/- ABANICO).

    En un pozo, un rumbo aleatorio apunta la mitad de las veces hacia el centro y el rover
    desanda lo subido (run `escalera`). Hacia fuera siempre es subir. El abanico conserva
    aproximaciones frontales, diagonales y de esquina.
    """

    ABANICO = math.radians(60.0)

    def _resample_command(self, env_ids: Sequence[int]):
        super()._resample_command(env_ids)
        pos = self.robot.data.root_pos_w[env_ids, :2] - self._env.scene.env_origins[env_ids, :2]
        radial = torch.atan2(pos[:, 1], pos[:, 0])
        self.heading_target[env_ids] = radial + torch.empty_like(radial).uniform_(-self.ABANICO, self.ABANICO)


def altura_alcanzada(env, env_ids, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """z final respecto al origen del terreno, en el reset y antes de recolocar. Solo se registra."""
    asset = env.scene[asset_cfg.name]
    z = asset.data.root_pos_w[env_ids, 2] - env.scene.env_origins[env_ids, 2]
    return {"media": torch.mean(z), "max": torch.max(z)}


class AlturaGanada(ManagerTermBase):
    """Paga solo cuando el record de altura del episodio sube.

    Devuelve (z - z_max) / dt cuando z supera el maximo previo y 0 si no. Como el RewardManager
    multiplica por dt, la suma del episodio es exactamente la altura ganada (m) por el peso.
    En llano vale 0, balancearse no lo farmea (solo paga el record) y no rompe el control por
    comandos de velocidad que usara Nav2.

    Por que hace falta: con std=0.05 en el tracking, quedarse quieto contra un escalon cobra
    el 57-95 % de la recompensa de velocidad, y subir solo cuesta penalizaciones. Sin este
    termino la politica aprendio a aparcar contra el primer escalon (run `escalera2`).

    El rover se coloca 65 cm en el aire y cae: durante ESPERA_S tras el reset se sigue la z
    sin pagar; si no, el record quedaria en el aire y nunca se superaria (run `escalera4`,
    que entreno 2000 iteraciones con esta recompensa valiendo 0).
    """

    ESPERA_S = 1.0

    def __init__(self, cfg: RewardTermCfg, env):
        super().__init__(cfg, env)
        self.asset = env.scene[cfg.params["asset_cfg"].name]
        self.z_max = torch.zeros(env.num_envs, device=env.device)
        self.z_spawn = torch.zeros(env.num_envs, device=env.device)  # z asentado tras la espera
        self.espera = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self.pasos_espera = int(self.ESPERA_S / env.step_dt)

    def _z(self, env_ids=slice(None)):
        return self.asset.data.root_pos_w[env_ids, 2] - self._env.scene.env_origins[env_ids, 2]

    def reset(self, env_ids: Sequence[int] | None = None):
        if env_ids is None:
            env_ids = slice(None)
        self.z_max[env_ids] = self._z(env_ids)
        self.espera[env_ids] = self.pasos_espera

    def __call__(self, env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
        z = self._z()
        activo = self.espera <= 0
        recien = self.espera == 1  # ultimo paso de espera: el rover ya esta asentado en su huella
        self.z_spawn = torch.where(recien, z, self.z_spawn)
        ganancia = torch.where(activo, torch.clamp(z - self.z_max, min=0.0), torch.zeros_like(z))
        # mientras espera, el record sigue a la z actual (asi acaba en la huella tras la caida)
        self.z_max = torch.where(activo, torch.maximum(self.z_max, z), z)
        self.espera -= 1
        return ganancia / env.step_dt


def exito_por_escalon(env, env_ids, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    """Diagnostico: anillo donde nacio -> escalon que tenia delante -> ¿subio al menos eso (-2 cm)?

    Se registra en Curriculum/exito/escalon_Xcm y n_Xcm. Ojo: un termino de curriculum solo
    registra el ULTIMO lote de resets de cada iteracion (uno o dos rovers volcados), asi que
    estas tasas son una muestra pequeña y sesgada. La version acumulada esta en `ExitoRampa`
    (fase 3); las tendencias de esta valen, los numeros no.
    """
    premio = env.reward_manager.get_term_cfg("altura_ganada").func  # instancia de AlturaGanada
    z_ini = premio.z_spawn[env_ids]
    ganado = premio.z_max[env_ids] - z_ini
    huellas = torch.tensor(_HUELLAS, device=env.device)
    anillo = (torch.bucketize(z_ini + _COTA_ROOT + 0.02, huellas) - 1).clamp(0, len(_SUBIDA_CM) - 1)
    subidas = torch.tensor(_SUBIDA_CM, device=env.device, dtype=torch.float) / 100.0
    supero = ganado >= subidas[anillo] - 0.02
    salida = {}
    for k, cm in enumerate(_SUBIDA_CM):
        m = anillo == k
        if m.any():
            salida[f"escalon_{cm}cm"] = supero[m].float().mean()
            salida[f"n_{cm}cm"] = m.sum().float()
    return salida


ESCALERA_CFG = TerrainGeneratorCfg(
    size=(32.0, 32.0),      # 11 anillos de 1 m + plataforma de 3 m + relleno de 2.5 m
    border_width=0.0,
    num_rows=2,
    num_cols=2,
    use_cache=False,
    curriculum=False,
    sub_terrains={
        "escalera": EscaleraProgresivaCfg(
            proportion=1.0,
            # parches planos sobre las huellas para nacer en cualquier anillo; z_range corta
            # en el anillo 9 (1.02 m): desde ahi quedan el 17 y el 18 y no llega al borde en 40 s
            flat_patch_sampling={
                "init_pos": FlatPatchSamplingCfg(
                    num_patches=4000, patch_radius=0.45, max_height_diff=0.02, z_range=(-0.1, 1.05)
                )
            },
        )
    },
)


@configclass
class RoverEscaleraEnvCfg(RoverEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        # Nace repartido por la escalera y mira hacia fuera: 40 s son 3-4 anillos de intento
        # sin llegar al borde del sub-terreno (y caer al pozo vecino)
        self.episode_length_s = 40.0
        self.commands.base_velocity.class_type = ComandoRadial
        self.commands.base_velocity.resampling_time_range = (40.0, 40.0)

        self.scene.terrain.terrain_generator = ESCALERA_CFG
        self.scene.terrain.max_init_terrain_level = None
        # Con 0.5 (average) la rueda no se agarra a la cara del escalon. Goma sobre suelo duro: 0.8-1.0
        self.scene.terrain.physics_material = sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0, dynamic_friction=1.0, friction_combine_mode="max"
        )

        # Spawn sobre un parche plano de cualquier anillo, en vez de en el centro: con spawn
        # central el rover gastaba el 80 % del episodio en escalones ya dominados
        for nombre, term in vars(self.events).items():
            if term is not None and getattr(term, "func", None) is mdp.reset_root_state_uniform:
                setattr(self.events, nombre, None)
        self.events.reset_en_escalera = EventTerm(
            func=mdp.reset_root_state_from_terrain,
            mode="reset",
            params={
                "pose_range": {"yaw": (-math.pi, math.pi)},
                "velocity_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0),
                                   "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0)},
            },
        )

        # Peso 200: cada cm subido vale 2.0, ~1-2 s de tracking perfecto
        self.rewards.altura_ganada = RewardTermCfg(
            func=AlturaGanada, weight=200.0, params={"asset_cfg": SceneEntityCfg("robot")}
        )
        # el curriculo de niveles no aplica: todos los terrenos son iguales
        self.curriculum.terreno = None
        self.curriculum.altura = CurrTerm(func=altura_alcanzada)
        self.curriculum.exito = CurrTerm(func=exito_por_escalon)
        # Tocar el escalon con el chasis es inevitable al subir 9-12 cm: si ademas termina el
        # episodio, la politica aprende a no intentarlo. Queda la penalizacion; muere solo por vuelco
        self.terminations.encallado = None
```

### Como leer las metricas de la fase 2

- `Episode_Reward/altura_ganada`: Isaac Lab divide `Episode_Reward` entre `episode_length_s`, asi que **metros subidos por episodio = valor × 40 / 200 = valor × 0.2**. Un 0.13 son 2.6 cm de media.
- `Curriculum/altura/media` y `max`: z al final del episodio. Con spawn repartido ya no es la altura ganada, solo donde acaban.
- `Curriculum/exito/escalon_Xcm`: fraccion que supero ese escalon (muestra pequeña, ver arriba).
- Escalon k superado ⇔ altura acumulada de `_HUELLAS[k+1]` alcanzada.

---

## 3. Fase 3: `rover_pendiente_env_cfg.py`

Hereda de `RoverEscaleraEnvCfg`: conserva `AlturaGanada`, la friccion 1.0, los 40 s y el spawn repartido. Cambia el terreno, el abanico del comando, el reset (sobre la rampa, morro cuesta arriba) y la metrica.

```python
"""Prueba de pendientes: hasta que inclinacion sube el rover ROBERT."""

import math

import torch

import isaaclab.utils.math as math_utils
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ManagerTermBase, SceneEntityCfg
from isaaclab.terrains import FlatPatchSamplingCfg, TerrainGeneratorCfg
from isaaclab.utils import configclass

from .rover_escalera_env_cfg import ComandoRadial, RoverEscaleraEnvCfg, _COTA_ROOT
from .terrenos_rover import PendienteProgresivaCfg

# Rampas finas alrededor del limite encontrado en pendiente1 (25-30 grados).
# pendiente1 uso (5, 10, 15, 20, 25, 30, 35, 40).
ANGULOS = (20, 22, 24, 26, 28, 30, 32, 34)
ANCHO = 1.5
PLATAFORMA = 3.0
# altura a la que empieza cada rampa (m)
_INICIOS = [0.0]
for _a in ANGULOS:
    _INICIOS.append(_INICIOS[-1] + ANCHO * math.tan(math.radians(_a)))


class ComandoRadialPendiente(ComandoRadial):
    # cruzar una rampa en diagonal la suaviza; abanico estrecho para medir la inclinacion real
    ABANICO = math.radians(20.0)


def _rampa_y_cuesta(env, env_ids, pos_w):
    """Para cada posicion: indice de rampa (-1 = plataforma), angulo (rad) y direccion cuesta arriba (xy unitaria)."""
    d = pos_w[:, :2] - env.scene.env_origins[env_ids, :2]
    ax, ay = d[:, 0].abs(), d[:, 1].abs()
    r = torch.maximum(ax, ay)  # distancia Chebyshev: los anillos son cuadrados
    k = torch.floor((r - 0.5 * PLATAFORMA) / ANCHO).long()
    k = torch.where(r < 0.5 * PLATAFORMA, torch.full_like(k, -1), k.clamp(max=len(ANGULOS) - 1))
    angulos = torch.tensor([0.0] + [math.radians(a) for a in ANGULOS], device=env.device)
    theta = angulos[k + 1]
    # cuesta arriba = normal del lado del cuadrado en el que esta
    lado_x = ax >= ay
    cuesta = torch.stack([torch.where(lado_x, d[:, 0].sign(), 0.0), torch.where(lado_x, 0.0, d[:, 1].sign())], dim=1)
    return k, theta, cuesta


def reset_en_rampa(env, env_ids, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"), holgura: float = 0.10):
    """Coloca al rover sobre un parche de la rampa, mirando cuesta arriba y con el cabeceo de la rampa.

    Nacer 70 cm en el aire con guiñada aleatoria hacia volcar al aterrizar en 25-30 grados
    (visto en play.py de pendiente1): contaba como fallo y gastaba episodios en caidas.
    """
    asset = env.scene[asset_cfg.name]
    terrain = env.scene.terrain
    parches = terrain.flat_patches["init_pos"]
    ids = torch.randint(0, parches.shape[2], (len(env_ids),), device=env.device)
    pos = parches[terrain.terrain_levels[env_ids], terrain.terrain_types[env_ids], ids].clone()
    k, theta, cuesta = _rampa_y_cuesta(env, env_ids, pos)
    yaw = torch.atan2(cuesta[:, 1], cuesta[:, 0])
    yaw = torch.where(k < 0, torch.empty_like(yaw).uniform_(-math.pi, math.pi), yaw)  # en la plataforma, al azar
    quat = math_utils.quat_from_euler_xyz(torch.zeros_like(yaw), -theta, yaw)  # cabeceo negativo = morro arriba
    pos[:, 2] += -_COTA_ROOT + holgura
    asset.write_root_pose_to_sim(torch.cat([pos, quat], dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim(torch.zeros(len(env_ids), 6, device=env.device), env_ids=env_ids)


class ExitoRampa(ManagerTermBase):
    """Tasa de exito por rampa con olvido exponencial por llamada.

    Un termino de curriculum normal registra solo el ultimo lote de resets de la iteracion
    (1-3 rovers volcados): muestra minuscula y sesgada. Aqui se acumulan exitos y cuentas.
    Los resets gotean (los episodios NO estan sincronizados, aunque el time_out sea igual
    para todos), asi que el olvido va por llamada: ~4000 llamadas = ~2 episodios de toda la
    poblacion. Con 6144 entornos, n_Xdeg se estabiliza en 350-1400 por rampa.
    """

    OLVIDO = 0.9995

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        n = len(ANGULOS)
        self.exitos = torch.zeros(n, device=env.device)
        self.cuenta = torch.zeros(n, device=env.device)
        self.inicios = torch.tensor(_INICIOS, device=env.device)

    def __call__(self, env, env_ids, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
        premio = env.reward_manager.get_term_cfg("altura_ganada").func
        z_ini = premio.z_spawn[env_ids]
        z_max = premio.z_max[env_ids]
        rampa = (torch.bucketize(z_ini + _COTA_ROOT + 0.02, self.inicios) - 1).clamp(0, len(ANGULOS) - 1)
        supero = (z_max + _COTA_ROOT >= self.inicios[rampa + 1] - 0.03).float()
        self.exitos *= self.OLVIDO
        self.cuenta *= self.OLVIDO
        self.exitos += torch.bincount(rampa, weights=supero, minlength=len(ANGULOS))
        self.cuenta += torch.bincount(rampa, minlength=len(ANGULOS)).float()
        salida = {}
        for k, ang in enumerate(ANGULOS):
            if self.cuenta[k] > 0:
                salida[f"{ang}deg"] = self.exitos[k] / self.cuenta[k]
                salida[f"n_{ang}deg"] = self.cuenta[k]
        return salida


PENDIENTE_CFG = TerrainGeneratorCfg(
    size=(32.0, 32.0),
    border_width=0.0,
    num_rows=2,
    num_cols=2,
    use_cache=False,
    curriculum=False,
    color_scheme="height",  # bandas de color por rampa: en el viewport se distingue cual es cual
    sub_terrains={
        "pendiente": PendienteProgresivaCfg(
            proportion=1.0,
            angulos_deg=ANGULOS,
            ancho=ANCHO,
            platform_width=PLATAFORMA,
            # en rampa no hay parches "planos": se acepta cualquier punto de cualquier rampa;
            # z_range excluye el relleno plano de arriba (el anillo mas grande por area)
            flat_patch_sampling={
                "init_pos": FlatPatchSamplingCfg(
                    num_patches=4000, patch_radius=0.45, max_height_diff=1.0, z_range=(-0.1, _INICIOS[-1] - 0.05)
                )
            },
        )
    },
)


@configclass
class RoverPendienteEnvCfg(RoverEscaleraEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.terrain.terrain_generator = PENDIENTE_CFG
        self.commands.base_velocity.class_type = ComandoRadialPendiente
        self.events.reset_en_escalera = None
        self.events.reset_en_rampa = EventTerm(func=reset_en_rampa, mode="reset")
        self.curriculum.exito = None
        self.curriculum.rampa = CurrTerm(func=ExitoRampa)
```

Rampa k superada ⇔ el record de altura llega al inicio de la rampa k+1 (menos 3 cm). Como el rover puede nacer en mitad de la rampa, es "cruzo lo que le quedaba"; con 6144 rovers repartidos al azar, promedia bien.

---

## 4. Fase 4: `rover_denso_env_cfg.py`

Hereda de `RoverEnvCfg` (fase 1), no de la escalera: se quieren los rumbos aleatorios y el curriculum de niveles originales, sin `AlturaGanada`. Solo cambia el generador y la friccion.

```python
"""Terreno irregular denso: fosos, bloques, huecos y baches por todas partes."""

import isaaclab.sim as sim_utils
from isaaclab.terrains import (
    HfDiscreteObstaclesTerrainCfg,
    HfRandomUniformTerrainCfg,
    HfSteppingStonesTerrainCfg,
    MeshRandomGridTerrainCfg,
    TerrainGeneratorCfg,
)
from isaaclab.utils import configclass

from .rover_env_cfg import RoverEnvCfg

DENSO_CFG = TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=20,        # niveles de dificultad
    num_cols=12,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    curriculum=True,
    sub_terrains={
        # bloques y fosos (alturas positivas y negativas) de 3 a 14 cm segun nivel;
        # 120 en 8x8 m cubren ~35 % del suelo
        "obstaculos": HfDiscreteObstaclesTerrainCfg(
            proportion=0.35,
            obstacle_height_mode="choice",
            obstacle_width_range=(0.3, 0.8),
            obstacle_height_range=(0.03, 0.14),
            num_obstacles=120,
            platform_width=1.5,
        ),
        # losas con huecos entre ellas; el hueco crece con el nivel, 10 cm de hondo
        "piedras": HfSteppingStonesTerrainCfg(
            proportion=0.25,
            stone_height_max=0.04,
            stone_width_range=(0.4, 1.0),
            stone_distance_range=(0.05, 0.22),
            holes_depth=-0.10,
            platform_width=1.5,
        ),
        # rejilla de celdas de 45 cm a alturas aleatorias
        "rejilla": MeshRandomGridTerrainCfg(
            proportion=0.25,
            grid_width=0.45,
            grid_height_range=(0.02, 0.10),
            platform_width=1.5,
        ),
        # baches aleatorios cada 25 cm (este tipo no escala con el nivel)
        "rugoso": HfRandomUniformTerrainCfg(
            proportion=0.15,
            noise_range=(0.02, 0.08),
            noise_step=0.01,
            downsampled_scale=0.25,
        ),
    },
)


@configclass
class RoverDensoEnvCfg(RoverEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.terrain.terrain_generator = DENSO_CFG
        # 0.5 era pesimista; 0.8 es goma sobre suelo duro
        self.scene.terrain.physics_material = sim_utils.RigidBodyMaterialCfg(
            static_friction=0.8, dynamic_friction=0.8, friction_combine_mode="max"
        )
```

Conversion de nivel a dificultad de este terreno: `dificultad = nivel / 19`. Nivel 12 → 0.63 → bloques y fosos de ±3 + 0.63 × 11 = **±9.9 cm**, huecos de 5 + 0.63 × 17 = **15.7 cm**, celdas de hasta 7 cm.

---

## 5. Registro de las tres tareas (`__init__.py`)

Se añaden a continuacion del registro de la fase 1:

```python
gym.register(
    id="Isaac-Rover-Robert-Escalera-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rover_escalera_env_cfg:RoverEscaleraEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RoverPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Rover-Robert-Pendiente-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rover_pendiente_env_cfg:RoverPendienteEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RoverPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Rover-Robert-Denso-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rover_denso_env_cfg:RoverDensoEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RoverPPORunnerCfg",
    },
)
```

Las tres usan el mismo `RoverPPORunnerCfg` de la fase 1 (`experiment_name="rover_robert"`), asi que todos los runs caen en `logs/rsl_rl/rover_robert/` y `--load_run` puede apuntar a cualquiera.

---

## 6. Scripts de diagnostico (`scripts/`)

### `prueba_fisica_escalera.py`: ¿puede subir sin politica?

Traccion a tope en las seis llantas, direccion recta, sin red. Mide hasta que escalon sube el rover por pura fisica e imprime cuantos episodios terminaron y por que.

```python
"""Sin politica: traccion a tope y direccion recta. Mide hasta que escalon sube el rover por pura fisica."""

import argparse
import re

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="Isaac-Rover-Robert-Escalera-v0")
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--segundos", type=float, default=90.0)
parser.add_argument("--invertir", type=str, default="", help="regex de llantas que giran al reves (p.ej. 'llanta_izq.*')")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)  # guardar la referencia: si se pierde, el proceso muere al crear la escena
simulation_app = app_launcher.app

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg

env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
env = gym.make(args.task, cfg=env_cfg)
env.reset()
base = env.unwrapped

# accion constante: direccion en 0 (recto), todo lo demas a +1 (a tope)
accion = torch.zeros(base.num_envs, base.action_manager.total_action_dim, device=base.device)
i = 0
for nombre in base.action_manager.active_terms:
    term = base.action_manager.get_term(nombre)
    joints = getattr(term, "_joint_names", [])
    print(f"[accion] termino '{nombre}' dim {term.action_dim}: {joints}", flush=True)
    if "direccion" not in nombre:
        for j, jn in enumerate(joints):
            accion[:, i + j] = -1.0 if (args.invertir and re.match(args.invertir, jn)) else 1.0
    i += term.action_dim

robot = base.scene["robot"]
origen = base.scene.env_origins
z_max = torch.full((base.num_envs,), -1.0, device=base.device)
asentado = int(3.0 / base.step_dt)  # ignorar la caida desde la altura de spawn
pasos = int(args.segundos / base.step_dt)
cada = int(10.0 / base.step_dt)
terminaciones = {n: 0 for n in base.termination_manager.active_terms}
for k in range(pasos):
    with torch.inference_mode():
        env.step(accion)
    for n in terminaciones:
        terminaciones[n] += int(base.termination_manager.get_term(n).sum().item())
    z = robot.data.root_pos_w[:, 2] - origen[:, 2]
    if k >= asentado:
        z_max = torch.maximum(z_max, z)
    if k % cada == 0:
        dist = torch.norm(robot.data.root_pos_w[:, :2] - origen[:, :2], dim=1)
        print(f"t={k * base.step_dt:4.0f}s     z: " + " ".join(f"{v:5.2f}" for v in z.tolist()), flush=True)
        print(f"        dist: " + " ".join(f"{v:5.2f}" for v in dist.tolist()), flush=True)

print("\nterminaciones acumuladas:", terminaciones, flush=True)
print("z_max final por rover:", " ".join(f"{v:.2f}" for v in z_max.tolist()), flush=True)
print("mejor:", f"{z_max.max().item():.2f} m  (en llano ~ -0.05)", flush=True)
env.close()
simulation_app.close()
```

```bash
./isaaclab.sh -p scripts/prueba_fisica_escalera.py --num_envs 16 --segundos 90 --headless 2>&1 | tail -40
```

Lo que se vio: los 16 rovers avanzan hasta el borde de la plataforma (`dist` 1.33-1.68 m, el escalon esta a 1.5 m) y se quedan en llano (`z` = -0.04) sin subir ni el escalon de 2 cm. La sospecha es que `encallado` (contacto de `base_link`) los reseteaba al morder el borde y en 8-9 s volvian a estar contra la pared; el contador de terminaciones se añadio para confirmarlo y no se llego a ejecutar. Queda como trabajo futuro.

### `ver_parches.py`: ¿donde nacen los rovers?

Histograma de alturas de los parches de spawn. Sirvio para descubrir que la escalera tenia una lista de escalones vieja (el maximo salia en 1.11 m en vez de 1.37) y que el relleno plano de arriba se llevaba el 40 % de los spawns en pendientes.

```python
import argparse
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="Isaac-Rover-Robert-Escalera-v0")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
launcher = AppLauncher(args)  # guardar la referencia, no solo .app
app = launcher.app

import gymnasium as gym
import torch
import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg

cfg = parse_env_cfg(args.task, device=args.device, num_envs=4)
env = gym.make(args.task, cfg=cfg)
p = env.unwrapped.scene.terrain.flat_patches["init_pos"]  # (filas, cols, N, 3)
z = (p[..., 2] - env.unwrapped.scene.terrain.terrain_origins[..., 2:3]).flatten()
# app.close() termina el proceso con os._exit y se pierde stdout: escribir a archivo
out = open("/tmp/parches.txt", "w")
print("parches:", z.numel(), " z min/max:", f"{z.min():.2f} {z.max():.2f}", file=out)
for lo in torch.arange(0.0, 1.4, 0.1):
    n = ((z >= lo - 0.05) & (z < lo + 0.05)).sum().item()
    print(f"  z~{lo:.1f}: {n}", file=out)
out.close()
env.close(); app.close()
```

```bash
./isaaclab.sh -p scripts/ver_parches.py --headless > /tmp/parches.log 2>&1; cat /tmp/parches.txt
```

---

## 7. Comandos de las fases 2 a 4

Prueba visual de cualquier tarea antes de gastar GPU (16 rovers, 20 iteraciones, con viewport):

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Rover-Robert-Escalera-v0 --num_envs 16 --resume --load_run '.*_clip90_cero' --run_name prueba_visual --max_iterations 20
```

Fase 2, escalera (desde el checkpoint base):

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Rover-Robert-Escalera-v0 --headless --num_envs 6144 --resume --load_run '.*_clip90_cero' --run_name escalera5 --max_iterations 2000
```

Fase 3, pendientes (linea base sin entrenar, entrenamiento, y rampas finas):

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Rover-Robert-Pendiente-v0 --headless --num_envs 6144 --resume --load_run '.*_clip90_cero' --run_name pend_diag --max_iterations 250
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Rover-Robert-Pendiente-v0 --headless --num_envs 6144 --resume --load_run '.*_pend_diag' --run_name pendiente1 --max_iterations 2000
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Rover-Robert-Pendiente-v0 --headless --num_envs 6144 --resume --load_run '.*_pendiente1' --run_name pendiente2 --max_iterations 1500
```

Fase 4, terreno denso:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Rover-Robert-Denso-v0 --headless --num_envs 6144 --resume --load_run '.*_clip90_cero' --run_name denso1 --max_iterations 3000
```

Reproducir cualquier run a velocidad real:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Rover-Robert-Denso-v0 --num_envs 16 --load_run '.*_denso1' --real-time
```

Detalles de `--load_run`: es un regex con `re.match`, anclado al principio del nombre. `'escalera'` no casa con `2026-09-12_..._escalera`; `'.*_escalera'` si. Siempre entre comillas.

Un diagnostico con `ExitoRampa` necesita al menos dos episodios completos para ser representativo: con episodios de 40 s y 24 pasos de 0.02 s por iteracion, son 84 iteraciones por episodio → minimo 170-250 iteraciones.

---

## 8. Cuadro de runs

| Run | Tarea | Desde | Iteraciones | Que se aprendio |
| --- | --- | --- | --- | --- |
| `2026-09-10_12-42-31` | v0 | cero | 10 000 | Base original (con normalizacion de observaciones) |
| `clip90_cero` | v0 | cero | 8000 | Base con clip ±90°; identico → el clip no cuesta |
| `escalera` | Escalera | base original | 2000 | Rumbos aleatorios: sube y baja; altura ~0 |
| `escalera2` | Escalera | base original | 2000 | Rumbo fijo: aparca contra el escalon |
| `escalera4` | Escalera | clip90_cero | 2000 | `AlturaGanada` valia 0 (bug del reset) |
| `escalera5` | Escalera | clip90_cero | 2000 | Diagnostico por escalon; 2.6 cm/episodio |
| `escalera6` | Escalera | escalera5 | 3000 | Sin `encallado`: identico. Rueda media bloquea |
| `pend_diag` | Pendiente | clip90_cero | 250 | Linea base: 5-15° al 90 %, 20° al 45 % |
| `pendiente1` | Pendiente | pend_diag | 2000 | 20° 63 %, 25° 31 %, 30° 16 % |
| `pendiente2` | Pendiente | pendiente1 | 1500 | Rampas finas: 22-24° con solvencia; mejor `model_11000.pt` |
| `denso1` | Denso | clip90_cero | 3000 | Nivel 12/20; candidata final |

Bench en la RTX 5080 (16 GB): 4096 entornos → 2.88 s/iteracion (34 000 pasos/s); 6144 → 3.60 s (41 000 pasos/s); 8192 no compensa. 2000 iteraciones con 6144 son unas 2 h.
