# Referencia completa: tarea `Isaac-Rover-Robert-v0`

Codigo completo de la fase 1 tal como quedo tras todas las correcciones. Cada archivo se crea desde la raiz de Isaac Lab con:

```bash
cd ~/Github/IsaacLab
BASE=source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/rover
mkdir -p $BASE/agents
cat > $BASE/NOMBRE << 'EOF'
(contenido)
EOF
```

El `'EOF'` con comillas simples evita que bash interprete el contenido. Comprobar la sintaxis despues de pegar:

```bash
python3 -c "import ast; ast.parse(open('$BASE/rover_env_cfg.py').read()); print('sintaxis OK')"
```

---

## 1. Registro de la tarea

Archivo:

```text
source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/rover/__init__.py
```

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

Archivo:

```text
source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/rover/agents/__init__.py
```

```python
from . import rsl_rl_ppo_cfg
```

---

## 2. Hiperparametros de PPO

Archivo:

```text
source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/rover/agents/rsl_rl_ppo_cfg.py
```

```python
from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class RoverPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 10000
    save_interval = 250
    experiment_name = "rover_robert"

    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        # Las observaciones van de 0.1 (velocidades) a 50 (joint_vel): sin
        # normalizar, esa disparidad desestabiliza los gradientes y acaba
        # reventando el parametro de desviacion tipica de la politica
        actor_obs_normalization=True,
        critic_obs_normalization=True,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
    )

    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=5.0e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=0.5,
    )
```

---

## 3. Entorno completo

Archivo:

```text
source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/rover/rover_env_cfg.py
```

```python
"""Entorno de RL para el rover ROBERT: superar obstaculos sin perder la trayectoria."""

import math

import torch

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
import isaaclab.terrains as terrain_gen
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import TerrainGeneratorCfg, TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

USD_ROVER = "/home/talos/IsaacsimManuel/ROBERT/robert_RL.usd"

RADIO_RUEDA = 0.0815
VEL_RUEDA_MAX = 2.045
PAR_RUEDA_MAX = 4.41
ANCHO_PELDANO = 0.9

# 3 cm (0.37 radios) hasta 25 cm (1.5 diametros, el limite del rocker-bogie).
# El nivel 0 debe ser superable casi por accidente: sin eso la exploracion
# nunca descubre que trepar paga.
ALTURA_MIN = 0.03
ALTURA_MAX = 0.25


def height_scan_seguro(env, sensor_cfg: SceneEntityCfg, offset: float = 0.5) -> torch.Tensor:
    """El raycaster devuelve inf cuando un rayo no impacta, y clamp no limpia los NaN."""
    v = mdp.height_scan(env, sensor_cfg, offset)
    return torch.nan_to_num(v, nan=0.0, posinf=1.0, neginf=-1.0)


def estado_invalido(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Corta el episodio si la fisica se desestabiliza, antes de que el NaN llegue a la red.

    Incluye los joints: el que sacamos de la articulacion para cerrar el lazo lo
    resuelve el solver de cuerpos rigidos y puede dispararse sin que el chasis lo note.
    """
    asset = env.scene[asset_cfg.name]
    vel = asset.data.root_lin_vel_w
    ang = asset.data.root_ang_vel_w
    malo = torch.any(~torch.isfinite(vel), dim=1)
    malo |= torch.any(~torch.isfinite(ang), dim=1)
    malo |= torch.norm(vel, dim=1) > 10.0
    malo |= torch.norm(ang, dim=1) > 50.0
    malo |= torch.any(~torch.isfinite(asset.data.joint_pos), dim=1)
    malo |= torch.any(~torch.isfinite(asset.data.joint_vel), dim=1)
    malo |= torch.any(torch.abs(asset.data.joint_vel) > 200.0, dim=1)
    return malo


def niveles_terreno_rover(
    env, env_ids: torch.Tensor, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Curriculum con banda neutra, calibrado para un rover de 0.167 m/s.

    terrain_levels_vel degrada a todo el que no supere la mitad del terreno y no
    deja zona intermedia: para un robot lento eso es una caida constante de nivel.
    """
    asset = env.scene[asset_cfg.name]
    terrain = env.scene.terrain
    distancia = torch.norm(
        asset.data.root_pos_w[env_ids, :2] - env.scene.env_origins[env_ids, :2], dim=1
    )
    sube = distancia > 1.5
    baja = distancia < 0.5
    baja *= ~sube
    terrain.update_env_origins(env_ids, sube, baja)
    return torch.mean(terrain.terrain_levels.float())


def nivel_maximo(env, env_ids: torch.Tensor) -> torch.Tensor:
    """Solo reporta. Con el minimo revela si unos tipos de terreno avanzan y otros no."""
    return torch.max(env.scene.terrain.terrain_levels.float())


def nivel_minimo(env, env_ids: torch.Tensor) -> torch.Tensor:
    return torch.min(env.scene.terrain.terrain_levels.float())


ROVER_TERRAIN = TerrainGeneratorCfg(
    # 5 m y no 8: promocionar exige recorrer parte del terreno,
    # y a 0.167 m/s cuatro metros eran inalcanzables
    size=(5.0, 5.0),
    border_width=20.0,
    num_rows=20,
    num_cols=20,
    horizontal_scale=0.05,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    curriculum=True,
    sub_terrains={
        "subir": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.35,
            step_height_range=(ALTURA_MIN, ALTURA_MAX),
            step_width=ANCHO_PELDANO,
            platform_width=2.0,
            border_width=1.0,
            holes=False,
        ),
        "bajar": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.35,
            step_height_range=(ALTURA_MIN, ALTURA_MAX),
            step_width=ANCHO_PELDANO,
            platform_width=2.0,
            border_width=1.0,
            holes=False,
        ),
        "obstaculos": terrain_gen.HfDiscreteObstaclesTerrainCfg(
            proportion=0.30,
            obstacle_height_mode="choice",
            obstacle_width_range=(0.15, 0.4),
            obstacle_height_range=(ALTURA_MIN, ALTURA_MAX),
            num_obstacles=20,
            platform_width=2.0,
        ),
    },
)


ROVER_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=USD_ROVER,
        activate_contact_sensors=True,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            # 32/8: el lazo cerrado con los union_sus de 43 g entre piezas
            # de kilos necesita mas iteraciones que un arbol normal
            enabled_self_collisions=False,
            solver_position_iteration_count=32,
            solver_velocity_iteration_count=8,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.65),
        joint_pos={".*": 0.0},
        joint_vel={".*": 0.0},
    ),
    actuators={
        "traccion": ImplicitActuatorCfg(
            joint_names_expr=["llanta.*"],
            effort_limit_sim=PAR_RUEDA_MAX,
            velocity_limit_sim=VEL_RUEDA_MAX,
            stiffness=0.0,
            damping=2.16,
        ),
        "direccion": ImplicitActuatorCfg(
            joint_names_expr=["reductor.*"],
            effort_limit_sim=20.0,
            velocity_limit_sim=3.0,
            stiffness=100.0,
            damping=10.0,
        ),
        # effort_limit 0: la suspension es pasiva, la politica no la acciona
        "suspension": ImplicitActuatorCfg(
            joint_names_expr=["suspension", "hombro.*", "codo.*", "union_sus.*"],
            effort_limit_sim=0.0,
            velocity_limit_sim=10.0,
            stiffness=0.0,
            damping=2.0,
        ),
    },
)


@configclass
class RoverSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=ROVER_TERRAIN,
        max_init_terrain_level=0,
        collision_group=-1,
    )

    robot = ROVER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/cuerpo_suspension/base_link",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 1.0)),
        # yaw: la rejilla no se inclina con el chasis al trepar
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[1.0, 0.85]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )

    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/cuerpo_suspension/.*",
        history_length=3,
        track_air_time=True,
    )

    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(intensity=750.0),
    )


@configclass
class CommandsCfg:
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        # 30-60 s y no 10: con seis rumbos por episodio el rover hacia un
        # paseo aleatorio y acababa cerca del origen
        resampling_time_range=(30.0, 60.0),
        rel_standing_envs=0.02,
        rel_heading_envs=1.0,
        heading_command=True,
        heading_control_stiffness=0.5,
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.05, 0.167),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(-0.3, 0.3),
            heading=(-math.pi, math.pi),
        ),
    )


@configclass
class ActionsCfg:
    traccion = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=["llanta.*"],
        scale=VEL_RUEDA_MAX,
        use_default_offset=False,
    )
    direccion = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["reductor.*"],
        scale=math.pi / 2,
        use_default_offset=True,
        # OJO: sin clip. El limite de ±90 grados se acordo el 2026-09-12 pero el parche
        # nunca llego a este archivo (se pego en una copia de la raiz). Verificado el
        # 2026-09-19 con referencia_reproducible.py: clip=null en clip90_cero, denso1 y
        # pendiente2. La red saca hasta ±3 en crudo, o sea ±270 grados. Se corrige en la
        # tarea nueva de la siguiente fase, no aqui.
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1), clip=(-10.0, 10.0))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2), clip=(-10.0, 10.0))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01), clip=(-10.0, 10.0))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.5, n_max=0.5), clip=(-50.0, 50.0))
        actions = ObsTerm(func=mdp.last_action)
        height_scan = ObsTerm(
            func=height_scan_seguro,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            noise=Unoise(n_min=-0.02, n_max=0.02),
            clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.2, 0.2), "y": (-0.2, 0.2), "yaw": (-3.14, 3.14)},
            "velocity_range": {"x": (0.0, 0.0), "y": (0.0, 0.0)},
        },
    )
    reset_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={"position_range": (1.0, 1.0), "velocity_range": (0.0, 0.0)},
    )
    randomize_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "mass_distribution_params": (0.8, 1.2),
            "operation": "scale",
        },
    )
    randomize_friction = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="llanta.*"),
            "static_friction_range": (0.6, 1.1),
            "dynamic_friction_range": (0.5, 1.0),
            "restitution_range": (0.2, 0.4),
            "num_buckets": 64,
        },
    )


@configclass
class RewardsCfg:
    # std 0.05, no 0.25: el rover va a 0.167 m/s y con std grande
    # quedarse quieto ya cobraba el 96% de la recompensa
    seguir_velocidad = RewTerm(
        func=mdp.track_lin_vel_xy_exp,
        weight=3.0,
        params={"command_name": "base_velocity", "std": 0.05},
    )
    seguir_rumbo = RewTerm(
        func=mdp.track_ang_vel_z_exp,
        weight=1.5,
        params={"command_name": "base_velocity", "std": 0.1},
    )
    penaliza_balanceo = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.01)
    penaliza_par = RewTerm(func=mdp.joint_torques_l2, weight=-2.5e-6)
    penaliza_aceleracion = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-8)
    penaliza_cambio_accion = RewTerm(func=mdp.action_rate_l2, weight=-0.005)
    # Todo el rover menos las llantas: solo las ruedas deben tocar el suelo
    penaliza_contacto_indebido = RewTerm(
        func=mdp.undesired_contacts,
        weight=-2.0,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces",
                body_names=["base_link", "hombro.*", "codo.*", "alma.*", "suspension.*", "union.*"],
            ),
            "threshold": 1.0,
        },
    )


@configclass
class TerminationsCfg:
    tiempo_agotado = DoneTerm(func=mdp.time_out, time_out=True)
    # 60 grados: inclinarse para trepar es legitimo, volcarse no
    volcado = DoneTerm(
        func=mdp.bad_orientation,
        params={"limit_angle": math.pi / 3},
    )
    encallado = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="base_link"),
            "threshold": 1.0,
        },
    )
    inestable = DoneTerm(func=estado_invalido)


@configclass
class CurriculumCfg:
    terreno = CurrTerm(func=niveles_terreno_rover)
    nivel_max = CurrTerm(func=nivel_maximo)
    nivel_min = CurrTerm(func=nivel_minimo)


@configclass
class RoverEnvCfg(ManagerBasedRLEnvCfg):
    scene: RoverSceneCfg = RoverSceneCfg(num_envs=2048, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        # dt 1/400 con decimation 8: el paso de control sigue siendo 0.02 s,
        # pero la fisica resuelve el doble de fino para el lazo cerrado
        self.decimation = 8
        self.episode_length_s = 60.0
        self.sim.dt = 0.0025
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt
```

---

## 4. Script de prueba antes de entrenar

Archivo, en la raiz de Isaac Lab:

```text
probar_rover.py
```

Instancia el entorno con pocos rovers, les manda velocidad fija a las ruedas y muestra los joints encontrados y las dimensiones de observacion y accion. Si los rovers avanzan, toda la cadena funciona.

```python
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=16)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

from isaaclab.envs import ManagerBasedRLEnv

from isaaclab_tasks.manager_based.locomotion.velocity.config.rover.rover_env_cfg import RoverEnvCfg


def main():
    cfg = RoverEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.scene.height_scanner.debug_vis = True
    env = ManagerBasedRLEnv(cfg=cfg)

    print("")
    print("JOINTS ENCONTRADOS: " + str(env.scene["robot"].joint_names))
    print("DIM OBSERVACION: " + str(env.observation_manager.group_obs_dim))
    print("DIM ACCION: " + str(env.action_manager.total_action_dim))
    print("")

    n = 0
    env.reset()
    while simulation_app.is_running():
        with torch.inference_mode():
            acciones = torch.zeros_like(env.action_manager.action)
            acciones[:, :6] = 0.6
            obs, rew, term, trunc, info = env.step(acciones)
            n = n + 1
            if n % 200 == 0:
                print("paso " + str(n) + "   recompensa media: " + str(round(rew.mean().item(), 3)))
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
```

```bash
./isaaclab.sh -p probar_rover.py --num_envs 16
```

Salida esperada:

```text
JOINTS ENCONTRADOS: ['hombro_der', 'hombro_izq', 'suspension', 'codo_der', 'reductor_der_d', 'codo_izq', 'reductor_izq_d', 'union_sus_der:0', 'union_sus_der:1', 'union_sus_der:2', 'union_sus_izq:0', 'union_sus_izq:1', 'union_sus_izq:2', 'llanta_der_m_', 'reductor_der_t_', 'llanta_der_d_', 'llanta_izq_m', 'reductor_izq_t', 'llanta_izq_d', 'llanta_der_t', 'llanta_izq_d_']
DIM OBSERVACION: {'policy': (442,)}
DIM ACCION: 10
```

---

## 5. Comandos

Entrenar de cero:

```bash
cd ~/Github/IsaacLab
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Rover-Robert-v0 --num_envs 2048 --headless
```

Reanudar el ultimo run:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Rover-Robert-v0 --num_envs 2048 --headless --resume
```

Reanudar desde un run concreto y dar nombre al nuevo:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Rover-Robert-v0 --num_envs 6144 --headless --resume --load_run '2026-09-10_12-42-31' --run_name escalera
```

TensorBoard:

```bash
./isaaclab.sh -p -m tensorboard.main --logdir logs/rsl_rl/rover_robert
```

Reproducir el ultimo checkpoint en el nivel 8 de terreno, acelerado:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Rover-Robert-v0 --num_envs 8 env.sim.render_interval=40 env.scene.terrain.max_init_terrain_level=8
```

Reproducir un checkpoint concreto:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Rover-Robert-v0 --num_envs 8 --load_run 2026-09-10_12-42-31 --checkpoint model_9999.pt
```

Listar runs y checkpoints:

```bash
ls -lt logs/rsl_rl/rover_robert/
ls logs/rsl_rl/rover_robert/NOMBRE_DEL_RUN/model_*.pt | sort -V
```

---

## 6. Conversion de nivel de terreno a centimetros

Con `num_rows=20` y `step_height_range=(0.03, 0.25)`:

```text
altura = 0.03 + nivel * (0.25 - 0.03) / 19 = 0.03 + nivel * 0.01158
```

| Nivel | Escalon |
| --- | --- |
| 0 | 3.0 cm |
| 2 | 5.3 cm |
| 4 | 7.6 cm |
| 5 | 8.8 cm (radio de rueda) |
| 8 | 12.3 cm |
| 12 | 16.9 cm |
| 16 | 21.5 cm |
| 19 | 25.0 cm |
