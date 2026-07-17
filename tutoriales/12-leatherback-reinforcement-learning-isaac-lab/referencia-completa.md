# Tutorial: Reinforcement Learning para un rover Ackermann tipo Leatherback en Isaac Lab

Este tutorial documenta el estado actual del ejemplo desarrollado: un robot tipo Ackermann/Leatherback entrenado con PPO en Isaac Lab para navegar hacia una meta visual, usando un LiDAR real por ray casting, obstáculos detectables por el sensor, rutas específicas de entrenamiento y gráficas de desempeño.

El objetivo es que este documento pueda subirse a GitHub junto con el código del entorno.

---

## 1. Objetivo del ejemplo

Se implementa una tarea de aprendizaje reforzado en Isaac Lab donde un vehículo tipo Ackermann debe:

1. iniciar desde una posición definida;
2. recibir una meta en el entorno, marcada con una esfera roja;
3. avanzar hacia la meta;
4. usar un LiDAR 2D tipo RPLIDAR para detectar paredes y obstáculos;
5. evitar obstáculos negros colocados en la escena;
6. recibir recompensa por acercarse y llegar a la meta;
7. recibir penalización por chocar, quedarse casi quieto, retroceder o atravesar obstáculos.

La escena usada para la fase actual es:

```text
/home/talos/isaacsim_assets/Assets/Isaac/5.0/Isaac/Environments/Grid/gridroom_curved.usd
```

El robot usado es:

```text
/home/talos/isaacsim_assets/Assets/Isaac/5.0/Isaac/Robots/NVIDIA/Leatherback/leatherback.usd
```

---

## 2. Requisitos previos

Este tutorial asume que ya existe una instalación funcional de Isaac Lab en:

```bash
~/Github/IsaacLab
```

También asume que se puede ejecutar Isaac Lab con:

```bash
cd ~/Github/IsaacLab
./isaaclab.sh -p -c "print('Isaac Lab OK')"
```

Además, los assets locales de Isaac Sim deben estar en:

```bash
/home/talos/isaacsim_assets/Assets/Isaac/5.0/Isaac
```

---

## 3. Estructura de archivos

La tarea se coloca dentro de `isaaclab_tasks/direct`:

```text
source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/
├── __init__.py
├── leatherback_navigation_env.py
└── agents/
    └── rsl_rl_ppo_cfg.py
```

Además, se incluye un script auxiliar en la raíz de Isaac Lab:

```text
plot_leatherback_training.py
```

---

## 4. Crear carpetas y archivos

Desde la raíz de Isaac Lab:

```bash
cd ~/Github/IsaacLab

mkdir -p source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/agents

touch source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/__init__.py

touch source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/leatherback_navigation_env.py

touch source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/agents/rsl_rl_ppo_cfg.py
```

---

## 5. Registro de la tarea: `__init__.py`

Archivo:

```text
source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/__init__.py
```

Código completo:

```python
# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""
Leatherback autonomous navigation environment.
"""

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

gym.register(
    id="Isaac-Leatherback-Navigation-Direct-v0",
    entry_point=f"{__name__}.leatherback_navigation_env:LeatherbackNavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.leatherback_navigation_env:LeatherbackNavigationEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:LeatherbackNavigationPPORunnerCfg",
    },
)
```

Este archivo permite lanzar la tarea con:

```bash
--task Isaac-Leatherback-Navigation-Direct-v0
```

---

## 6. Configuración PPO: `rsl_rl_ppo_cfg.py`

Archivo:

```text
source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/agents/rsl_rl_ppo_cfg.py
```

Código completo:

```python
# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class LeatherbackNavigationPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 1500
    save_interval = 20
    experiment_name = "leatherback_navigation_direct"

    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_obs_normalization=True,
        critic_obs_normalization=True,
        actor_hidden_dims=[64, 64],
        critic_hidden_dims=[64, 64],
        activation="elu",
    )

    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=3.0e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
```

En este punto se usa una red pequeña `[64, 64]` para evitar una política demasiado compleja. La política controla dos acciones:

```text
acción 0: dirección / steering
acción 1: aceleración hacia adelante / throttle
```

---

## 7. Entorno completo: `leatherback_navigation_env.py`

Archivo:

```text
source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/leatherback_navigation_env.py
```

Código completo consolidado:

```python
# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import math
from collections.abc import Sequence

import torch
import omni.usd
from pxr import UsdGeom

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import MultiMeshRayCaster, MultiMeshRayCasterCfg, patterns
from isaaclab.utils import configclass


TEST_OBSTACLE_POSITIONS_LOCAL = (
    # Obstáculo central pequeño: obliga a esquivar, pero no bloquea todo el camino.
    (0.45, 0.00),

    # Obstáculos laterales separados: hacen que el LiDAR vea objetos sin saturar la escena.
    (1.55, 1.15),
    (1.55, -1.15),
)

TRAINING_SCENE_USD_PATH = "/home/talos/isaacsim_assets/Assets/Isaac/5.0/Isaac/Environments/Grid/gridroom_curved.usd"

LEATHERBACK_USD_PATH = "/home/talos/isaacsim_assets/Assets/Isaac/5.0/Isaac/Robots/NVIDIA/Leatherback/leatherback.usd"


@configclass
class LeatherbackNavigationEnvCfg(DirectRLEnvCfg):
    # Environment
    seed = 42
    decimation = 4
    episode_length_s = 10.0
    action_space = 2
    observation_space = 80
    state_space = 0

    # Simulation
    sim: sim_utils.SimulationCfg = sim_utils.SimulationCfg(
        dt=1 / 120,
        render_interval=decimation,
        physx=sim_utils.PhysxCfg(
            enable_external_forces_every_iteration=True,
            min_velocity_iteration_count=1,
        ),
    )

    # Scene. The CLI argument --num_envs can override num_envs.
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4,
        env_spacing=4.0,
        replicate_physics=True,
        clone_in_fabric=False,
    )

    # Robot
    robot_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=LEATHERBACK_USD_PATH,
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.08),
            joint_pos={
                "Shock__Rear_Right": -0.03,
                "Shock__Rear_Left": -0.03,
                "Shock__Front_Right": 0.03,
                "Shock__Front_Left": 0.03,
            },
            joint_vel={".*": 0.0},
        ),
        actuators={
            "steering": ImplicitActuatorCfg(
                joint_names_expr=[
                    "Knuckle__Upright__Front_Left",
                    "Knuckle__Upright__Front_Right",
                ],
                effort_limit_sim=200.0,
                stiffness=1000.0,
                damping=100.0,
            ),
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=[
                    "Wheel__Knuckle__Front_Left",
                    "Wheel__Knuckle__Front_Right",
                    "Wheel__Upright__Rear_Left",
                    "Wheel__Upright__Rear_Right",
                ],
                effort_limit_sim=400.0,
                stiffness=0.0,
                damping=1000.0,
            ),
            "passive_suspension": ImplicitActuatorCfg(
                joint_names_expr=[
                    "Chassis__Arm_Rear_Lower_Right",
                    "Chassis__Arm_Rear_Lower_Left",
                    "Chassis__Arm_Front_Lower_Right",
                    "Chassis__Arm_Front_Lower_Left",
                    "Upright__Arm__Rear_Lower_Right",
                    "Shock__Arm__Rear_Lower_Right",
                    "Upright__Arm__Rear_Lower_Left",
                    "Shock__Arm__Rear_Lower_Left",
                    "Upright__Arm__Front_Lower_Right",
                    "Shock__Arm__Front_Lower_Right",
                    "Upright__Arm__Front_Lower_Left",
                    "Shock__Arm__Front_Lower_Left",
                    "Upright__Arm__Rear_Upper_Right",
                    "Shock__Rear_Right",
                    "Upright__Arm__Rear_Upper_Left",
                    "Shock__Rear_Left",
                    "Upright__Arm__Front_Upper_Right",
                    "Shock__Front_Right",
                    "Upright__Arm__Front_Upper_Left",
                    "Shock__Front_Left",
                ],
                effort_limit_sim=1.0,
                stiffness=0.0,
                damping=0.0,
            ),
        },
    )

    steer_joint_names = [
        "Knuckle__Upright__Front_Left",
        "Knuckle__Upright__Front_Right",
    ]

    wheel_joint_names = [
        "Wheel__Knuckle__Front_Left",
        "Wheel__Knuckle__Front_Right",
        "Wheel__Upright__Rear_Left",
        "Wheel__Upright__Rear_Right",
    ]

    # Vehicle control limits
    max_steer_angle = 0.42
    max_wheel_velocity = 14.0
    min_wheel_velocity = 2.0

    # Navigation
    goal_radius = 0.60
    near_goal_distance = 1.20
    min_goal_distance = 1.5
    max_goal_distance = 4.0

    # Real 2D LiDAR approximation for SLAMTEC RPLIDAR C1 / DFR1138.
    # Real sensor reference: 360 deg, around 12 m range.
    # Training version: 72 rays, 5 deg resolution.
    lidar_num_rays = 72
    lidar_max_distance = 12.0
    lidar_collision_distance = 0.01
    lidar_safety_distance = 0.01

    lidar_cfg = MultiMeshRayCasterCfg(
        prim_path="/World/envs/env_.*/Robot",
        update_period=0.1,
        offset=MultiMeshRayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.35)),
        mesh_prim_paths=[
            MultiMeshRayCasterCfg.RaycastTargetCfg(
                prim_expr="/World/Warehouse",
                is_shared=True,
                merge_prim_meshes=True,
                track_mesh_transforms=False,
            ),
            MultiMeshRayCasterCfg.RaycastTargetCfg(
                prim_expr="/World/TrainingObstacles",
                is_shared=True,
                merge_prim_meshes=True,
                track_mesh_transforms=False,
            ),
        ],
        ray_alignment="yaw",
        pattern_cfg=patterns.LidarPatternCfg(
            channels=1,
            vertical_fov_range=(0.0, 0.0),
            horizontal_fov_range=(-180.0, 180.0),
            horizontal_res=5.0,
        ),
        max_distance=12.0,
        debug_vis=False,
    )

    # Test obstacles visible to the real RayCaster LiDAR.
    test_obstacles_enabled = True
    test_obstacle_box_size = 0.18
    test_obstacle_height = 0.60
    test_obstacle_collision_distance = 0.32
    test_obstacle_safety_distance = 0.65

    # Numerical safety limits for PPO stability.
    max_abs_reward = 100.0
    max_distance_for_reward = 8.0
    max_progress_per_step = 0.35
    max_root_distance_from_origin = 8.0
    max_root_height = 2.0


class LeatherbackNavigationEnv(DirectRLEnv):
    cfg: LeatherbackNavigationEnvCfg

    def __init__(self, cfg: LeatherbackNavigationEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self.actions = torch.zeros((self.num_envs, self.cfg.action_space), device=self.device)
        self._previous_actions = torch.zeros_like(self.actions)

        self._target_pos_w = torch.zeros((self.num_envs, 2), device=self.device)
        self._previous_distance = torch.zeros(self.num_envs, device=self.device)

        self._test_obstacle_positions_local = torch.tensor(
            TEST_OBSTACLE_POSITIONS_LOCAL,
            device=self.device,
            dtype=torch.float32,
        )

        # Valid goal points for the curved grid room scene.
        self._valid_goal_points_local = torch.tensor(
            [
                [1.5, 0.0],
                [2.5, 0.0],
                [3.0, 0.8],
                [3.0, -0.8],
                [2.0, 1.5],
                [2.0, -1.5],
                [0.0, 1.8],
                [0.0, -1.8],
                [-1.5, 0.0],
                [-2.0, 1.0],
                [-2.0, -1.0],
            ],
            device=self.device,
        )

        # Specific training routes: [start_x, start_y, goal_x, goal_y].
        self._route_start_goal_local = torch.tensor(
            [
                # Rutas izquierda -> derecha
                [-2.0, 0.0, 2.4, 0.0],
                [-2.0, -0.5, 2.4, 0.5],
                [-2.0, 0.5, 2.4, -0.5],

                # Rutas derecha -> izquierda
                [2.4, 0.0, -2.0, 0.0],
                [2.4, 0.5, -2.0, -0.5],
                [2.4, -0.5, -2.0, 0.5],

                # Rutas diagonales suaves
                [-1.8, -0.9, 1.9, 0.9],
                [-1.8, 0.9, 1.9, -0.9],
            ],
            device=self.device,
        )

        self._steer_joint_ids, _ = self.robot.find_joints(self.cfg.steer_joint_names)
        self._wheel_joint_ids, _ = self.robot.find_joints(self.cfg.wheel_joint_names)

        print(f"[INFO] Leatherback steer joints: {self._steer_joint_ids}")
        print(f"[INFO] Leatherback wheel joints: {self._wheel_joint_ids}")

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)

        # Predetermined curved grid room environment.
        scene_cfg = sim_utils.UsdFileCfg(
            usd_path=TRAINING_SCENE_USD_PATH,
        )
        scene_cfg.func(
            "/World/Warehouse",
            scene_cfg,
            translation=(0.0, 0.0, 0.0),
        )

        self.scene.clone_environments(copy_from_source=False)

        # Visual target markers. These are updated on every reset.
        target_marker_cfg = sim_utils.SphereCfg(
            radius=0.25,
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(1.0, 0.0, 0.0),
            ),
        )

        self._target_marker_paths = []
        for env_id in range(self.cfg.scene.num_envs):
            marker_path = f"/World/TargetMarker_{env_id}"
            target_marker_cfg.func(
                marker_path,
                target_marker_cfg,
                translation=(0.0, 0.0, 0.25),
            )
            self._target_marker_paths.append(marker_path)

        # Static test obstacles. They are visual meshes, collision objects and RayCaster targets.
        self._test_obstacle_paths = []
        if self.cfg.test_obstacles_enabled:
            stage = omni.usd.get_context().get_stage()
            UsdGeom.Xform.Define(stage, "/World/TrainingObstacles")

            obstacle_cfg = sim_utils.CuboidCfg(
                size=(
                    self.cfg.test_obstacle_box_size,
                    self.cfg.test_obstacle_box_size,
                    self.cfg.test_obstacle_height,
                ),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.0, 0.0, 0.0),
                ),
            )

            local_obstacle_positions = TEST_OBSTACLE_POSITIONS_LOCAL

            for env_id in range(self.cfg.scene.num_envs):
                env_origin_xy = self.scene.env_origins[env_id, 0:2].detach().cpu().tolist()

                for obs_id, local_xy in enumerate(local_obstacle_positions):
                    obstacle_path = f"/World/TrainingObstacles/Obstacle_{env_id}_{obs_id}"

                    obstacle_cfg.func(
                        obstacle_path,
                        obstacle_cfg,
                        translation=(
                            float(env_origin_xy[0] + local_xy[0]),
                            float(env_origin_xy[1] + local_xy[1]),
                            self.cfg.test_obstacle_height / 2.0,
                        ),
                    )

                    self._test_obstacle_paths.append(obstacle_path)

        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])

        self.scene.articulations["robot"] = self.robot

        # Real ray-cast LiDAR sensor mounted on the robot.
        self.lidar = MultiMeshRayCaster(self.cfg.lidar_cfg)
        self.scene.sensors["lidar"] = self.lidar

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _update_target_markers(self, env_ids) -> None:
        if not hasattr(self, "_target_marker_paths"):
            return

        if isinstance(env_ids, torch.Tensor):
            env_ids_list = env_ids.detach().cpu().tolist()
        else:
            env_ids_list = list(env_ids)

        stage = omni.usd.get_context().get_stage()

        for env_id in env_ids_list:
            env_id = int(env_id)

            if env_id >= len(self._target_marker_paths):
                continue

            prim = stage.GetPrimAtPath(self._target_marker_paths[env_id])
            if not prim:
                continue

            target_xy = self._target_pos_w[env_id].detach().cpu().tolist()
            translation = (float(target_xy[0]), float(target_xy[1]), 0.25)

            xformable = UsdGeom.Xformable(prim)
            translate_op = None

            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                    break

            if translate_op is None:
                translate_op = xformable.AddTranslateOp()

            translate_op.Set(translation)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.actions = torch.clamp(actions, -1.0, 1.0)

    def _apply_action(self) -> None:
        steer_cmd = self.actions[:, 0:1] * self.cfg.max_steer_angle

        # Throttle solo hacia adelante.
        # La acción [-1, 1] se convierte en velocidad [min_wheel_velocity, max_wheel_velocity].
        # Esto evita que la política aprenda a quedarse casi quieta o a oscilar hacia atrás.
        throttle_01 = 0.5 * (self.actions[:, 1:2] + 1.0)
        throttle_01 = torch.clamp(throttle_01, 0.0, 1.0)
        wheel_cmd = self.cfg.min_wheel_velocity + throttle_01 * (
            self.cfg.max_wheel_velocity - self.cfg.min_wheel_velocity
        )

        steer_targets = steer_cmd.repeat(1, len(self._steer_joint_ids))
        wheel_targets = wheel_cmd.repeat(1, len(self._wheel_joint_ids))

        self.robot.set_joint_position_target(
            steer_targets,
            joint_ids=self._steer_joint_ids,
        )

        self.robot.set_joint_velocity_target(
            wheel_targets,
            joint_ids=self._wheel_joint_ids,
        )

    def _compute_real_lidar_observations(self) -> torch.Tensor:
        raw_ray_hits_w = self.lidar.data.ray_hits_w
        raw_sensor_pos_w = self.lidar.data.pos_w.unsqueeze(1)

        valid_hits = torch.isfinite(raw_ray_hits_w).all(dim=-1)
        valid_sensor = torch.isfinite(raw_sensor_pos_w).all(dim=-1)

        ray_hits_w = torch.nan_to_num(
            raw_ray_hits_w,
            nan=0.0,
            posinf=self.cfg.lidar_max_distance,
            neginf=-self.cfg.lidar_max_distance,
        )

        sensor_pos_w = torch.nan_to_num(
            raw_sensor_pos_w,
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )

        distances = torch.norm(ray_hits_w - sensor_pos_w, dim=-1)

        valid = valid_hits & valid_sensor
        max_dist = torch.full_like(distances, self.cfg.lidar_max_distance)

        distances = torch.where(valid, distances, max_dist)
        distances = torch.nan_to_num(
            distances,
            nan=self.cfg.lidar_max_distance,
            posinf=self.cfg.lidar_max_distance,
            neginf=self.cfg.lidar_max_distance,
        )
        distances = torch.clamp(distances, 0.0, self.cfg.lidar_max_distance)

        if distances.shape[1] > self.cfg.lidar_num_rays:
            distances = distances[:, : self.cfg.lidar_num_rays]
        elif distances.shape[1] < self.cfg.lidar_num_rays:
            pad = torch.full(
                (distances.shape[0], self.cfg.lidar_num_rays - distances.shape[1]),
                self.cfg.lidar_max_distance,
                device=self.device,
            )
            distances = torch.cat((distances, pad), dim=-1)

        lidar_obs = distances / self.cfg.lidar_max_distance

        lidar_obs = torch.nan_to_num(
            lidar_obs,
            nan=1.0,
            posinf=1.0,
            neginf=1.0,
        )

        return torch.clamp(lidar_obs, 0.0, 1.0)

    def _compute_min_lidar_distance(self) -> torch.Tensor:
        lidar = self._compute_real_lidar_observations()
        distances = lidar * self.cfg.lidar_max_distance
        distances = torch.nan_to_num(
            distances,
            nan=self.cfg.lidar_max_distance,
            posinf=self.cfg.lidar_max_distance,
            neginf=self.cfg.lidar_max_distance,
        )
        return torch.min(distances, dim=1).values

    def _compute_front_lidar_distance(self) -> torch.Tensor:
        lidar = self._compute_real_lidar_observations()
        distances = lidar * self.cfg.lidar_max_distance

        distances = torch.nan_to_num(
            distances,
            nan=self.cfg.lidar_max_distance,
            posinf=self.cfg.lidar_max_distance,
            neginf=self.cfg.lidar_max_distance,
        )

        num_rays = distances.shape[1]
        center = num_rays // 2
        half_width = max(1, num_rays // 8)

        front_distances = distances[:, center - half_width : center + half_width + 1]
        return torch.min(front_distances, dim=1).values

    def _get_observations(self) -> dict:
        root_pos = torch.nan_to_num(
            self.robot.data.root_pos_w[:, 0:2],
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )

        raw_quat = torch.nan_to_num(
            self.robot.data.root_quat_w,
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )

        quat_norm = torch.norm(raw_quat, dim=-1, keepdim=True)
        identity_quat = torch.zeros_like(raw_quat)
        identity_quat[:, 0] = 1.0

        root_quat = torch.where(
            quat_norm > 1.0e-6,
            raw_quat / torch.clamp(quat_norm, min=1.0e-6),
            identity_quat,
        )

        root_lin_vel = torch.nan_to_num(
            self.robot.data.root_lin_vel_w,
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )
        root_lin_vel = torch.clamp(root_lin_vel, -5.0, 5.0)

        root_ang_vel = torch.nan_to_num(
            self.robot.data.root_ang_vel_w,
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )
        root_ang_vel = torch.clamp(root_ang_vel, -10.0, 10.0)

        qw = root_quat[:, 0]
        qx = root_quat[:, 1]
        qy = root_quat[:, 2]
        qz = root_quat[:, 3]

        yaw = torch.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

        goal_vector = self._target_pos_w - root_pos
        goal_vector = torch.nan_to_num(
            goal_vector,
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )

        distance_to_goal = torch.norm(goal_vector, dim=-1)
        distance_to_goal = torch.nan_to_num(
            distance_to_goal,
            nan=self.cfg.max_distance_for_reward,
            posinf=self.cfg.max_distance_for_reward,
            neginf=self.cfg.max_distance_for_reward,
        )
        distance_to_goal = torch.clamp(
            distance_to_goal,
            0.0,
            self.cfg.max_distance_for_reward,
        )

        goal_heading = torch.atan2(goal_vector[:, 1], goal_vector[:, 0])

        heading_error = torch.atan2(
            torch.sin(goal_heading - yaw),
            torch.cos(goal_heading - yaw),
        )

        lidar = self._compute_real_lidar_observations()

        safe_actions = torch.nan_to_num(
            self.actions,
            nan=0.0,
            posinf=1.0,
            neginf=-1.0,
        )
        safe_actions = torch.clamp(safe_actions, -1.0, 1.0)

        obs = torch.cat(
            (
                root_lin_vel[:, 0:1],
                root_lin_vel[:, 1:2],
                root_ang_vel[:, 2:3],
                (distance_to_goal / self.cfg.max_goal_distance).unsqueeze(-1),
                torch.sin(heading_error).unsqueeze(-1),
                torch.cos(heading_error).unsqueeze(-1),
                safe_actions,
                lidar,
            ),
            dim=-1,
        )

        obs = torch.nan_to_num(
            obs,
            nan=0.0,
            posinf=1.0,
            neginf=-1.0,
        )

        obs = torch.clamp(obs, -10.0, 10.0)

        return {"policy": obs}

    def _compute_min_test_obstacle_distance(self) -> torch.Tensor:
        if not self.cfg.test_obstacles_enabled:
            return torch.full(
                (self.num_envs,),
                self.cfg.lidar_max_distance,
                device=self.device,
            )

        obstacle_pos_w = (
            self.scene.env_origins[:, 0:2].unsqueeze(1)
            + self._test_obstacle_positions_local.unsqueeze(0)
        )

        root_pos = torch.nan_to_num(
            self.robot.data.root_pos_w[:, 0:2],
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )

        distances = torch.norm(
            obstacle_pos_w - root_pos.unsqueeze(1),
            dim=-1,
        )

        distances = torch.nan_to_num(
            distances,
            nan=self.cfg.lidar_max_distance,
            posinf=self.cfg.lidar_max_distance,
            neginf=self.cfg.lidar_max_distance,
        )

        return torch.min(distances, dim=1).values

    def _get_rewards(self) -> torch.Tensor:
        root_pos = torch.nan_to_num(
            self.robot.data.root_pos_w[:, 0:2],
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )

        root_lin_vel = torch.nan_to_num(
            self.robot.data.root_lin_vel_w[:, 0:2],
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )
        root_lin_vel = torch.clamp(root_lin_vel, -5.0, 5.0)

        raw_quat = torch.nan_to_num(
            self.robot.data.root_quat_w,
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )

        quat_norm = torch.norm(raw_quat, dim=-1, keepdim=True)
        identity_quat = torch.zeros_like(raw_quat)
        identity_quat[:, 0] = 1.0

        root_quat = torch.where(
            quat_norm > 1.0e-6,
            raw_quat / torch.clamp(quat_norm, min=1.0e-6),
            identity_quat,
        )

        goal_vector = self._target_pos_w - root_pos
        goal_vector = torch.nan_to_num(
            goal_vector,
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )

        distance_to_goal = torch.norm(goal_vector, dim=-1)
        distance_to_goal = torch.nan_to_num(
            distance_to_goal,
            nan=self.cfg.max_distance_for_reward,
            posinf=self.cfg.max_distance_for_reward,
            neginf=self.cfg.max_distance_for_reward,
        )
        distance_to_goal = torch.clamp(
            distance_to_goal,
            0.0,
            self.cfg.max_distance_for_reward,
        )

        goal_dir = goal_vector / torch.clamp(distance_to_goal.unsqueeze(-1), min=1.0e-6)
        goal_heading = torch.atan2(goal_vector[:, 1], goal_vector[:, 0])

        qw = root_quat[:, 0]
        qx = root_quat[:, 1]
        qy = root_quat[:, 2]
        qz = root_quat[:, 3]

        yaw = torch.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

        heading_error = torch.atan2(
            torch.sin(goal_heading - yaw),
            torch.cos(goal_heading - yaw),
        )

        previous_distance = torch.nan_to_num(
            self._previous_distance,
            nan=self.cfg.max_distance_for_reward,
            posinf=self.cfg.max_distance_for_reward,
            neginf=self.cfg.max_distance_for_reward,
        )
        previous_distance = torch.clamp(
            previous_distance,
            0.0,
            self.cfg.max_distance_for_reward,
        )

        # Recompensa principal: reducir distancia a la meta.
        progress_reward = previous_distance - distance_to_goal
        progress_reward = torch.clamp(
            progress_reward,
            -self.cfg.max_progress_per_step,
            self.cfg.max_progress_per_step,
        )

        # Recompensa por velocidad real hacia la meta.
        velocity_to_goal = torch.sum(root_lin_vel * goal_dir, dim=-1)
        velocity_to_goal = torch.clamp(velocity_to_goal, -2.0, 2.0)

        forward_velocity_reward = torch.clamp(velocity_to_goal, min=0.0)
        backward_penalty = torch.clamp(-velocity_to_goal, min=0.0)

        # Penaliza quedarse casi quieto cuando todavía no ha llegado.
        low_speed = torch.clamp(0.35 - forward_velocity_reward, min=0.0)
        far_from_goal = distance_to_goal > self.cfg.goal_radius
        low_speed_penalty = torch.where(
            far_from_goal,
            low_speed,
            torch.zeros_like(low_speed),
        )

        # La orientación solo se usa como penalización, no como premio constante.
        heading_penalty = torch.abs(heading_error) / math.pi

        success_reward = torch.where(
            distance_to_goal < self.cfg.goal_radius,
            torch.full_like(distance_to_goal, 100.0),
            torch.zeros_like(distance_to_goal),
        )

        near_goal_factor = torch.clamp(
            (self.cfg.near_goal_distance - distance_to_goal) / max(float(self.cfg.near_goal_distance), 1.0e-6),
            min=0.0,
            max=1.0,
        )
        near_goal_reward = 15.0 * near_goal_factor * near_goal_factor

        min_lidar_distance = self._compute_min_lidar_distance()
        front_lidar_distance = self._compute_front_lidar_distance()

        wall_collision_penalty = torch.where(
            min_lidar_distance < self.cfg.lidar_collision_distance,
            torch.full_like(min_lidar_distance, 10.0),
            torch.zeros_like(min_lidar_distance),
        )

        safety_distance = max(float(self.cfg.lidar_safety_distance), 1.0e-6)

        wall_proximity_penalty = torch.clamp(
            safety_distance - front_lidar_distance,
            min=0.0,
        ) / safety_distance

        min_test_obstacle_distance = self._compute_min_test_obstacle_distance()

        obstacle_collision_penalty = torch.where(
            min_test_obstacle_distance < self.cfg.test_obstacle_collision_distance,
            torch.full_like(min_test_obstacle_distance, 60.0),
            torch.zeros_like(min_test_obstacle_distance),
        )

        obstacle_safety_distance = max(float(self.cfg.test_obstacle_safety_distance), 1.0e-6)

        obstacle_proximity_penalty = torch.clamp(
            obstacle_safety_distance - min_test_obstacle_distance,
            min=0.0,
        ) / obstacle_safety_distance

        if not hasattr(self, "_previous_actions"):
            self._previous_actions = torch.zeros_like(self.actions)

        safe_actions = torch.nan_to_num(
            self.actions,
            nan=0.0,
            posinf=1.0,
            neginf=-1.0,
        )
        safe_actions = torch.clamp(safe_actions, -1.0, 1.0)

        previous_actions = torch.nan_to_num(
            self._previous_actions,
            nan=0.0,
            posinf=1.0,
            neginf=-1.0,
        )
        previous_actions = torch.clamp(previous_actions, -1.0, 1.0)

        steering_penalty = 0.08 * torch.square(safe_actions[:, 0])
        throttle_penalty = 0.002 * torch.square(safe_actions[:, 1])

        action_rate_penalty = 0.03 * torch.sum(
            torch.square(safe_actions - previous_actions),
            dim=-1,
        )

        # Penalización temporal para evitar quedarse dudando.
        time_penalty = 0.08

        reward = (
            12.0 * progress_reward
            + 3.0 * forward_velocity_reward
            - 2.0 * backward_penalty
            - 2.0 * low_speed_penalty
            - 0.35 * heading_penalty
            + near_goal_reward
            + success_reward
            - wall_collision_penalty
            - 0.05 * wall_proximity_penalty
            - obstacle_collision_penalty
            - 0.80 * obstacle_proximity_penalty
            - steering_penalty
            - throttle_penalty
            - action_rate_penalty
            - time_penalty
        )

        reward = torch.nan_to_num(
            reward,
            nan=-self.cfg.max_abs_reward,
            posinf=self.cfg.max_abs_reward,
            neginf=-self.cfg.max_abs_reward,
        )
        reward = torch.clamp(
            reward,
            -self.cfg.max_abs_reward,
            self.cfg.max_abs_reward,
        )

        self._previous_distance = distance_to_goal.detach()
        self._previous_actions = safe_actions.detach().clone()

        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        raw_root_state = self.robot.data.root_state_w
        invalid_state = ~torch.isfinite(raw_root_state).all(dim=1)

        root_pos_3d = torch.nan_to_num(
            self.robot.data.root_pos_w,
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )

        root_pos = root_pos_3d[:, 0:2]

        env_origin_xy = self.scene.env_origins[:, 0:2]
        distance_from_origin = torch.norm(root_pos - env_origin_xy, dim=-1)

        out_of_bounds = distance_from_origin > self.cfg.max_root_distance_from_origin
        bad_height = torch.abs(root_pos_3d[:, 2]) > self.cfg.max_root_height

        distance_to_goal = torch.norm(
            self._target_pos_w - root_pos,
            dim=-1,
        )

        distance_to_goal = torch.nan_to_num(
            distance_to_goal,
            nan=self.cfg.max_distance_for_reward,
            posinf=self.cfg.max_distance_for_reward,
            neginf=self.cfg.max_distance_for_reward,
        )

        reached_goal = distance_to_goal < self.cfg.goal_radius

        min_lidar_distance = self._compute_min_lidar_distance()
        wall_collision = min_lidar_distance < self.cfg.lidar_collision_distance

        min_test_obstacle_distance = self._compute_min_test_obstacle_distance()
        obstacle_collision = min_test_obstacle_distance < self.cfg.test_obstacle_collision_distance

        time_out = self.episode_length_buf >= self.max_episode_length - 1

        terminated = (
            reached_goal
            | wall_collision
            | obstacle_collision
            | invalid_state
            | out_of_bounds
            | bad_height
        )

        return terminated, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES

        if isinstance(env_ids, torch.Tensor):
            env_ids = env_ids.to(device=self.device, dtype=torch.long)
        else:
            env_ids = torch.tensor(env_ids, device=self.device, dtype=torch.long)

        super()._reset_idx(env_ids)

        num_resets = len(env_ids)

        joint_pos = self.robot.data.default_joint_pos[env_ids].clone()
        joint_vel = self.robot.data.default_joint_vel[env_ids].clone()

        default_root_state = self.robot.data.default_root_state[env_ids].clone()

        route_ids = torch.randint(
            low=0,
            high=self._route_start_goal_local.shape[0],
            size=(num_resets,),
            device=self.device,
        )

        routes = self._route_start_goal_local[route_ids]

        start_offsets = routes[:, 0:2]
        target_offsets = routes[:, 2:4]

        env_origins_xy = self.scene.env_origins[env_ids, 0:2]

        start_pos_w = env_origins_xy + start_offsets
        target_pos_w = env_origins_xy + target_offsets

        self._target_pos_w[env_ids] = target_pos_w

        # Inicializar el robot mirando aproximadamente hacia la meta.
        start_to_goal = target_pos_w - start_pos_w
        yaw = torch.atan2(start_to_goal[:, 1], start_to_goal[:, 0])

        # Pequeña variación angular para que no memorice una sola orientación.
        yaw_noise = torch.empty(num_resets, device=self.device).uniform_(-0.20, 0.20)
        yaw = yaw + yaw_noise

        default_root_state[:, 0] = start_pos_w[:, 0]
        default_root_state[:, 1] = start_pos_w[:, 1]

        default_root_state[:, 3] = torch.cos(0.5 * yaw)
        default_root_state[:, 4] = 0.0
        default_root_state[:, 5] = 0.0
        default_root_state[:, 6] = torch.sin(0.5 * yaw)

        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

        root_pos = default_root_state[:, 0:2]

        self._previous_distance[env_ids] = torch.norm(
            self._target_pos_w[env_ids] - root_pos,
            dim=-1,
        )

        self._previous_actions[env_ids] = 0.0

        self._update_target_markers(env_ids)
```

---

## 8. Validar que la tarea quedó registrada

```bash
cd ~/Github/IsaacLab

./isaaclab.sh -p - <<'PY'
from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import gymnasium as gym
import isaaclab_tasks.direct.leatherback_navigation

spec = gym.spec("Isaac-Leatherback-Navigation-Direct-v0")
print("TAREA REGISTRADA CORRECTAMENTE:")
print(spec)

simulation_app.close()
PY
```

La salida esperada debe contener:

```text
TAREA REGISTRADA CORRECTAMENTE:
EnvSpec(id='Isaac-Leatherback-Navigation-Direct-v0', ...)
```

---

## 9. Probar estabilidad del entorno con 4 entornos

Antes de entrenar se recomienda verificar que no haya `NaN`, `Inf` o recompensas fuera de rango:

```bash
cd ~/Github/IsaacLab

./isaaclab.sh -p - <<'PY'
from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import torch
import gymnasium as gym

import isaaclab_tasks.direct.leatherback_navigation
from isaaclab_tasks.utils import parse_env_cfg

task_name = "Isaac-Leatherback-Navigation-Direct-v0"

env_cfg = parse_env_cfg(
    task_name,
    device="cuda:0",
    num_envs=4,
    use_fabric=False,
)

env = gym.make(task_name, cfg=env_cfg)
obs, info = env.reset()

print("RESET OK")
print("OBS SHAPE:", obs["policy"].shape)
print("ROOT POS:", env.unwrapped.robot.data.root_pos_w[:, 0:2])
print("TARGET POS:", env.unwrapped._target_pos_w)
print("MIN OBSTACLE DIST:", env.unwrapped._compute_min_test_obstacle_distance())

for i in range(3000):
    actions = 2.0 * torch.rand((4, 2), device=env.unwrapped.device) - 1.0
    obs, reward, terminated, truncated, info = env.step(actions)

    if not torch.isfinite(obs["policy"]).all():
        print("NaN/Inf en obs, step:", i)
        raise SystemExit(1)

    if not torch.isfinite(reward).all():
        print("NaN/Inf en reward, step:", i)
        raise SystemExit(1)

    if torch.max(torch.abs(reward)) > env.unwrapped.cfg.max_abs_reward:
        print("Reward fuera de rango, step:", i)
        print(reward)
        raise SystemExit(1)

print("PRUEBA OK: 3000 pasos sin NaN/Inf y reward acotado.")

env.close()
simulation_app.close()
PY
```

---

## 10. Entrenar la política PPO

Entrenamiento base recomendado:

```bash
cd ~/Github/IsaacLab

./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Leatherback-Navigation-Direct-v0 \
  --num_envs 4 \
  --max_iterations 1500 \
  --headless
```

También puede probarse con 8 entornos si el entorno sigue estable:

```bash
cd ~/Github/IsaacLab

./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Leatherback-Navigation-Direct-v0 \
  --num_envs 8 \
  --max_iterations 1500 \
  --headless
```

No se recomienda saltar directamente a `32` mientras la escena USD no esté clonada por entorno. Aunque sea estable, los robots pueden interferir o quedar fuera de la parte útil de la escena.

---

## 11. Ejecutar el checkpoint entrenado

```bash
cd ~/Github/IsaacLab

RUN_DIR=$(ls -td logs/rsl_rl/leatherback_navigation_direct/* | head -1)
CKPT=$(ls -1 "$RUN_DIR"/model_*.pt | sort -V | tail -1)

echo "RUN_DIR: $RUN_DIR"
echo "CKPT: $CKPT"

./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task Isaac-Leatherback-Navigation-Direct-v0 \
  --num_envs 1 \
  --checkpoint "$CKPT"
```

Durante la prueba visual se debe revisar:

1. si el vehículo avanza hacia la esfera roja;
2. si evita los obstáculos negros;
3. si deja de atravesar cubos;
4. si no oscila hacia adelante y atrás;
5. si se aproxima a la meta sin quedarse detenido.

---

## 12. Script para graficar entrenamiento: `plot_leatherback_training.py`

Archivo:

```text
plot_leatherback_training.py
```

Código completo:

```python
from pathlib import Path
import argparse

import matplotlib.pyplot as plt
from tensorboard.backend.event_processing import event_accumulator


def find_latest_run(base_dir: Path) -> Path:
    runs = [p for p in base_dir.iterdir() if p.is_dir()]
    if not runs:
        raise RuntimeError(f"No encontré runs en: {base_dir}")
    return sorted(runs, key=lambda p: p.stat().st_mtime, reverse=True)[0]


def find_event_file(run_dir: Path) -> Path:
    event_files = list(run_dir.rglob("events.out.tfevents.*"))
    if not event_files:
        raise RuntimeError(f"No encontré archivos TensorBoard en: {run_dir}")
    return sorted(event_files, key=lambda p: p.stat().st_mtime, reverse=True)[0]


def smooth(values, window: int):
    if window <= 1 or len(values) < window:
        return values

    smoothed = []
    for i in range(len(values)):
        start = max(0, i - window + 1)
        chunk = values[start : i + 1]
        smoothed.append(sum(chunk) / len(chunk))
    return smoothed


def pick_tag(tags, candidates):
    normalized = {tag.lower(): tag for tag in tags}

    for candidate in candidates:
        candidate = candidate.lower()
        for tag_lower, original_tag in normalized.items():
            if candidate in tag_lower:
                return original_tag

    return None


def read_scalar(ea, tag):
    events = ea.Scalars(tag)
    steps = [event.step for event in events]
    values = [event.value for event in events]
    return steps, values


def plot_one(ax, ea, tags, title, candidates, smooth_window):
    tag = pick_tag(tags, candidates)

    if tag is None:
        ax.set_title(title + " — no encontrado")
        ax.text(
            0.5,
            0.5,
            "Tag no encontrado",
            ha="center",
            va="center",
            transform=ax.transAxes,
        )
        return

    steps, values = read_scalar(ea, tag)
    values_smoothed = smooth(values, smooth_window)

    ax.plot(steps, values, alpha=0.25, label="raw")
    ax.plot(steps, values_smoothed, linewidth=2.0, label=f"smooth {smooth_window}")
    ax.set_title(f"{title}\n{tag}")
    ax.set_xlabel("Learning iteration")
    ax.grid(True, alpha=0.3)
    ax.legend()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--run_dir",
        type=str,
        default="",
        help="Ruta específica del entrenamiento. Si se omite, usa el último run.",
    )
    parser.add_argument(
        "--base_dir",
        type=str,
        default="logs/rsl_rl/leatherback_navigation_direct",
    )
    parser.add_argument(
        "--smooth",
        type=int,
        default=20,
        help="Ventana de suavizado.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="leatherback_training_curves.png",
    )
    args = parser.parse_args()

    base_dir = Path(args.base_dir)

    if args.run_dir:
        run_dir = Path(args.run_dir)
    else:
        run_dir = find_latest_run(base_dir)

    event_file = find_event_file(run_dir)

    print(f"RUN_DIR: {run_dir}")
    print(f"EVENT_FILE: {event_file}")

    ea = event_accumulator.EventAccumulator(
        str(event_file),
        size_guidance={event_accumulator.SCALARS: 0},
    )
    ea.Reload()

    scalar_tags = ea.Tags().get("scalars", [])

    print("\nSCALAR TAGS ENCONTRADOS:")
    for tag in scalar_tags:
        print(" -", tag)

    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    axes = axes.flatten()

    plot_one(
        axes[0],
        ea,
        scalar_tags,
        "Mean value loss",
        ["value loss", "value_function", "value"],
        args.smooth,
    )

    plot_one(
        axes[1],
        ea,
        scalar_tags,
        "Mean surrogate loss",
        ["surrogate"],
        args.smooth,
    )

    plot_one(
        axes[2],
        ea,
        scalar_tags,
        "Mean entropy loss",
        ["entropy"],
        args.smooth,
    )

    plot_one(
        axes[3],
        ea,
        scalar_tags,
        "Mean reward",
        ["mean reward", "reward"],
        args.smooth,
    )

    plot_one(
        axes[4],
        ea,
        scalar_tags,
        "Mean episode length",
        ["episode length", "episode_length"],
        args.smooth,
    )

    axes[5].axis("off")
    axes[5].text(
        0.0,
        0.9,
        f"Run:\n{run_dir}\n\n"
        f"Archivo TensorBoard:\n{event_file}\n\n"
        "Nota:\n"
        "En PPO, el value loss puede ser inestable.\n"
        "La mejora real se evalúa mejor con reward,\n"
        "episode length y pruebas visuales/evaluación.",
        fontsize=11,
        va="top",
    )

    fig.tight_layout()
    output_path = Path(args.output)
    fig.savefig(output_path, dpi=180)

    print(f"\nGráfica guardada en: {output_path.resolve()}")


if __name__ == "__main__":
    main()
```

---

## 13. Graficar el entrenamiento

Para graficar el último entrenamiento:

```bash
cd ~/Github/IsaacLab

./isaaclab.sh -p plot_leatherback_training.py

xdg-open leatherback_training_curves.png
```

Para un entrenamiento específico:

```bash
cd ~/Github/IsaacLab

ls -td logs/rsl_rl/leatherback_navigation_direct/*
```

Luego:

```bash
./isaaclab.sh -p plot_leatherback_training.py \
  --run_dir logs/rsl_rl/leatherback_navigation_direct/NOMBRE_DEL_RUN

xdg-open leatherback_training_curves.png
```

---

## 14. Interpretación de las gráficas

### `Mean reward`

Debe subir con el entrenamiento. Es la métrica más importante para ver si la política está mejorando.

### `Mean episode length`

Debe interpretarse junto con la prueba visual:

```text
episode length baja  -> puede estar llegando más rápido o chocando más rápido
episode length sube  -> puede estar sobreviviendo más o tardando sin llegar
```

Por eso no debe evaluarse sola.

### `Mean value loss`

En PPO puede tener picos. No debe usarse como única métrica. Si aparece en escalas enormes como `1e34`, el entorno está produciendo recompensas o estados extremos. En esta versión se agregaron límites para evitarlo.

### `Mean entropy loss`

Debe bajar lentamente. Eso indica que la política se vuelve menos aleatoria.

### `Mean surrogate loss`

Normalmente se mantiene cerca de cero. Picos moderados son esperables.

---

## 15. Problemas encontrados y soluciones aplicadas

### 15.1. El robot caía desde muy alto

Se ajustó la altura inicial:

```python
pos=(0.0, 0.0, 0.08)
```

---

### 15.2. La esfera roja no representaba un obstáculo

La esfera roja es solo visual. El RayCaster apunta a:

```python
prim_expr="/World/Warehouse"
prim_expr="/World/TrainingObstacles"
```

La esfera está en:

```text
/World/TargetMarker_0
```

Por eso no se incluye en el LiDAR.

---

### 15.3. El LiDAR producía errores visuales

Se desactivó:

```python
debug_vis=False
```

El LiDAR sigue funcionando para observaciones, aunque no se dibujen los rayos.

---

### 15.4. El modelo aprendía a oscilar hacia adelante y atrás

Se cambió el control de ruedas para permitir solo avance:

```python
throttle_01 = 0.5 * (self.actions[:, 1:2] + 1.0)
wheel_cmd = self.cfg.min_wheel_velocity + throttle_01 * (
    self.cfg.max_wheel_velocity - self.cfg.min_wheel_velocity
)
```

---

### 15.5. El modelo se quedaba quieto

Se agregó penalización por baja velocidad:

```python
low_speed = torch.clamp(0.35 - forward_velocity_reward, min=0.0)
low_speed_penalty = torch.where(
    far_from_goal,
    low_speed,
    torch.zeros_like(low_speed),
)
```

---

### 15.6. El modelo atravesaba obstáculos

Se agregaron dos mecanismos:

1. colisión física del cubo:

```python
collision_props=sim_utils.CollisionPropertiesCfg()
```

2. terminación matemática por distancia:

```python
obstacle_collision = min_test_obstacle_distance < self.cfg.test_obstacle_collision_distance
```

---

### 15.7. El entrenamiento generaba valores extremos

Se agregó clipping de observaciones y recompensas:

```python
reward = torch.clamp(
    reward,
    -self.cfg.max_abs_reward,
    self.cfg.max_abs_reward,
)
```

También se limitan velocidades, distancias y estados fuera de rango.

---

## 16. Recomendación de uso actual

Para la versión actual, entrenar con 4 u 8 entornos:

```bash
--num_envs 4
```

o:

```bash
--num_envs 8
```

No se recomienda usar `32` todavía porque la escena USD no está clonada completamente por entorno. Lo ideal para una versión futura es tener:

```text
/World/envs/env_0/Scene
/World/envs/env_1/Scene
/World/envs/env_2/Scene
...
```

De esa manera, cada robot tendría su propia escena, sus propios obstáculos y su propio LiDAR sin interferencias.

---

## 17. Trabajo futuro

Los siguientes pasos recomendados son:

1. agregar métricas explícitas de éxito, choque y timeout;
2. clonar la escena completa por entorno para poder entrenar con 32 o más entornos;
3. hacer currículo de obstáculos:
   - primero sin obstáculos;
   - luego 1 obstáculo;
   - luego 2 o 3 obstáculos;
4. hacer fine-tuning en escenarios más complejos, como Hospital;
5. exportar la política entrenada;
6. crear un nodo ROS 2 que use la política para controlar el Leatherback.

---

## 18. Resumen del estado actual

El ejemplo actual incluye:

```text
Leatherback Ackermann
Isaac Lab DirectRLEnv
PPO con rsl_rl
LiDAR real por RayCaster
Escena gridroom_curved
Obstáculos negros detectables por LiDAR
Colisión física y castigo matemático
Metas visuales con esfera roja
Rutas específicas start -> goal
Gráficas TensorBoard de entrenamiento
Protección contra NaN/Inf y valores extremos
```

Este estado ya sirve como base documentada para subir al repositorio y continuar con mejoras incrementales.
