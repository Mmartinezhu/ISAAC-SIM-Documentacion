# ROBERT: a six-wheel rocker-bogie rover in Isaac Sim and Isaac Lab

Simulation-only project: a rover designed in Fusion 360, imported into Isaac Sim with its closed-loop suspension,
driven by a reinforcement-learning policy, and navigated with Nav2 over a map built only from a simulated depth camera.
Nothing here has run on the physical robot.

## Guides (in Spanish)

1. [Part 1: simulation](01-simulacion/README.md). From Fusion 360 to a USD controllable over ROS 2, including the closed kinematic loops.
2. [Part 2: reinforcement learning](02-reinforcement-learning/README.md). The Isaac Lab task, PPO configuration, training phases and metrics.
3. [Part 2b: from a training policy to a usable one](02-reinforcement-learning/guia-avanzada.md). Fixed evaluation, bounded actions, nominal control with a learned residual, stopping, real motor limits and export.

## What it does

- **Imports a real mechanism.** The rocker-bogie has a differential and two closed kinematic loops. Isaac needs a tree, so one
  joint per loop is excluded from the articulation (never deleted).
- **Controls it with a learned policy plus explicit modes.** A nominal kinematic controller with a learned correction; the
  actor runs outside the simulator as a NumPy core and as ONNX, and stopping is an explicit controller mode.
- **Navigates with Nav2.** Goal in RViz, Nav2 plans on a map, a supervisor can inhibit motion when the terrain ahead has not been
  observed, and an operator can cancel or re-authorise.
- **Builds its own map.** The planner map comes from the simulated camera's point cloud and the rover pose, not from the
  simulator's geometry. Unobserved cells stay unknown and are never planned through.

## Architecture

```text
 Isaac Lab (host, Python 3.11)                        ROS 2 Jazzy (container, Python 3.12)
 +--------------------------------+   TCP, lockstep   +--------------------------------------------+
 | plant: ROBERT USD + terrain    | <---------------> | bridge -> controller node (policy)         |
 | depth camera (ray-cast mesh)   |                   |        -> supervisor node (cover, inhibit) |
 | joint targets / sensor states  |                   |        -> Nav2 (planner, path follower)    |
 +--------------------------------+                   |        -> map node (cloud -> /map, steps)  |
                                                      +--------------------------------------------+
                                                                        |  RViz: robot, map, plan, goal, state
```

Two processes are not a preference: Jazzy is built for Python 3.12 and Isaac Sim ships 3.11, so `rclpy` cannot be imported inside
Isaac. The plant, controller and supervisor advance in lockstep, one step per simulation step, so their time base is simulated
time. Nav2's planner and path-following loops run on their own timers and are not locked to the simulation step.

## Current configuration ("common traction" variant)

| Item | Value |
| --- | --- |
| Drive | one speed shared by the six wheels, command ceiling 7.16 rad/s = 0.584 m/s |
| Policy | `model_400` of a short training run of this variant |
| Steering | kinematic nominal; the learned steering residual is set to zero when evaluating |
| Simulated wheel torque | 2.452 N·m per wheel sustained, servo gain K = 4.4, torque-speed curve kept. A deliberate assumption of the simulation (the datasheet's instantaneous limit used as sustained), not a configuration of the real robot |
| Torque the policy was trained with | 0.981 N·m, K = 1.0 |
| Camera | one forward camera at the planned ZED 2i mount; depth simulated by ray casting; optical axis about 40 degrees below the horizontal |
| Pose | ideal (from the simulator) |
| Nav2 | no recovery behaviours; the rover cannot spin in place or reverse |

## Results (simulation, with the configuration each belongs to)

| What | Result | Configuration |
| --- | --- | --- |
| Nav2 on flat ground | validated at 0.15, 0.30, 0.45 and 0.55 m/s with 0.10 m goal tolerance; 0.05 m tolerance at 0.15 and 0.55 m/s | planning map from the simulator geometry; torque cap 0.981 N·m, K = 1.0 |
| Moderate rough terrain (level 3, three seeds) | 0.15 / 0.30 / 0.45 m/s: 3/3 routes; 0.55 m/s: 2/3; no rollovers in 12 runs | same |
| Navigation on the perceived map | interactive demonstration at 0.15 m/s | ideal pose, four-obstacle test scenario |
| Perceived height against the scene geometry | 0.1 mm error, a geometric consistency check after correcting a half-pixel offset in the simulated cloud | same scenario and pose; not the accuracy of a real camera |
| Isolated steps, 30 attempts per cell | 6 cm: 30/30 at 0.10, 0.20 and 0.30 m/s; 10 cm crossed once in 30 | torque cap 2.452 N·m, K = 4.4, `model_400` |
| Map step threshold | `escalon_max_m` = 0.10 m, the greatest height crossed completely at least once; it makes a step plannable, not crossable | same |
| Earlier B4 policy on slopes | no rollovers in 480 episodes up to 26 degrees; arrival 100 / 95 / 70 % at 10 / 15 / 20 degrees | previous physics and policy |

The simulation runs slower than real time, so these are not real-time figures. Repeated runs of the same condition vary by
2-3 out of 30 (GPU physics is not deterministic); small differences are not signal.

## Run it (on the lab machine)

The ROBERT code lives on the lab machine and is not in a public repository. Interactive demo with the perceived map:

```bash
python3 lanzar_comun.py --modo demo --mapa percibido --vel 0.15
```

It starts the plant with a window, the bridge, the supervisor, the map node, Nav2 and RViz, and prints on-screen instructions.
Goals are set with "2D Goal Pose" in RViz; `c` + Enter cancels, `q` + Enter quits.

## Limitations

- Simulation only, with an ideal pose and depth generated by ray casting (stereo matching is not simulated). The camera mount,
  the pose estimate and the real motor limits are not tested on hardware.
- The torque assumption above is what makes steps of 6 to 10 cm possible; with the training torque (0.981 N·m) the policy
  crossed about 5 cm.
- The policy's own height grid does not see obstacles built from boxes; Nav2 avoids them using the perceived map.
- Nav2 plans through steps up to the threshold but cannot guarantee the crossing; stuck detection and cancellation stay on.
