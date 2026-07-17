# Tutorial 12: Reinforcement Learning para Leatherback en Isaac Lab

Referencia completa del desarrollo:

- [referencia-completa.md](referencia-completa.md)

## Objetivo

El objetivo de este tutorial es documentar una tarea de aprendizaje reforzado en Isaac Lab donde un robot tipo Ackermann/Leatherback aprende a navegar hacia una meta usando PPO, observaciones de LiDAR por ray casting y penalizaciones por choque, baja velocidad o alejamiento de la ruta.

El flujo de trabajo:

- Se registra una tarea custom de Isaac Lab: `Isaac-Leatherback-Navigation-Direct-v0`.
- Se configura PPO con `rsl_rl`.
- Se carga el robot `Leatherback` y una escena de entrenamiento.
- Se agrega un LiDAR 2D por `MultiMeshRayCaster`.
- Se entrenan politicas con 4 u 8 entornos.
- Se valida el checkpoint entrenado y se grafican metricas de TensorBoard.

## Requisitos

Este tutorial asume una instalacion funcional de Isaac Lab en:

```text
~/Github/IsaacLab
```

Tambien asume que los assets locales de Isaac Sim estan disponibles en:

```text
/home/talos/isaacsim_assets/Assets/Isaac/5.0/Isaac
```

Valida Isaac Lab desde la raiz del repositorio:

```bash
cd ~/Github/IsaacLab
./isaaclab.sh -p -c "print('Isaac Lab OK')"
```

Este tutorial no usa ROS2 directamente. Por eso no requiere abrir `./terminal_b.bash`, a menos que despues conectes la politica entrenada con ROS2.

## Parte 1: Crear la estructura de la tarea

Desde la raiz de Isaac Lab:

```bash
cd ~/Github/IsaacLab
mkdir -p source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/agents
touch source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/__init__.py
touch source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/leatherback_navigation_env.py
touch source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/agents/rsl_rl_ppo_cfg.py
```

La estructura esperada es:

```text
source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/
|-- __init__.py
|-- leatherback_navigation_env.py
`-- agents/
    `-- rsl_rl_ppo_cfg.py
```

## Parte 2: Registrar la tarea

Archivo:

```text
source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/__init__.py
```

Contenido:

```python
import gymnasium as gym

from . import agents

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

Ese `id` es el que se usa despues con `--task`.

## Parte 3: Configurar PPO

Archivo:

```text
source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/agents/rsl_rl_ppo_cfg.py
```

Configuracion base usada:

| Campo | Valor |
| --- | --- |
| `num_steps_per_env` | `24` |
| `max_iterations` | `1500` |
| `save_interval` | `20` |
| `experiment_name` | `leatherback_navigation_direct` |
| `actor_hidden_dims` | `[64, 64]` |
| `critic_hidden_dims` | `[64, 64]` |
| `learning_rate` | `3.0e-4` |
| `gamma` | `0.99` |
| `lam` | `0.95` |

La politica tiene dos acciones:

| Accion | Uso |
| --- | --- |
| `0` | Direccion / steering |
| `1` | Aceleracion hacia adelante / throttle |

El codigo completo de esta configuracion esta en la seccion `6. Configuracion PPO` de [referencia-completa.md](referencia-completa.md).

## Parte 4: Crear el entorno `DirectRLEnv`

Archivo:

```text
source/isaaclab_tasks/isaaclab_tasks/direct/leatherback_navigation/leatherback_navigation_env.py
```

El entorno usa:

| Componente | Configuracion |
| --- | --- |
| Robot | `Leatherback` desde assets locales |
| Escena | `gridroom_curved.usd` |
| Sensor | `MultiMeshRayCaster` tipo LiDAR 2D |
| Observaciones | LiDAR, pose relativa a la meta, velocidad y acciones previas |
| Recompensas | Progreso hacia la meta, velocidad hacia adelante y llegada |
| Penalizaciones | Choques, proximidad a obstaculos, steering excesivo, baja velocidad y tiempo |
| Terminaciones | Meta alcanzada, choque, salida de limites, estado invalido o timeout |

Rutas usadas por el entorno:

```python
TRAINING_SCENE_USD_PATH = "/home/talos/isaacsim_assets/Assets/Isaac/5.0/Isaac/Environments/Grid/gridroom_curved.usd"
LEATHERBACK_USD_PATH = "/home/talos/isaacsim_assets/Assets/Isaac/5.0/Isaac/Robots/NVIDIA/Leatherback/leatherback.usd"
```

Para crear el archivo completo, copia la seccion `7. Entorno completo` de [referencia-completa.md](referencia-completa.md) en `leatherback_navigation_env.py`.

## Parte 5: Validar que la tarea quedo registrada

Desde la raiz de Isaac Lab:

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

La salida debe contener:

```text
TAREA REGISTRADA CORRECTAMENTE:
EnvSpec(id='Isaac-Leatherback-Navigation-Direct-v0', ...)
```

## Parte 6: Probar estabilidad antes de entrenar

Antes de entrenar, corre una prueba corta con acciones aleatorias. El objetivo es detectar `NaN`, `Inf` o recompensas fuera de rango.

La prueba completa esta en la seccion `9. Probar estabilidad del entorno con 4 entornos` de [referencia-completa.md](referencia-completa.md).

Resultado esperado:

```text
RESET OK
OBS SHAPE: torch.Size([4, 80])
PRUEBA OK: 3000 pasos sin NaN/Inf y reward acotado.
```

No avances al entrenamiento si esta prueba falla.

## Parte 7: Entrenar la politica PPO

Entrenamiento base recomendado:

```bash
cd ~/Github/IsaacLab

./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Leatherback-Navigation-Direct-v0 \
  --num_envs 4 \
  --max_iterations 1500 \
  --headless
```

Si el entorno se mantiene estable, prueba con 8 entornos:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Leatherback-Navigation-Direct-v0 \
  --num_envs 8 \
  --max_iterations 1500 \
  --headless
```

No se recomienda saltar directamente a `32` entornos mientras la escena USD no este clonada completamente por entorno.

## Parte 8: Ejecutar el checkpoint entrenado

Usa el ultimo checkpoint disponible:

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

Durante la prueba visual revisa:

- Si el vehiculo avanza hacia la esfera roja.
- Si evita obstaculos negros.
- Si no atraviesa cubos.
- Si no oscila hacia adelante y atras.
- Si se aproxima a la meta sin quedarse detenido.

## Parte 9: Graficar entrenamiento

Crea el script:

```text
plot_leatherback_training.py
```

El codigo completo esta en la seccion `12. Script para graficar entrenamiento` de [referencia-completa.md](referencia-completa.md).

Para graficar el ultimo entrenamiento:

```bash
cd ~/Github/IsaacLab
./isaaclab.sh -p plot_leatherback_training.py
xdg-open leatherback_training_curves.png
```

Para un entrenamiento especifico:

```bash
ls -td logs/rsl_rl/leatherback_navigation_direct/*

./isaaclab.sh -p plot_leatherback_training.py \
  --run_dir logs/rsl_rl/leatherback_navigation_direct/NOMBRE_DEL_RUN
```

## Parte 10: Interpretar las metricas

| Metrica | Interpretacion |
| --- | --- |
| `Mean reward` | Debe subir durante el entrenamiento. Es la metrica principal. |
| `Mean episode length` | Debe interpretarse junto con pruebas visuales; puede subir por supervivencia o bajar por llegada rapida. |
| `Mean value loss` | Puede tener picos en PPO. No debe usarse como unica metrica. |
| `Mean entropy loss` | Debe bajar lentamente cuando la politica se vuelve menos aleatoria. |
| `Mean surrogate loss` | Normalmente se mantiene cerca de cero. |

## Problemas resueltos

- El robot caia desde muy alto: se ajusto `pos=(0.0, 0.0, 0.08)`.
- La esfera roja no era obstaculo: se dejo solo como marcador visual de meta.
- El LiDAR generaba ruido visual: se desactivo `debug_vis`.
- El modelo oscilaba hacia adelante y atras: el throttle se limito a avance.
- El modelo se quedaba quieto: se agrego penalizacion por baja velocidad.
- El modelo atravesaba obstaculos: se agregaron colisiones fisicas y terminacion matematica por distancia.
- El entrenamiento generaba valores extremos: se hizo clipping de observaciones y recompensas.

## Trabajo futuro

- Agregar metricas explicitas de exito, choque y timeout.
- Clonar la escena completa por entorno para entrenar con 32 o mas entornos.
- Crear curriculo de obstaculos.
- Hacer fine-tuning en escenarios mas complejos, como Hospital.
- Exportar la politica entrenada.
- Crear un nodo ROS2 que use la politica para controlar Leatherback.
