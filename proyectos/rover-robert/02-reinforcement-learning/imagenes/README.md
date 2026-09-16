# Imagenes de la Parte 2 (RL)

Capturas pendientes. Guardarlas aqui con el nombre indicado y el `README.md` las mostrara solo. Formato PNG, ancho de 1200 a 1600 px; las de TensorBoard con el smoothing en 0.6 y el nombre del run visible.

| Archivo | Que debe mostrar | Como obtenerla |
| --- | --- | --- |
| `01-terreno-fase1.png` | El terreno de la fase 1 desde arriba: columnas de bajada, subida y obstaculos, filas de dificultad creciente | `play.py --task Isaac-Rover-Robert-v0 --num_envs 8`, camara en perspectiva alta |
| `02-height-scan.png` | La rejilla de puntos del height scan sobre un rover en terreno irregular | Mismo `play.py` con `debug_vis=True` en el `RayCasterCfg`; seleccionar un rover y pulsar `F` |
| `03-play-viewport.png` | 8 rovers moviendose en el nivel 8, con las flechas verde (comando) y azul (velocidad real) | `play.py --task Isaac-Rover-Robert-v0 --num_envs 8 env.scene.terrain.max_init_terrain_level=8` |
| `04-tensorboard-metricas.png` | Panel general de TensorBoard de `clip90_cero`: `Train/mean_reward`, `Episode_Reward/seguir_velocidad`, `Curriculum/terreno`, `Episode_Termination/*` | TensorBoard, filtrar el run `clip90_cero` |
| `05-fase1-curriculum.png` | `Curriculum/terreno`, `nivel_max` y `nivel_min` de `clip90_cero` (terreno ~9, nivel_min en 0) | TensorBoard, seccion Curriculum |
| `06-escalera-terreno.png` | La escalera progresiva completa (las 4 copias) vista desde arriba en angulo | `play.py --task Isaac-Rover-Robert-Escalera-v0 --num_envs 16 --load_run '.*_escalera6'` |
| `07-escalera-rueda-media.png` | Un rover con la rueda delantera sobre el escalon y la media contra la cara vertical | Mismo `play.py` con `--real-time` y `--num_envs 4`; ya existe una captura de esta escena, buscarla en el historial |
| `08-escalera-exito.png` | `Curriculum/exito/escalon_5cm` ... `escalon_14cm` de `escalera6` | TensorBoard, filtrar `escalera6`, seccion Curriculum |
| `09-pendiente-terreno.png` | Las rampas con bandas de color por altura y rovers nacidos sobre ellas mirando cuesta arriba | `play.py --task Isaac-Rover-Robert-Pendiente-v0 --num_envs 8 --load_run '.*_pendiente2'` |
| `10-pendiente-rampa.png` | `Curriculum/rampa/20deg` ... `34deg` y algun `n_Xdeg` de `pendiente2` | TensorBoard, filtrar `pendiente2` |
| `11-denso-terreno.png` | El terreno denso con sus cuatro columnas de tipos y las filas de dificultad | `play.py --task Isaac-Rover-Robert-Denso-v0 --num_envs 16 --load_run '.*_denso1'` |
| `12-denso-tensorboard.png` | `Curriculum/terreno` (llega a 12), `Episode_Reward/seguir_velocidad` y `Episode_Termination/*` de `denso1` | TensorBoard, filtrar `denso1` |

Opcionales, si se quiere ilustrar mas:

- Un rover volcando al nacer en el aire en `pendiente1` (el fallo que motivo `reset_en_rampa`).
- La grafica `Episode_Reward/altura_ganada` de `escalera4` (plana en 0) junto a la de `escalera5` (sube), para el bug del record en el aire.
- `Curriculum/altura/max` de `escalera2` clavada en -0.047: la politica que aprendio a aparcar.
