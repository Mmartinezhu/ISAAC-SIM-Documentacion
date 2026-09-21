# Siguiente fase: mediciones fiables y politica transferible

Entrega de la fase de evaluacion y reentrenamiento realizada entre el 19 y el 20 de septiembre de 2026, siguiendo el plan de ocho puntos recibido de la revision del proyecto. Codigo completo en [referencia-evaluacion.md](referencia-evaluacion.md).

Regla que se siguio en toda la fase: no cambiar PPO, recompensas, observaciones y fisica a la vez; conservar los checkpoints; cada experimento en una configuracion nueva.

## Resumen ejecutivo

**Aviso (2026-09-21):** la primera entrega evaluo `robot_v1` con ruido de exploracion en el actor y con cuotas desiguales. La seccion **Revision 2** contiene los resultados corregidos y sustituye a las secciones 4, 5, 7 y 8 para cualquier decision. Cambian el checkpoint propuesto (`model_1000` en vez de `model_9999`), la afirmacion de que los recortes no cuestan rendimiento (falsa en rampa) y la lectura de los vuelcos y de la direccion.

- **Cuatro errores de medida** que invalidaban cifras anteriores, corregidos antes de entrenar nada: el recorte de direccion nunca estuvo activo; las tasas de exito por escalon y por rampa se calculaban sobre el ultimo lote de resets (1-3 rovers volcados); el spawn aleatorio en la mitad alta de las rampas inflaba las tasas; y `applied_torque` del actuador implicito no es el par que aplica el solver.
- **Las politicas anteriores controlaban par a bang-bang**: pedian 30-66 rad/s a ruedas limitadas a 2.045 y saturaban el servo de direccion contra el tope de ±90° el 63-93 % del tiempo. **Acotar las acciones no cuesta rendimiento** (medido con la misma politica, 4 variantes, 3 niveles).
- **Politica nueva `robot_v1`**: actor con solo lo que el robot mide (414 entradas, sin suspension), critico privilegiado (442), tracción y direccion acotadas, entrenada desde cero 10 000 iteraciones. **Iguala a `denso1` en terreno denso y llano** con saturacion de traccion del 95 % al 4 % y la mitad de deriva en parada.
- **Checkpoint propuesto: `robot_v1/model_9999.pt`** (en `logs/rsl_rl/rover_robert_v1/2026-09-19_*_robot_v1/`). `model_8000` es indistinguible en distribucion y vuelca 18 veces mas fuera de ella.
- **Limites reales, medidos con politica determinista, semilla 42, 100 episodios por condicion**: escalon aislado de 5 cm al 28-40 % (7 cm al 1-13 %, 9 cm 0-1 %); rampas de 20° o mas 0 % para toda politica no entrenada en rampas (la entrenada en rampas, `pendiente2`, 90 % a 20°, 59 % a 22°, 27 % a 24°, 0 % a 26°); terreno denso de nivel 6 (obstaculos ±7 cm, huecos 10 cm) 75-81 % en bloques y 25-28 % en losas con huecos.
- **Fallos abiertos**: con comando cero el rover deriva 0.5 m en 60 s; la politica nueva pone las ruedas de las esquinas a ±90° la mayor parte del tiempo, y sobre rampas eso la deja clavada (y `model_8000` vuelca).

## Revision 2: evaluador corregido y resultados definitivos (2026-09-21)

Esta seccion sustituye, para cualquier decision, a las secciones 4, 5, 7 y 8 de mas abajo, que se conservan como registro de la primera entrega. Todo lo que sigue se midio con el evaluador v2 ([referencia-evaluacion.md](referencia-evaluacion.md), seccion "Evaluador v2"), con resultados en `~/robert_eval_v2/` (etiquetas `e2_*` y `v_*`); los de la primera entrega siguen en `~/robert_eval/`.

### R2.1 Que estaba mal en la primera entrega

| Defecto | Efecto | Correccion |
| --- | --- | --- |
| Las clases de evaluacion de `robot_v1` llamaban a `aplicar_robot()` despues de desactivar el ruido, y `aplicar_robot` instala un grupo de observaciones nuevo con `enable_corruption=True` | **Toda la evaluacion de `robot_v1` de la primera entrega se hizo con ruido de exploracion en el actor.** Los 128 vuelcos en rampa de `model_8000` y el "96 % con las ruedas al tope" de `model_9999` no se reproducen sin ruido | `enable_corruption = False` despues de `aplicar_robot` en las cinco clases; el evaluador aborta si el actor tiene corrupcion activa y registra el estado en `resumen.json` |
| Los entornos que terminaban pronto seguian aportando episodios mientras se esperaba a los demas | Denominadores desiguales (108-134 intentos en 30-34°) y sesgo hacia los rovers que vuelcan | Cuota exacta por entorno en el script y en los gestores de metricas (`fijar_cuota`); verificacion de intentos por grupo; informe de cuotas incompletas |
| El porcentaje de acciones crudas fuera de ±1 se presentaba como "tiempo con las ruedas al tope" | Sobreestimacion: son cosas distintas | Cuatro medidas separadas por reductor: accion cruda fuera de ±1, objetivo recortado a 90°, angulo real ≥ 85°, y error objetivo-angulo, velocidad y par |
| "Parada" solo media distancia recorrida | No distingue derivar de girar sobre si mismo | Trayectoria, desplazamiento final, desviacion maxima y velocidad residual, desde el reset y tras 2 s de asentamiento |
| "Exito" en rampas y escalones mezclaba llegada y estabilidad | El criterio solo comprueba posicion 1 s | `llegadas_pos_1s` (solo posicion), `completos_sin_fallo` (llegada y time_out sin vuelco) y `vuelco_tras_llegada`, por separado |
| Las lecturas de actuadores tras `env.step()` incluian el paso terminal de cada episodio, en el que el entorno ya esta reseteado | Una muestra por episodio mezclaba accion a cero, comando nuevo y estado nuevo | Esas muestras se excluyen de las estadisticas por paso (`muestras_excluidas_paso_terminal` en el JSON); las series guardan la mascara `valido` |
| El modulo del wrench de reaccion se llamaba "par medido" | No es el par del motor | Renombrado `wrench_par_modulo_*_NO_es_par_motor`; el par del actuador implicito se llama `par_estimado` |
| Las rampas eran constantes de modulo (20-34°) | No se podia medir por debajo de 20° | Geometria leida de la configuracion del terreno; tarea `Pendiente-v2-Eval` con 10, 15, 20, 22, 24 y 26° |
| `pendiente2` se evaluaba sin recortes y `robot_v1` con ellos, sin decirlo | Comparacion no controlada | Cada JSON registra los recortes efectivos; `pendiente2` evaluada en A (sin) y D (con) |

Verificacion de la cuota: 48 entornos, 2 episodios, rampas de 10-26°: 16/16 intentos exactos por rampa, 0 entornos incompletos. En todas las ejecuciones de esta revision `cuota_ok=True`, `incompletos=0`, `actor_enable_corruption=False`, `normalizacion_sin_cambios=True` y `actor_modo_eval=True`.

### R2.2 Condiciones

Politica determinista, semilla 42 (y 43, 44 donde se indica), sin ruido en observaciones, normalizacion cargada del checkpoint y verificada sin cambios, curriculum congelado, posiciones iniciales fijas por indice de entorno, misma fisica (friccion del terreno 0.8 en denso y llano, 1.0 en rampas y escalones). Recortes efectivos: `robot_v1` siempre con traccion ±2.045 rad/s y direccion ±90° (tareas `*-v1-Eval`); `denso1` con ambos recortes (variante D); `pendiente2` en A (sin recortes, como se entreno) y D. Cuotas: llano 100 episodios de 60 s por guion (500 entornos × 1); denso 20 rovers por columna × 2 episodios de 60 s (bloques 160, losas 120, rejilla 120, rugoso 80); rampas y escalones 100 episodios de 40 s por condicion.

### R2.3 Tablas

**Llano (100 por guion).** Exito = `time_out` sin vuelco y < 10 % del tiempo atascado. Entre parentesis, desplazamiento final en 60 s.

| Guion | `robot_v1` 1000 | `robot_v1` 8000 | `robot_v1` 9999 | `denso1` D |
| --- | --- | --- | --- | --- |
| 0.05 m/s | 100/100 (3.08 m) | 95/100 (3.19) | 100/100 (3.27) | 100/100 (3.39) |
| 0.10 m/s | 100/100 (6.57) | 100/100 (6.68) | 100/100 (6.44) | 100/100 (7.17) |
| 0.15 m/s | 100/100 (8.83 = 0.147 m/s) | 100/100 (7.65 = 0.127) | 100/100 (7.50 = 0.125) | 100/100 (7.80) |
| Giro 90° | 100/100 | 98/100 | 99/100 | 100/100 |
| Parada: desplazamiento final / trayectoria / velocidad residual tras 2 s | 0.17 m / 0.43 m / 0.01 m/s | 0.30 / 0.50 / 0.01 | 0.29 / 0.49 / 0.01 | **2.57 m / 2.76 m / 0.05 m/s** |

Semillas 43 y 44 repiten estos valores con diferencias de ±0.1 m y ±5 episodios.

**Denso (bloques / losas / rejilla / rugoso, exitos / intentos = x/160, x/120, x/120, x/80).**

| Nivel | `robot_v1` 1000 | `robot_v1` 8000 | `robot_v1` 9999 | `denso1` D |
| --- | --- | --- | --- | --- |
| 0 | 160 / 117 / 114 / 78 | 160 / 117 / 109 / 67 | 159 / 117 / 105 / 68 | 160 / 115 / 113 / 76 |
| 6 (semilla 42) | **138 / 38 / 24 / 49** | 122 / 32 / 12 / 43 | 114 / 33 / 9 / 44 | 118 / 30 / 17 / 42 |
| 6 (semilla 43) | 137 / 38 / 28 / 44 | 119 / 29 / 18 / 39 | 112 / 31 / 16 / 39 | — |
| 6 (semilla 44) | 137 / 34 / 23 / 49 | 117 / 27 / 16 / 39 | 118 / 24 / 13 / 42 | — |
| 12 | 19 / 5 / 6 / 37 | 24 / 2 / 2 / 34 | 23 / 7 / 1 / 32 | 19 / 12 / 5 / 34 |

En nivel 6, `model_1000` gana en las cuatro columnas en las tres semillas: bloques 86 % frente a 73-76 % (8000) y 70-74 % (9999); rejilla 19-23 % frente a 10-15 % y 7-13 %. En nivel 12 los cuatro candidatos estan dentro del ruido.

**Rampas (llegadas / 100; `completos_sin_fallo` coincidio con las llegadas y no hubo vuelcos ni vuelcos tras llegada en ninguna ejecucion).**

| Rampa | `pendiente2` A (sin recortes) | `pendiente2` D (con recortes) | `robot_v1` 1000 | `robot_v1` 8000 | `robot_v1` 9999 |
| --- | --- | --- | --- | --- | --- |
| 10° | 97 | 97 | 99 | 85 | 66 |
| 15° | 97 | 98 | 12 | 51 | 1 |
| 20° | 84 | **0** | 0 | 0 | 0 |
| 22° | 47 | 0 | 0 | 0 | 0 |
| 24° | 17 | 0 | 0 | 0 | 0 |
| 26° | 0 | 0 | 0 | 0 | 0 |

**Escalon aislado (llegadas / 100).**

| Escalon | `robot_v1` 1000 | `robot_v1` 8000 | `robot_v1` 9999 | `denso1` D |
| --- | --- | --- | --- | --- |
| 5 cm | **63** | 36 | 27 | 42 |
| 7 cm | 1 | 0 | 1 | 1 |
| 9 cm | 0 | 0 | 0 | 0 |

### R2.4 Actuadores, por reductor

Fraccion del tiempo (muestras validas) con el angulo real ≥ 85°, en el orden `reductor_der_d`, `reductor_izq_d`, `reductor_der_t_`, `reductor_izq_t` (delantero derecho, delantero izquierdo, trasero derecho, trasero izquierdo). Error = |objetivo − angulo| medio.

| Ejecucion | angulo ≥ 85° | objetivo recortado a 90° | error medio (rad) | par estimado medio (N·m, limite 20) |
| --- | --- | --- | --- | --- |
| `robot_v1` 1000, llano | 0.20 / 0.01 / 0.39 / **0.79** | 0.38 / 0.20 / 0.56 / 0.89 | 0.75 / 0.77 / 0.58 / 0.22 | 13.4 / 13.9 / 10.2 / 4.2 |
| `robot_v1` 9999, llano | 0.20 / 0.21 / 0.64 / 0.58 | 0.30 / 0.26 / 0.77 / 0.72 | 0.51 / 0.57 / 0.23 / 0.43 | 11.3 / 11.9 / 4.8 / 7.8 |
| `denso1` D, llano | 0.05 / 0.05 / **0.75** / 0.22 | 0.21 / 0.24 / 0.81 / 0.50 | 0.73 / 0.74 / 0.21 / 0.60 | 13.5 / 13.2 / 4.3 / 10.6 |
| `robot_v1` 1000, rampas | 0.16 (media) | 0.22 | 0.27 | — |
| `robot_v1` 9999, rampas | 0.81 / 0.80 / 0.83 / 0.80 | 0.84 / 0.85 / 0.87 / 0.85 | 0.12 / 0.14 / 0.10 / 0.11 | 3.5 / 4.1 / 3.3 / 3.8 |

Traccion: en todos los candidatos con recorte, la accion cruda esta fuera de ±1 el 86-99 % del tiempo y el objetivo escalado esta en el tope de 2.045 rad/s ese mismo porcentaje; el par estimado se recorta solo el 2-6 % del tiempo (antes 95-99 %).

### R2.5 Videos y series sincronizadas

Once grabaciones en `~/robert_eval_v2/v_*/video/*.mp4`, cada una con su panel `series_env<i>.png` del mismo rover y la misma ejecucion (fotograma 0 = paso 0): comando y velocidad real, objetivo y angulo de cada reductor, objetivo y velocidad de cada rueda, acciones crudas de traccion y marca de terminacion con su motivo. `robot_v1` 1000 y 9999 en recta a 0.10 m/s, giro de 90° y parada; 1000 y 9999 en la columna de rejilla de nivel 6; 1000, 8000 y 9999 en la rampa de 15°.

Lo que muestran las series (observado):

- **Rampa de 15°, `model_9999`**: a los 2-3 s los cuatro reductores van a ±90° (dos a +90, dos a −90) y no vuelven; las ruedas giran a ±2 rad/s alternando sentido; velocidad de avance cero durante los 40 s. Asi es "atascarse" para esta politica: una postura de pivote con las ruedas de lado, patinando.
- **Recta a 0.10 m/s, `model_1000`**: la velocidad sigue el comando (error 0.02 m/s), pero los objetivos de direccion oscilan a 50 Hz entre ±90° y los angulos reales barren ±75°; el trasero izquierdo esta bloqueado de lado el 79 % del tiempo; la rueda media derecha gira en sentido contrario al avance a −2 rad/s.
- **Parada, `model_1000`**: tres reductores clavados a ±90°, el cuarto oscilando ±75°, y las seis ruedas girando a ±2 rad/s en sentidos opuestos durante 60 s. El desplazamiento de 0.17 m se consigue porque las ruedas se anulan entre si, no porque el rover frene. Las medidas de parada no lo detectan; hace falta la velocidad media de rueda por guion (pendiente).
- `denso1` con recortes presenta el mismo patron de rueda trasera atravesada (derecha, 75 % del tiempo).

Confirmacion visual en los `.mp4`: *pendiente de anotar por el autor*.

### R2.6 Hallazgos y hipotesis

Observado:

1. **Los recortes no son gratis en rampa.** `pendiente2` cae de 84 % a 0 % a 20° al acotar tracción y direccion; en denso y llano no cambian nada. En rampa la politica usa el objetivo de velocidad como palanca de par, y con el tope el par disponible en movimiento (2.16 × (2.045 − ω)) no basta.
2. **La actuacion es degenerada en todos los candidatos**: direccion oscilando o bloqueada a 90°, ruedas contrarrotando. Con los recortes, el bang-bang de velocidad se ha convertido en bang-bang de direccion y de sentido de giro. Los objetivos ya no superan los limites, pero el uso de los actuadores no es transferible a servos reales.
3. **`model_1000` es el mejor checkpoint de `robot_v1` en llano, denso (tres semillas) y escalon**; en rampa de 15° lo es `model_8000`. Las 9000 iteraciones posteriores a la 1000 no mejoraron nada en evaluacion determinista.
4. **Quitar la suspension del actor no cuesta rendimiento medible**: `robot_v1` iguala o supera a `denso1` con recortes en todo salvo losas de nivel 12 (5 frente a 12 de 120, dentro del ruido). Sigue sin ser una comparacion limpia (historias de entrenamiento distintas).
5. **Los 128 vuelcos y el "96 % al tope" de la primera entrega eran ruido de evaluacion.** Sin ruido no hay vuelcos en ningun candidato ni escenario; el bloqueo a 90° de `model_9999` en rampa es real (81 %) pero no provoca vuelcos.
6. `denso1` con recortes deriva 2.57 m con comando cero; `robot_v1` 0.17-0.30 m (con ruedas girando).
7. Ninguna politica sin entrenamiento en rampas sube 20°. `robot_v1` sube 10° (99 % el 1000) y 15° a medias (51 % el 8000).

Hipotesis (no comprobadas):

- La rueda trasera atravesada actua como estabilizador de guiñada o freno de deriva; explicaria por que ninguna politica pasa de 0.13 m/s cuando se le piden 0.15 (el 1000, con menos bloqueo, llega a 0.147).
- La oscilacion de direccion a 50 Hz es gratis para la politica: `penaliza_cambio_accion` (−0.005) y `penaliza_par` (−2.5e−6) no la castigan lo bastante frente al tracking, y con el recorte una desviacion grande de la gaussiana no cuesta nada (por eso `Policy/mean_std` subio a 1.66).
- El curriculum promociona por distancia recorrida con rumbos aleatorios, que se puede maximizar con esta actuacion; la evaluacion determinista con posiciones fijas no la premia. Eso explicaria que la evaluacion fija sea plana desde la iteracion 1000 mientras el curriculum subia.

### R2.7 Recomendacion provisional y punto de parada

Checkpoint provisional: **`robot_v1/model_1000.pt`** (`logs/rsl_rl/rover_robert_v1/2026-09-19_*_robot_v1/`), por ser el mejor en llano, denso y escalon con tres semillas, con la reserva de que su actuacion (direccion oscilante, ruedas contrarrotando en parada) no es aceptable para el robot real. `model_8000` solo lo supera en la rampa de 15°.

Con estos resultados, el unico experimento corto que tiene sentido es sobre la **direccion**: la parada y las rampas son sintomas del mismo uso degenerado de los actuadores. Que forma toma (penalizar velocidad o angulo de los reductores, reducir la escala de la accion de direccion, filtrar los objetivos) es la decision pendiente; no se ha cambiado nada todavia.

Pendiente de esta revision: confirmacion visual de los videos por el autor; velocidad media de rueda por guion en llano; evaluacion con friccion 0.6 fijando suelo y ruedas; politica de control con observaciones completas (10 h, no ejecutada por indicacion del plan).

---

## 1. Referencia reproducible

| | |
| --- | --- |
| Isaac Sim | 5.1.0-rc.19 (release 26219, `9c81211b`) |
| Isaac Lab | 2.3.2, commit `c22775241e28f465fe345fa1a482ad6d29d712b0` (2 jun 2026), `main` |
| Paquetes | isaaclab 0.54.4, isaaclab_rl 0.5.2, isaaclab_tasks 0.11.16, rsl-rl-lib 5.0.1, torch 2.7.0+cu128, warp 1.14.0, trimesh 4.5.1, numpy 1.26.0 |
| Maquina | Ubuntu 24.04.4, RTX 5080 16 GB, driver 570.211.01, Ryzen 7 9700X |
| Semilla | 42 en todos los runs (`agent.yaml`); el entorno hereda la del runner en `train.py` |
| Checkpoints conservados | `~/robert_referencia/`: `2026-09-13_12-13-07_clip90_cero` (model_8000), `2026-09-15_10-58-49_pendiente2` (model_11000), `2026-09-15_15-43-31_denso1` (model_10998), cada uno con `params/env.yaml` y `params/agent.yaml` |
| Codigo de las tareas | `~/robert_referencia/codigo/` (no estaba en ningun repositorio) |

Volcado de la configuracion efectiva (`scripts/referencia_reproducible.py`), identico en `v0`, `Denso-v0` y `Pendiente-v0`:

```text
runner clip_actions: None          traccion._clip: None          direccion._clip: None
obs[policy] 442 = base_lin_vel 0-2 | base_ang_vel 3-5 | projected_gravity 6-8 | velocity_commands 9-11
                  | joint_pos 12-32 (21) | joint_vel 33-53 (21) | actions 54-63 | height_scan 64-441 (378)
acciones 10 = traccion 0-5 [llanta_der_m_, llanta_der_d_, llanta_izq_m, llanta_izq_d, llanta_der_t, llanta_izq_d_] escala 2.045
              direccion 6-9 [reductor_der_d, reductor_izq_d, reductor_der_t_, reductor_izq_t] escala pi/2, offset 0
joints 0-20 = hombro_der, hombro_izq, suspension, codo_der, reductor_der_d, codo_izq, reductor_izq_d, union_sus_der:0/1/2,
              union_sus_izq:0/1/2, llanta_der_m_, reductor_der_t_, llanta_der_d_, llanta_izq_m, reductor_izq_t, llanta_izq_d,
              llanta_der_t, llanta_izq_d_
actuadores: traccion effort 4.41 / vel 2.045 / K 0 / D 2.16;  direccion effort 20 / vel 3 / K 100 / D 10;  suspension effort 0 / D 2
limites USD del reductor: ±1.5708 rad
```

**Hallazgo.** `clip: null` en las tres configuraciones efectivas. El recorte de direccion a ±90° acordado el 12 de septiembre se pego en una copia de `rover_env_cfg.py` fuera del paquete y nunca entro en vigor; `clip90_cero` se llama asi por error. Como el joint del USD si tiene tope en ±90°, el rover nunca giro mas: lo que hacia la politica era empujar contra el tope. Ninguna politica anterior tenia recorte de traccion.

Lecciones: leer la configuracion efectiva que guarda cada run (`params/env.yaml`), no la que uno cree haber escrito; y `--load_run` usa `re.match` anclado al principio, siempre entre comillas.

## 2. Metricas corregidas antes de entrenar

Tarea nueva `Isaac-Rover-Robert-Pendiente-v1` (y `-v1-Eval`), sin tocar la v0:

| Pieza | Que hace |
| --- | --- |
| `ResetEnRampaV1` | Guarda la rampa inicial de cada episodio (`rampa_inicial`, -1 = plataforma). Modo de evaluacion con posiciones fijas por indice de entorno: rampa `i % 8`, lado `(i // 8) % 4`, 5 desplazamientos a lo largo del lado, siempre 25 cm dentro de la rampa, morro cuesta arriba, 10 cm sobre la superficie |
| `AlturaGanadaV1` | Misma recompensa que v0; `z_spawn` se fija en cada reset y se marca `asentado` al acabar el primer segundo, nunca hereda del episodio anterior |
| `RastreoEpisodio` | Terminacion que nunca termina: gancho por paso. Atasco = comando de avance > 0.03 m/s y progreso radial con signo (distancia Chebyshev al centro; resbalar cuesta abajo es negativo) < 0.10 m en 5 s. Linea de llegada = borde exterior de la rampa inicial mantenido 1 s |
| `MetricasRampa` | Contadores enteros por rampa: intentos, exitos (linea), exitos por altura (criterio v0, -3 cm), tempranos (termino antes de asentarse; cuenta como intento fallido), atascados, tiempo atascado. Media exponencial solo como indicador de entrenamiento |

Primer resultado con estas metricas (con ruido de exploracion, `pendiente2/model_11000`): 20° 58 %, 22° 26 %, 24° 4 %, 26° 0 %. Las cifras de la fase anterior (22° al 60-92 %) estaban infladas porque los rovers nacian en cualquier punto de la rampa, muchos en la mitad alta. Y el progreso en modulo contaba como avance el resbalar hacia atras en 32-34°; de ahi el progreso con signo.

Descubierto tambien: los episodios no estan sincronizados aunque el `time_out` sea igual para todos (los resets gotean), asi que cualquier metrica "por lote" debe acumular por llamada.

## 3. Instrumentacion de actuadores

`scripts/evaluar_rover.py` registra por paso acciones crudas, objetivos escalados y recortados, velocidades objetivo y reales de las seis ruedas, angulos objetivo y reales de los cuatro reductores, par estimado por el actuador implicito (`computed_torque` antes del recorte, `applied_torque` despues) y par "medido" (modulo del par del wrench de reaccion del solver, `body_incoming_joint_wrench_b`: incluye reacciones fuera del eje, es una cota superior). Series temporales en `series.npz`, agregados en `resumen.json`.

`pendiente2/model_11000` en `Pendiente-v1-Eval`, 512 episodios, politica determinista:

| Medida | Traccion | Direccion |
| --- | --- | --- |
| Acciones crudas fuera de [-1, 1] | 99 % | 84 % |
| Velocidad objetivo media | 11.2 rad/s (limite 2.045; real 1.68) | — |
| Tiempo con par recortado (`computed` > limite) | 97 % | 93 % |
| Par estimado medio (`applied`) | 4.35 N·m | 18.9 N·m |
| Par medido medio (cota) | 1.63 N·m | 4.1 N·m |
| Fraccion de `penaliza_par` | 7 % | **93 %** |

En terreno denso la traccion llegaba a 30-47 rad/s de objetivo y a 66 rad/s en llano.

Conclusiones:
- La politica no controlaba velocidad: usaba el objetivo de velocidad como palanca de par maximo (bang-bang).
- `penaliza_par` estaba dominada por direccion (93 %), como se sospechaba, porque el servo empujaba contra el tope del joint. No se subio su peso; se acotaron las acciones.
- El par estimado del actuador implicito no es el aplicado: con `velocity_limit_sim` activo, PhysX frena la rueda en 2.045 rad/s y el par real necesario cae, pero `applied_torque` sigue diciendo 4.41 porque solo mira objetivo menos real. Las graficas de `penaliza_par` de todos los runs anteriores median el modelo, no la fisica.

## 4. Evaluacion fija

*Primera entrega (2026-09-20). Las cifras de `robot_v1` de esta seccion se midieron con ruido en el actor y sin cuota exacta; ver Revision 2 para las corregidas.*

Protocolo: politica determinista (media de la gaussiana), semilla 42, curriculum congelado, normalizacion de observaciones cargada del checkpoint, sin ruido en observaciones, mismas posiciones iniciales. Escenarios y criterios:

| Escenario | Tarea | Condiciones | Criterio de exito |
| --- | --- | --- | --- |
| Llano | `Llano-Eval` | 100 episodios de 60 s por guion: 0.05, 0.10, 0.15 m/s recto; giro de +90° a 0.10; parada | `time_out` sin vuelco y < 10 % del tiempo atascado |
| Denso | `Denso-Eval-A` / `Denso-v1-Eval` | Niveles fijos 0, 6 y 12; 20 rovers por columna × 2 episodios de 60 s = 160/120/120/80 por tipo | Idem |
| Pendientes | `Pendiente-v1-Eval` | 20 a 34° de 2 en 2, 100 episodios de 40 s por rampa | Cruzar el borde exterior de la rampa y mantenerse 1 s |
| Escalon aislado | `Escalon-Eval` | 5, 7 y 9 cm, 100 episodios de 40 s, desde la plataforma mirando hacia fuera ±20° | Cruzar una linea 1 m tras el escalon y mantenerse 1 s |

El criterio "< 10 % atascado" es exigente en terreno denso; lo que importa es comparar con el mismo criterio. Dimensiones reales: los terrenos de height-field usan `horizontal_scale=0.1`, asi que los huecos de las losas configurados a 10.4 cm (nivel 6) y 15.7 cm (nivel 12) se cuantizan a 10 o 20 cm.

### 4.1 Tabla por checkpoint y escenario

Exitos / intentos en porcentaje. `robot_v1` es la politica nueva (seccion 7); las otras dos son las candidatas anteriores.

| Escenario | `clip90_cero` | `denso1` | `robot_v1` 8000 | `robot_v1` 9999 | `pendiente2` 11000 |
| --- | --- | --- | --- | --- | --- |
| Llano 0.05 m/s | 83 (3.18 m) | 100 (3.07 m) | 96 (3.54 m) | 99 (3.41 m) | — |
| Llano 0.10 m/s | 100 (5.29 m) | 100 (6.32 m) | 100 (6.80 m) | 100 (6.64 m) | — |
| Llano 0.15 m/s | 100 (7.67 m) | 100 (7.73 m) | 100 (7.85 m) | 100 (7.66 m) | — |
| Giro 90° | 100 | 99 | 99 | 100 | — |
| Parada: deriva en 60 s | 0.41 m | 1.17 m | 0.50 m | 0.49 m | — |
| Denso n0 obst / piedras / rejilla / rugoso | 96 / 84 / 84 / 80 | 99 / 94 / 91 / 89 | 100 / 93 / 89 / 80 | 100 / 93 / 85 / 80 | — |
| Denso n6 | 63 / 25 / 18 / 48 | 67 / 27 / 12 / 52 | **81** / 28 / 12 / 49 | 76 / 28 / 9 / 49 | — |
| Denso n12 | 15 / 3 / 3 / 44 | 16 / 2 / 3 / 41 | 13 / 4 / 4 / 39 | 10 / 2 / 2 / 35 | — |
| Escalon 5 / 7 / 9 cm | **40** / 13 / 1 | 21 / 2 / 0 | 28 / 1 / 0 | 29 / 1 / 0 | — |
| Pendiente 20 / 22 / 24 / 26° | 0 / 0 / 0 / 0 | 1 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | **90 / 59 / 27 / 0** |
| Pendiente 28 / 30 / 32 / 34° | 0 | 0 | 0 | 0 | 0 |
| Vuelcos en pendientes (de ~800 ep.) | 21 | 30 | **128** | 7 | 4 |

Distancia recorrida en 60 s en denso nivel 12 (bloques): `clip90_cero` 3.75 m, `denso1` 4.62 m, `robot_v1` 3.95 m. En llano a 0.15 m/s ninguna politica supera 0.13 m/s de media (el limite fisico es 0.167).

### 4.2 Errores de seguimiento, atasco y saturacion

| | `denso1` llano | `denso1` denso n12 | `robot_v1` 9999 llano | `robot_v1` 9999 denso n12 |
| --- | --- | --- | --- | --- |
| error medio de vx (comando activo) | 0.017 m/s | 0.084 m/s | 0.019 m/s | 0.084 m/s |
| error medio de wz | 0.034 rad/s | 0.105 rad/s | 0.033 rad/s | 0.091 rad/s |
| tiempo atascado (bloques / losas) | 0 s | 25 s / 32 s de 60 | 0 s | 28 s / 33 s de 60 |
| velocidad objetivo de rueda | 66 rad/s | 31 rad/s | 2.00 rad/s | 2.00 rad/s |
| tiempo con par de traccion recortado | 98 % | 95 % | 4 % | 5 % |
| tiempo con par de direccion recortado | 73 % | 78 % | 42 % | 49 % |
| acciones de direccion fuera de ±1 | 44 % | 53 % | 59 % | 65 % |

## 5. Comparacion aislada de los recortes

*Primera entrega (2026-09-20). Las cifras de `robot_v1` de esta seccion se midieron con ruido en el actor y sin cuota exacta; ver Revision 2 para las corregidas.*

`denso1` evaluado con la misma semilla en cuatro variantes: A original, B recorte de traccion (`clip={".*": (-2.045, 2.045)}` en `JointVelocityActionCfg`), C recorte de direccion (±pi/2), D ambos. El recorte actua sobre el objetivo escalado: velocidad objetivo media 2.02 rad/s en B y D frente a 30.6 en A y C.

| Nivel | Variante | Bloques | Losas | Rejilla | Rugoso |
| --- | --- | --- | --- | --- | --- |
| 12 | A | 16 | 2.5 | 3 | 41 |
| 12 | B | 15 | 4 | 3 | 46 |
| 12 | C | 16 | 3 | 2 | 40 |
| 12 | D | 12 | 10 | 4 | 43 |
| 6 | A | 67 | 27 | 12 | 52 |
| 6 | B | 73 | 25 | 13 | 54 |
| 6 | D | 74 | 25 | 14 | 52 |
| 0 | A | 99 | 94 | 91 | 89 |
| 0 | B | 100 | 95 | 94 | 93 |
| 0 | D | 100 | 96 | 94 | 95 |

Diferencias dentro del ruido (n = 80-160 por celda). **Acotar las acciones no cuesta rendimiento y elimina la saturacion** (par de traccion recortado del 95-99 % al 2-4 %). La prediccion previa de que la politica dependia de objetivos por encima del limite era incorrecta: con `velocity_limit_sim` la rueda nunca pasaba de 2.045 rad/s y a rueda casi parada el par es el mismo pida 2 o 46 rad/s.

## 6. Observaciones disponibles en el robot

Sensores confirmados: encoders en las seis ruedas y en los cuatro reductores, IMU. Mapa de alturas previsto desde la ZED 2 (formato, resolucion y latencia por definir). Sin sensores en la suspension.

| Entrada | Dim | Sensor | Actor | Critico |
| --- | --- | --- | --- | --- |
| Velocidad angular del chasis | 3 | Giroscopo IMU | si | si |
| Gravedad proyectada | 3 | Orientacion IMU | si | si |
| Velocidad lineal del chasis | 3 | **Estimacion** a bordo (odometria de ruedas + IMU); falta decidir quien la calcula y con que ruido | si | si |
| Comando de velocidad | 3 | Nav2 | si | si |
| Posicion de reductores | 4 | Encoders | si | (en `joint_pos` 21) |
| Velocidad de ruedas y reductores | 10 | Encoders | si | (en `joint_vel` 21) |
| Posicion y velocidad de 11 joints pasivos | 22 | ninguno | **no** | si |
| Ultima accion | 10 | interno | si | si |
| Height scan 21 × 18 a 5 cm | 378 | ZED 2 | si | si |
| **Total** | | | **414** | **442** |

El critico conserva la informacion privilegiada de simulacion sin ruido (`obs_groups = {"policy": ["policy"], "critic": ["critic"]}`). MLP sin memoria. No se rellenan con ceros ni se reutilizan checkpoints de otra dimension.

## 7. Entrenamiento nuevo: `robot_v1`

*Primera entrega (2026-09-20). Las cifras de `robot_v1` de esta seccion se midieron con ruido en el actor y sin cuota exacta; ver Revision 2 para las corregidas.*

Tarea `Isaac-Rover-Robert-Denso-v1`: terreno denso, curriculum, comandos, recompensas, terminaciones, eventos y PPO identicos a `denso1`; cambian solo el actor (seccion 6) y los dos recortes de accion. Desde cero, 6144 entornos, 10 000 iteraciones (10 h), checkpoints cada 250, semilla 42. Logs en `logs/rsl_rl/rover_robert_v1/`.

Curvas de entrenamiento (valor final; maximo e iteracion):

| Metrica | Final | Maximo |
| --- | --- | --- |
| `Curriculum/terreno` | 13.0 | 13.1 (it. 9612) — `denso1` llego a 12 |
| `Curriculum/nivel_min` / `nivel_max` | 0 / 19 | 5 (it. 1789) / 19 (it. 2168) |
| `Episode_Reward/seguir_velocidad` | 0.89 | 2.24 (it. 145, terreno facil) |
| `Episode_Reward/seguir_rumbo` | 0.94 | 1.16 (it. 385) |
| `Episode_Reward/penaliza_par` | -0.0024 | (antes -0.004) |
| `Episode_Reward/penaliza_cambio_accion` | -0.32 | (antes -0.38 a -0.42) |
| `Episode_Termination` tiempo / encallado / vuelco / inestable | 99.5 % / 0.5 % / 0.08 % / 0 | inestable 0.05 % en it. 3456, sin NaN |
| `Policy/mean_std` | **1.66, subiendo** | (inicial 1.0) |
| `Train/mean_reward` | 83.6 | 192.9 (it. 257) |

Evaluacion fija por checkpoint, denso nivel 12 (bloques / losas / rejilla / rugoso):

| it. | 1000 | 2000 | 4000 | 5000 | 6000 | 7000 | 8000 | 9000 | 9999 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| % exito | 8/5/3/39 | 8/2/4/36 | 5/4/3/33 | 12/3/2/33 | 11/3/2/33 | 7/3/4/36 | **13/4/4/39** | 10/2/3/35 | 10/2/2/35 |
| distancia bloques | 3.40 m | 3.10 | 3.60 | 3.65 | 3.79 | 3.93 | 3.95 | 3.81 | — |

**La evaluacion fija es plana desde la iteracion 1000** mientras `Curriculum/terreno` subia de 9 a 13: el nivel medio del curriculum no es un buen criterio de eleccion, como advertia el plan. Con n = 80-160 por celda las diferencias de 2-5 puntos son ruido.

**Eleccion.** En distribucion, `model_8000` y `model_9999` son indistinguibles. Fuera de distribucion (rampas), `model_8000` vuelca 128 veces de ~850 episodios y `model_9999` 7. Se propone **`model_9999`**; `model_8000` queda documentado como alternativa si solo cuenta el conjunto fijo en distribucion.

**Comparacion con `denso1`.** Con 28 entradas menos y acciones acotadas, `robot_v1` iguala a `denso1` en denso y llano (mejor en bloques de nivel 6: 81 frente a 67 %; peor en rejilla de nivel 0: 85-89 frente a 91 %), reduce la deriva en parada de 1.17 a 0.5 m y la saturacion de traccion del 95 al 4 %. No es una comparacion limpia: `denso1` acumula 11 000 iteraciones con otra historia. La politica de control (observaciones completas, desde cero, mismos recortes, terreno y semilla) no se ha entrenado; son 10 h de GPU y queda como pendiente declarado.

`Policy/mean_std` crece hasta 1.66 porque con las acciones recortadas una desviacion grande sale gratis y el termino de entropia (0.005) la empuja: la politica estocastica del entrenamiento vive de la saturacion. Es otra razon para decidir por evaluacion determinista.

## 8. Fallos observados y posibles explicaciones

*Primera entrega (2026-09-20). Las cifras de `robot_v1` de esta seccion se midieron con ruido en el actor y sin cuota exacta; ver Revision 2 para las corregidas.*

Separados: lo observado es medido; la explicacion es hipotesis salvo que se indique.

| Observado | Explicacion propuesta | Estado |
| --- | --- | --- |
| Ninguna politica sin entrenamiento en rampas sube 20° (0 %, 99 % atascada) | Fuera de distribucion: ni la fase 1 ni el denso tienen rampas. `pendiente2`, entrenada en ellas, sube 20° al 90 % | Confirmado por comparacion |
| `robot_v1` pone las ruedas de las esquinas a ±90° el 65 % del tiempo en denso y el 96 % sobre rampas; `model_8000` vuelca 128 veces en rampas | Hipotesis: estrategia de pivote aprovechando que las ruedas medias son fijas, degenerada fuera de distribucion. Con clip ya no satura el par, pero el comportamiento persiste | **Pendiente de ver en el viewport** |
| Con comando cero el rover deriva 0.4-1.2 m en 60 s | Solo el 2 % de los entornos de entrenamiento reciben comando cero (`rel_standing_envs=0.02`) y con `std=0.05` el tracking apenas distingue 0 de 0.02 m/s | Hipotesis; no se cambio en este experimento por indicacion del plan |
| A 0.15 m/s ninguna politica pasa de 0.13 m/s de media | Incluye la fase de alineacion inicial y la perdida en giros; el tope fisico es 0.167 | Parcial |
| Escalon aislado de 5 cm solo al 28-40 %; 7 cm al 1-13 % | El techo de 7-9 cm de la fase 2 era de `escalera6`, una politica especializada. Las candidatas se atascan al pie del escalon (50-95 % del tiempo) | Confirmado |
| Rejilla es el tipo mas duro del denso incluso en nivel 6 (celdas de 4.5 cm): 9-18 % | Hipotesis: las ruedas caen en celdas hundidas y quedan encajonadas, o el chasis apoya | Pendiente de ver en el viewport |
| Evaluacion fija plana desde it. 1000 mientras el curriculum sube | El curriculum promociona por distancia recorrida en su parcela, con rumbos aleatorios; la evaluacion fija exige avanzar sin atascarse. Miden cosas distintas | Confirmado |
| `Curriculum/nivel_min` siempre 0 | Algun subconjunto de rovers no sale del nivel 0; la evaluacion por tipo apunta a losas y rejilla | Parcial |
| Test fisico de la fase 2 (traccion maxima, no subia 2 cm) | Nunca se repitio con el contador de terminaciones; la sospecha (`encallado` reseteaba) sigue sin confirmar | Abierto |

## 9. Que se entrega y donde

| Que | Donde |
| --- | --- |
| Configuracion efectiva de cada variante | `params/env.yaml` y `agent.yaml` de cada run; volcados de `referencia_reproducible.py` en `~/robert_referencia/` |
| Cambios exactos de cada variante | [referencia-evaluacion.md](referencia-evaluacion.md): `rover_pendiente_v1_env_cfg.py`, `rover_denso_eval_cfg.py` (A/B/C/D), `rover_eval_extra_cfg.py` (llano, escalon), `rover_robot_v1_cfg.py` |
| Resultados de evaluacion (json + series temporales) | `~/robert_eval/<etiqueta>/resumen.json`, `resumen.txt`, `series.npz` |
| Scripts | `scripts/referencia_reproducible.py`, `evaluar_rover.py`, `tabla_eval.py`, `resumen_tb.py` |
| Videos | **Pendientes**: avance/giro/parada, denso, rampa y escalon con `play.py --video` |
| Checkpoint propuesto | `logs/rsl_rl/rover_robert_v1/2026-09-19_*_robot_v1/model_9999.pt` (copiar a `~/robert_referencia/`) |

## 10. Pendiente, en orden

1. Ver en el viewport el comportamiento de direccion de `robot_v1` (rampa y rejilla) y grabar los videos.
2. Politica de control: `Denso-v1` con observaciones completas, desde cero, mismos recortes y semilla (10 h). Sin ella, la diferencia `robot_v1` vs `denso1` no se puede atribuir a las observaciones.
3. Rampas de 10 y 15° en el escenario de pendientes, para que las politicas de denso tengan algo que medir por debajo de 20°.
4. Evaluacion con friccion 0.6 y 1.0 fijando suelo **y** ruedas (con `friction_combine_mode="max"` no basta cambiar uno).
5. Decidir el estimador de velocidad lineal a bordo y el formato del mapa de alturas de la ZED 2 antes de cualquier exportacion.
6. Robustez (friccion, latencia, ruido) y exportacion a ROS 2: despues de revisar esta entrega, como pedia el plan.
