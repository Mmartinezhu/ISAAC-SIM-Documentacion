# Referencia completa: evaluacion fija, instrumentacion y tarea `robot_v1`

Codigo de la fase descrita en [siguiente-fase.md](siguiente-fase.md), tal como quedo en la maquina el 2026-09-20. Todo va en `source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/rover/` salvo los scripts, que van en `scripts/`. Hereda de las tareas de [referencia-completa.md](referencia-completa.md) y [referencia-fases-2-4.md](referencia-fases-2-4.md) sin modificarlas.

Convenciones comunes:
- Un termino de curriculum solo registra el ultimo lote de resets de cada iteracion; para contar hay que acumular dentro de una clase (`ManagerTermBase`).
- Un termino de terminacion que siempre devuelve `False` es el gancho mas limpio para medir cada paso.
- `CurriculumManager` no tiene `get_term_cfg`; se accede con `dict(zip(cm._term_names, cm._term_cfgs))`.
- Con `curriculum=False` el generador reparte los tipos de sub-terreno al azar por parcela; agrupar por columna exige `curriculum=True`.

---

## 1. `rover_pendiente_v1_env_cfg.py`

```python
"""Pendientes v1: misma fisica, recompensas y PPO que v0. Cambian el reset y las metricas.

Umbrales (documentados aqui y solo aqui):
- ESPERA_S = 1.0 s: asentamiento tras el reset. No se paga altura ni se mide; un episodio que
  termina antes cuenta como intento fallido "temprano".
- Atasco: comando de avance > ATASCO_CMD_MIN (0.03 m/s; el minimo comandado es 0.05) y
  progreso radial (incremento de la distancia Chebyshev al centro, CON SIGNO: resbalar cuesta
  abajo es negativo) < ATASCO_PROGRESO_MIN (0.10 m) en una ventana de ATASCO_VENTANA_S (5 s),
  es decir < 0.02 m/s = 40 % del comando minimo. Independiente del contacto del chasis.
- Linea de llegada: borde exterior de la rampa inicial (distancia Chebyshev al centro >= r_meta),
  mantenida LLEGADA_ESTABLE_S (1 s) seguidos. Es el criterio oficial de exito.
- Exito por altura (criterio v0, se conserva para comparar): record de z >= inicio de la rampa
  siguiente - TOLERANCIA_ALTURA (3 cm).
"""

import math
from collections.abc import Sequence

import torch

import isaaclab.utils.math as math_utils
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ManagerTermBase, RewardTermCfg, SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

from .rover_escalera_env_cfg import _COTA_ROOT
from .rover_pendiente_env_cfg import ANCHO, ANGULOS, PLATAFORMA, RoverPendienteEnvCfg, _INICIOS, _rampa_y_cuesta

ESPERA_S = 1.0
ATASCO_VENTANA_S = 5.0
ATASCO_CMD_MIN = 0.03
ATASCO_PROGRESO_MIN = 0.10
LLEGADA_ESTABLE_S = 1.0
TOLERANCIA_ALTURA = 0.03

N_RAMPAS = len(ANGULOS)
ETIQUETAS = [f"{a}deg" for a in ANGULOS]


def _r_meta(k: torch.Tensor) -> torch.Tensor:
    """Borde exterior de la rampa k (Chebyshev). Para la plataforma (k=-1), el final de la primera rampa."""
    return 0.5 * PLATAFORMA + (k.clamp(min=-1) + 1).float() * ANCHO


class ResetEnRampaV1(ManagerTermBase):
    """Coloca al rover en una rampa mirando cuesta arriba y GUARDA la rampa inicial del episodio.

    posiciones_fijas=False: parche aleatorio de cualquier rampa (como v0).
    posiciones_fijas=True (evaluacion): rampa = i % 8, lado = (i // 8) % 4, desplazamiento a lo
    largo del lado = uno de 5 valores fijos; siempre 25 cm dentro de la rampa. Sin aleatoriedad.
    """

    def __init__(self, cfg: EventTerm, env):
        super().__init__(cfg, env)
        self.rampa_inicial = torch.full((env.num_envs,), -2, dtype=torch.long, device=env.device)
        i = torch.arange(env.num_envs, device=env.device)
        self._k_fijo = i % N_RAMPAS
        lado = (i // N_RAMPAS) % 4
        t = ((i // (4 * N_RAMPAS)) % 5 - 2).float() * 0.5  # -1, -0.5, 0, 0.5, 1 m a lo largo del lado
        r = 0.5 * PLATAFORMA + self._k_fijo.float() * ANCHO + 0.25
        x = torch.where(lado == 0, r, torch.where(lado == 1, -r, t))
        y = torch.where(lado == 2, r, torch.where(lado == 3, -r, t))
        inicios = torch.tensor(_INICIOS, device=env.device)
        ang = torch.tensor([math.radians(a) for a in ANGULOS], device=env.device)
        z = inicios[self._k_fijo] + 0.25 * torch.tan(ang[self._k_fijo])
        self._pos_fija = torch.stack([x, y, z], dim=1)

    def __call__(self, env, env_ids, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
                 posiciones_fijas: bool = False, holgura: float = 0.10):
        asset = env.scene[asset_cfg.name]
        if posiciones_fijas:
            pos = self._pos_fija[env_ids] + env.scene.env_origins[env_ids]
        else:
            terrain = env.scene.terrain
            parches = terrain.flat_patches["init_pos"]
            ids = torch.randint(0, parches.shape[2], (len(env_ids),), device=env.device)
            pos = parches[terrain.terrain_levels[env_ids], terrain.terrain_types[env_ids], ids].clone()
        k, theta, cuesta = _rampa_y_cuesta(env, env_ids, pos)
        self.rampa_inicial[env_ids] = k
        yaw = torch.atan2(cuesta[:, 1], cuesta[:, 0])
        yaw = torch.where(k < 0, torch.empty_like(yaw).uniform_(-math.pi, math.pi), yaw)
        quat = math_utils.quat_from_euler_xyz(torch.zeros_like(yaw), -theta, yaw)
        pos[:, 2] += -_COTA_ROOT + holgura
        asset.write_root_pose_to_sim(torch.cat([pos, quat], dim=-1), env_ids=env_ids)
        asset.write_root_velocity_to_sim(torch.zeros(len(env_ids), 6, device=env.device), env_ids=env_ids)


class AlturaGanadaV1(ManagerTermBase):
    """Identica a AlturaGanada en lo que paga. Corrige la contabilidad: z_spawn se fija en cada
    reset y se marca `asentado` al acabar la espera, asi que nunca hereda del episodio anterior."""

    def __init__(self, cfg: RewardTermCfg, env):
        super().__init__(cfg, env)
        self.asset = env.scene[cfg.params["asset_cfg"].name]
        n = env.num_envs
        self.z_max = torch.zeros(n, device=env.device)
        self.z_spawn = torch.zeros(n, device=env.device)
        self.asentado = torch.zeros(n, dtype=torch.bool, device=env.device)
        self.espera = torch.zeros(n, dtype=torch.long, device=env.device)
        self.pasos_espera = int(ESPERA_S / env.step_dt)

    def _z(self, env_ids=slice(None)):
        return self.asset.data.root_pos_w[env_ids, 2] - self._env.scene.env_origins[env_ids, 2]

    def reset(self, env_ids: Sequence[int] | None = None):
        if env_ids is None:
            env_ids = slice(None)
        z = self._z(env_ids)
        self.z_max[env_ids] = z
        self.z_spawn[env_ids] = z
        self.asentado[env_ids] = False
        self.espera[env_ids] = self.pasos_espera

    def __call__(self, env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
        z = self._z()
        activo = self.espera <= 0
        recien = self.espera == 1
        self.z_spawn = torch.where(recien, z, self.z_spawn)
        self.asentado |= recien
        ganancia = torch.where(activo, torch.clamp(z - self.z_max, min=0.0), torch.zeros_like(z))
        self.z_max = torch.where(activo, torch.maximum(self.z_max, z), z)
        self.espera -= 1
        return ganancia / env.step_dt


class RastreoEpisodio(ManagerTermBase):
    """Termino de terminacion que NUNCA termina: es el gancho para medir cada paso."""

    def __init__(self, cfg: DoneTerm, env):
        super().__init__(cfg, env)
        n, dev = env.num_envs, env.device
        self.asset = env.scene["robot"]
        self.pasos_1s = int(1.0 / env.step_dt)
        self.n_hist = int(ATASCO_VENTANA_S) + 1  # muestras a 1 s
        self.hist = torch.zeros(n, self.n_hist, device=dev)  # distancia Chebyshev al centro, a 1 s
        self.edad = torch.zeros(n, dtype=torch.long, device=dev)
        self.pasos_atascado = torch.zeros(n, dtype=torch.long, device=dev)
        self.tras_linea = torch.zeros(n, dtype=torch.long, device=dev)
        self.llego = torch.zeros(n, dtype=torch.bool, device=dev)
        self.pasos_llegada = int(LLEGADA_ESTABLE_S / env.step_dt)
        self.pasos_ventana = int(ATASCO_VENTANA_S / env.step_dt)

    def _xy(self, env_ids=slice(None)):
        return self.asset.data.root_pos_w[env_ids, :2] - self._env.scene.env_origins[env_ids, :2]

    def reset(self, env_ids: Sequence[int] | None = None):
        if env_ids is None:
            env_ids = slice(None)
        xy = self._xy(env_ids)
        self.hist[env_ids] = torch.maximum(xy[:, 0].abs(), xy[:, 1].abs()).unsqueeze(1)
        self.edad[env_ids] = 0
        self.pasos_atascado[env_ids] = 0
        self.tras_linea[env_ids] = 0
        self.llego[env_ids] = False

    def __call__(self, env) -> torch.Tensor:
        xy = self._xy()
        r = torch.maximum(xy[:, 0].abs(), xy[:, 1].abs())
        self.edad += 1
        rotar = (self.edad % self.pasos_1s) == 0
        if rotar.any():
            self.hist[rotar] = torch.roll(self.hist[rotar], shifts=1, dims=1)
            self.hist[rotar, 0] = r[rotar]
        progreso = r - self.hist[:, -1]  # con signo: retroceder no cuenta como moverse
        cmd_x = env.command_manager.get_command("base_velocity")[:, 0]
        atascado = (self.edad >= self.pasos_ventana) & (cmd_x > ATASCO_CMD_MIN) & (progreso < ATASCO_PROGRESO_MIN)
        self.pasos_atascado += atascado.long()
        k = env.event_manager.get_term_cfg("reset_en_rampa").func.rampa_inicial
        self.tras_linea = torch.where(r >= _r_meta(k), self.tras_linea + 1, torch.zeros_like(self.tras_linea))
        self.llego |= self.tras_linea >= self.pasos_llegada
        return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)


class MetricasRampa(ManagerTermBase):
    """Contadores ENTEROS acumulados por rampa (y plataforma aparte), mas una tasa EMA solo
    como indicador de entrenamiento. Se evalua en cada reset, antes de recolocar."""

    OLVIDO = 0.9995

    def __init__(self, cfg: CurrTerm, env):
        super().__init__(cfg, env)
        dev = env.device
        m = N_RAMPAS + 1  # ultimo indice = plataforma
        self.intentos = torch.zeros(m, device=dev)
        self.exitos = torch.zeros(m, device=dev)          # linea de llegada (oficial)
        self.exitos_altura = torch.zeros(m, device=dev)   # criterio v0
        self.tempranos = torch.zeros(m, device=dev)
        self.atascados = torch.zeros(m, device=dev)
        self.tiempo_atascado = torch.zeros(m, device=dev)
        self.ema_exitos = torch.zeros(m, device=dev)
        self.ema_intentos = torch.zeros(m, device=dev)
        self.inicios = torch.tensor(_INICIOS, device=dev)

    def __call__(self, env, env_ids, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
        reset = env.event_manager.get_term_cfg("reset_en_rampa").func
        premio = env.reward_manager.get_term_cfg("altura_ganada").func
        rastreo = env.termination_manager.get_term_cfg("rastreo").func
        k = reset.rampa_inicial[env_ids]
        validos = k >= -1  # -2 = nunca reseteado
        if not validos.any():
            return {}
        k = k[validos]
        ids = env_ids[validos] if torch.is_tensor(env_ids) else torch.as_tensor(env_ids, device=env.device)[validos]
        grupo = torch.where(k < 0, torch.full_like(k, N_RAMPAS), k)
        temprano = ~premio.asentado[ids]
        llego = rastreo.llego[ids] & ~temprano
        kr = k.clamp(min=0)
        altura_ok = (premio.z_max[ids] + _COTA_ROOT >= self.inicios[kr + 1] - TOLERANCIA_ALTURA) & ~temprano
        atascado = rastreo.pasos_atascado[ids] > 0
        t_atascado = rastreo.pasos_atascado[ids].float() * env.step_dt

        def acumula(buf, valores):
            buf += torch.bincount(grupo, weights=valores.float(), minlength=N_RAMPAS + 1)

        acumula(self.intentos, torch.ones_like(k))
        acumula(self.exitos, llego)
        acumula(self.exitos_altura, altura_ok)
        acumula(self.tempranos, temprano)
        acumula(self.atascados, atascado)
        acumula(self.tiempo_atascado, t_atascado)
        self.ema_exitos *= self.OLVIDO
        self.ema_intentos *= self.OLVIDO
        acumula(self.ema_exitos, llego)
        acumula(self.ema_intentos, torch.ones_like(k))
        return self._dict_tensorboard()

    def reiniciar(self):
        """Pone a cero los contadores (el evaluador lo llama tras el reset que fija el nivel)."""
        for c in ("intentos", "exitos", "exitos_altura", "tempranos", "atascados", "tiempo_atascado", "ema_exitos", "ema_intentos"):
            getattr(self, c).zero_()

    def resumen(self):
        out = {}
        for g, nombre in enumerate(ETIQUETAS + ["plataforma"]):
            if self.intentos[g] > 0:
                out[nombre] = {
                    "intentos": int(self.intentos[g]), "exitos": int(self.exitos[g]),
                    "exitos_altura": int(self.exitos_altura[g]), "tempranos": int(self.tempranos[g]),
                    "atascados": int(self.atascados[g]), "t_atascado_medio_s": float(self.tiempo_atascado[g] / self.intentos[g]),
                }
        return out

    def _dict_tensorboard(self):
        salida = {}
        for g, nombre in enumerate(ETIQUETAS + ["plataforma"]):
            if self.intentos[g] == 0:
                continue
            salida[f"{nombre}/intentos"] = self.intentos[g]
            salida[f"{nombre}/exitos"] = self.exitos[g]
            salida[f"{nombre}/exitos_altura"] = self.exitos_altura[g]
            salida[f"{nombre}/tempranos"] = self.tempranos[g]
            salida[f"{nombre}/atascados"] = self.atascados[g]
            salida[f"{nombre}/t_atascado_medio_s"] = self.tiempo_atascado[g] / self.intentos[g]
            if self.ema_intentos[g] > 0:
                salida[f"{nombre}/tasa_ema"] = self.ema_exitos[g] / self.ema_intentos[g]
        return salida


@configclass
class RoverPendienteV1EnvCfg(RoverPendienteEnvCfg):
    posiciones_fijas: bool = False

    def __post_init__(self):
        super().__post_init__()
        self.events.reset_en_rampa = EventTerm(
            func=ResetEnRampaV1, mode="reset", params={"posiciones_fijas": self.posiciones_fijas}
        )
        self.rewards.altura_ganada = RewardTermCfg(
            func=AlturaGanadaV1, weight=200.0, params={"asset_cfg": SceneEntityCfg("robot")}
        )
        self.terminations.rastreo = DoneTerm(func=RastreoEpisodio)  # nunca termina: solo mide
        self.curriculum.rampa = None
        self.curriculum.metricas = CurrTerm(func=MetricasRampa)


@configclass
class RoverPendienteV1EvalCfg(RoverPendienteV1EnvCfg):
    """Evaluacion: posiciones fijas, sin ruido en observaciones. La semilla la fija el script."""

    posiciones_fijas: bool = True

    def __post_init__(self):
        super().__post_init__()
        self.observations.policy.enable_corruption = False
```

---

## 2. `rover_denso_eval_cfg.py` (variantes A/B/C/D)

```python
"""Evaluacion de denso1: curriculum congelado, nivel fijo (lo escribe el script), sin ruido,
metricas por tipo de terreno, y cuatro variantes de recorte de acciones (punto 5 del plan).

Umbrales:
- Atasco: comando de avance > 0.03 m/s y desplazamiento xy < 0.10 m en 5 s.
- Exito: episodio que llega al time_out sin vuelco y con tiempo atascado < 10 % del episodio.
"""

import math
from collections.abc import Sequence

import numpy as np
import torch

from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import ManagerTermBase
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

from .rover_denso_env_cfg import RoverDensoEnvCfg
from .rover_env_cfg import VEL_RUEDA_MAX

ATASCO_VENTANA_S = 5.0
ATASCO_CMD_MIN = 0.03
ATASCO_PROGRESO_MIN = 0.10
EXITO_FRAC_ATASCADO_MAX = 0.10


def _nombres_por_columna(terrain) -> list[str]:
    """Tipo de sub-terreno de cada columna, segun las proporciones del generador (requiere curriculum=True)."""
    subs = terrain.cfg.terrain_generator.sub_terrains
    nombres = list(subs.keys())
    prop = np.array([subs[k].proportion for k in nombres], dtype=float)
    acum = np.cumsum(prop) / prop.sum()
    num_cols = terrain.cfg.terrain_generator.num_cols
    return [nombres[int(np.searchsorted(acum, (c + 0.5) / num_cols))] for c in range(num_cols)]


class RastreoDenso(ManagerTermBase):
    """Terminacion que nunca termina: acumula por entorno atasco, distancia y errores de seguimiento."""

    def __init__(self, cfg: DoneTerm, env):
        super().__init__(cfg, env)
        n, dev = env.num_envs, env.device
        self.asset = env.scene["robot"]
        self.pasos_1s = int(1.0 / env.step_dt)
        self.hist = torch.zeros(n, int(ATASCO_VENTANA_S) + 1, 2, device=dev)
        self.edad = torch.zeros(n, dtype=torch.long, device=dev)
        self.pasos_atascado = torch.zeros(n, dtype=torch.long, device=dev)
        self.distancia = torch.zeros(n, device=dev)
        self.err_vx = torch.zeros(n, device=dev)
        self.err_wz = torch.zeros(n, device=dev)
        self.pasos_cmd = torch.zeros(n, device=dev)
        self.xy_prev = torch.zeros(n, 2, device=dev)
        self.pasos_ventana = int(ATASCO_VENTANA_S / env.step_dt)

    def _xy(self, env_ids=slice(None)):
        return self.asset.data.root_pos_w[env_ids, :2]

    def reset(self, env_ids: Sequence[int] | None = None):
        if env_ids is None:
            env_ids = slice(None)
        xy = self._xy(env_ids)
        self.hist[env_ids] = xy.unsqueeze(1)
        self.xy_prev[env_ids] = xy
        self.edad[env_ids] = 0
        self.pasos_atascado[env_ids] = 0
        self.distancia[env_ids] = 0.0
        self.err_vx[env_ids] = 0.0
        self.err_wz[env_ids] = 0.0
        self.pasos_cmd[env_ids] = 0.0

    def __call__(self, env) -> torch.Tensor:
        xy = self._xy()
        self.edad += 1
        self.distancia += torch.norm(xy - self.xy_prev, dim=1)
        self.xy_prev = xy.clone()
        rotar = (self.edad % self.pasos_1s) == 0
        if rotar.any():
            self.hist[rotar] = torch.roll(self.hist[rotar], shifts=1, dims=1)
            self.hist[rotar, 0] = xy[rotar]
        progreso = torch.norm(xy - self.hist[:, -1], dim=1)
        cmd = env.command_manager.get_command("base_velocity")
        activo = cmd[:, 0].abs() > ATASCO_CMD_MIN
        self.pasos_atascado += ((self.edad >= self.pasos_ventana) & activo & (progreso < ATASCO_PROGRESO_MIN)).long()
        vb = self.asset.data.root_lin_vel_b
        wb = self.asset.data.root_ang_vel_b
        self.err_vx += (cmd[:, 0] - vb[:, 0]).abs() * activo
        self.err_wz += (cmd[:, 2] - wb[:, 2]).abs()
        self.pasos_cmd += activo.float()
        return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)


class MetricasDenso(ManagerTermBase):
    """Contadores enteros por grupo. Por defecto el grupo es el tipo de terreno de la columna;
    las subclases (llano, escalon) redefinen _tipos/_grupo/_exito."""

    def __init__(self, cfg: CurrTerm, env):
        super().__init__(cfg, env)
        self.tipos = self._tipos(env)
        m = len(self.tipos)
        dev = env.device
        self.intentos = torch.zeros(m, device=dev)
        self.exitos = torch.zeros(m, device=dev)
        self.vuelcos = torch.zeros(m, device=dev)
        self.timeouts = torch.zeros(m, device=dev)
        self.atascados = torch.zeros(m, device=dev)
        self.t_atascado = torch.zeros(m, device=dev)
        self.distancia = torch.zeros(m, device=dev)
        self.err_vx = torch.zeros(m, device=dev)
        self.err_wz = torch.zeros(m, device=dev)
        self.pasos_cmd = torch.zeros(m, device=dev)
        self.pasos = torch.zeros(m, device=dev)
        self.primer_reset = True

    def _tipos(self, env) -> list[str]:
        terrain = env.scene.terrain
        self.nombres_col = _nombres_por_columna(terrain)
        tipos = sorted(set(self.nombres_col))
        idx = {t: i for i, t in enumerate(tipos)}
        self.col_a_tipo = torch.tensor([idx[nc] for nc in self.nombres_col], device=env.device)
        return tipos

    def _grupo(self, env, ids):
        return self.col_a_tipo[env.scene.terrain.terrain_types[ids]]

    def _exito(self, env, ids, rastreo, volco, timeout):
        frac_atascado = rastreo.pasos_atascado[ids].float() / rastreo.edad[ids].clamp(min=1).float()
        return timeout & ~volco & (frac_atascado < EXITO_FRAC_ATASCADO_MAX)

    def __call__(self, env, env_ids):
        if self.primer_reset:  # el reset inicial de la simulacion no es un episodio
            self.primer_reset = False
            return {}
        ids = env_ids if torch.is_tensor(env_ids) else torch.as_tensor(env_ids, device=env.device)
        rastreo = env.termination_manager.get_term_cfg("rastreo").func
        grupo = self._grupo(env, ids)
        tm = env.termination_manager
        volco = tm.get_term("volcado")[ids]
        timeout = tm.get_term("tiempo_agotado")[ids]
        exito = self._exito(env, ids, rastreo, volco, timeout)
        m = len(self.tipos)

        def acumula(buf, val):
            buf += torch.bincount(grupo, weights=val.float(), minlength=m)

        acumula(self.intentos, torch.ones_like(grupo))
        acumula(self.exitos, exito)
        acumula(self.vuelcos, volco)
        acumula(self.timeouts, timeout)
        acumula(self.atascados, rastreo.pasos_atascado[ids] > 0)
        acumula(self.t_atascado, rastreo.pasos_atascado[ids].float() * env.step_dt)
        acumula(self.distancia, rastreo.distancia[ids])
        acumula(self.err_vx, rastreo.err_vx[ids])
        acumula(self.err_wz, rastreo.err_wz[ids])
        acumula(self.pasos_cmd, rastreo.pasos_cmd[ids])
        acumula(self.pasos, rastreo.edad[ids].float())
        return {f"{t}/exitos": self.exitos[i] for i, t in enumerate(self.tipos)} | {
            f"{t}/intentos": self.intentos[i] for i, t in enumerate(self.tipos)}

    def reiniciar(self):
        for c in ("intentos", "exitos", "vuelcos", "timeouts", "atascados", "t_atascado", "distancia", "err_vx", "err_wz", "pasos_cmd", "pasos"):
            getattr(self, c).zero_()

    def resumen(self):
        out = {}
        for i, t in enumerate(self.tipos):
            n = self.intentos[i]
            if n == 0:
                continue
            out[t] = {
                "intentos": int(n), "exitos": int(self.exitos[i]), "vuelcos": int(self.vuelcos[i]),
                "timeouts": int(self.timeouts[i]), "atascados": int(self.atascados[i]),
                "t_atascado_medio_s": float(self.t_atascado[i] / n),
                "distancia_media_m": float(self.distancia[i] / n),
                "err_vx_medio": float(self.err_vx[i] / self.pasos_cmd[i].clamp(min=1)),
                "err_wz_medio": float(self.err_wz[i] / self.pasos[i].clamp(min=1)),
            }
        return out


@configclass
class RoverDensoEvalCfg(RoverDensoEnvCfg):
    """Variante A: sin recortes (como se entreno denso1). El nivel fijo lo escribe el evaluador."""

    clip_traccion: bool = False
    clip_direccion: bool = False

    def __post_init__(self):
        super().__post_init__()
        self.curriculum.terreno = None  # congelado: nadie cambia de nivel
        self.observations.policy.enable_corruption = False
        self.terminations.rastreo = DoneTerm(func=RastreoDenso)
        self.curriculum.metricas = CurrTerm(func=MetricasDenso)
        if self.clip_traccion:
            self.actions.traccion.clip = {".*": (-VEL_RUEDA_MAX, VEL_RUEDA_MAX)}
        if self.clip_direccion:
            self.actions.direccion.clip = {".*": (-math.pi / 2, math.pi / 2)}


@configclass
class RoverDensoEvalBCfg(RoverDensoEvalCfg):
    clip_traccion: bool = True


@configclass
class RoverDensoEvalCCfg(RoverDensoEvalCfg):
    clip_direccion: bool = True


@configclass
class RoverDensoEvalDCfg(RoverDensoEvalCfg):
    clip_traccion: bool = True
    clip_direccion: bool = True
```

---

## 3. `rover_eval_extra_cfg.py` (llano y escalon aislado)

```python
"""Escenarios de evaluacion que faltaban del punto 4: llano con guiones fijos y escalones aislados.

Llano: terreno plano; cada entorno sigue un guion fijo segun su indice (i % 5):
  0: 0.05 m/s recto   1: 0.10 m/s recto   2: 0.15 m/s recto   3: giro de +90 grados a 0.10 m/s   4: parada
Escalon: plataforma de 3 m y un unico escalon de 5, 7 o 9 cm (una columna por altura); el rover nace
en el centro mirando hacia fuera (+/- 20 grados). Exito = cruzar la linea a 1 m del escalon y
mantenerse 1 s (LINEA_ESCALON = 1.5 + 1.0 m Chebyshev desde el centro).
"""

import math
from collections.abc import Sequence

import torch

from isaaclab.envs.mdp.commands.velocity_command import UniformVelocityCommand
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.terrains import FlatPatchSamplingCfg, MeshPlaneTerrainCfg, TerrainGeneratorCfg
from isaaclab.utils import configclass

from .rover_denso_eval_cfg import MetricasDenso, RastreoDenso, RoverDensoEvalCfg
from .rover_escalera_env_cfg import RoverEscaleraEnvCfg
from .rover_pendiente_env_cfg import ComandoRadialPendiente
from .terrenos_rover import EscaleraProgresivaCfg

GUIONES = ["v0.05", "v0.10", "v0.15", "giro90", "parada"]
VEL_GUION = [0.05, 0.10, 0.15, 0.10, 0.0]
LINEA_ESCALON = 2.5
LLEGADA_ESTABLE_S = 1.0


class ComandoGuion(UniformVelocityCommand):
    """Comando fijo por entorno: velocidad segun el guion y rumbo = guiñada inicial (+90 en 'giro90')."""

    def _resample_command(self, env_ids: Sequence[int]):
        super()._resample_command(env_ids)
        ids = env_ids if torch.is_tensor(env_ids) else torch.as_tensor(env_ids, device=self.device)
        guion = ids % len(GUIONES)
        vel = torch.tensor(VEL_GUION, device=self.device)[guion]
        self.vel_command_b[env_ids, 0] = vel
        self.vel_command_b[env_ids, 1] = 0.0
        self.vel_command_b[env_ids, 2] = 0.0
        yaw = self.robot.data.heading_w[env_ids]
        self.heading_target[env_ids] = yaw + torch.where(guion == 3, math.pi / 2, 0.0)
        self.is_standing_env[env_ids] = guion == 4


class MetricasLlano(MetricasDenso):
    def _tipos(self, env):
        return list(GUIONES)

    def _grupo(self, env, ids):
        return ids % len(GUIONES)


LLANO_CFG = TerrainGeneratorCfg(
    size=(8.0, 8.0), border_width=10.0, num_rows=2, num_cols=2, use_cache=False, curriculum=False,
    sub_terrains={"plano": MeshPlaneTerrainCfg(proportion=1.0)},
)


@configclass
class RoverLlanoEvalCfg(RoverDensoEvalCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.terrain.terrain_generator = LLANO_CFG
        self.commands.base_velocity.class_type = ComandoGuion
        self.commands.base_velocity.resampling_time_range = (self.episode_length_s, self.episode_length_s)
        self.curriculum.metricas = CurrTerm(func=MetricasLlano)


class RastreoLinea(RastreoDenso):
    """RastreoDenso + linea de llegada fija (Chebyshev desde el origen del terreno)."""

    def __init__(self, cfg: DoneTerm, env):
        super().__init__(cfg, env)
        self.tras_linea = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self.llego = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        self.pasos_llegada = int(LLEGADA_ESTABLE_S / env.step_dt)

    def reset(self, env_ids=None):
        super().reset(env_ids)
        if env_ids is None:
            env_ids = slice(None)
        self.tras_linea[env_ids] = 0
        self.llego[env_ids] = False

    def __call__(self, env):
        out = super().__call__(env)
        d = self.asset.data.root_pos_w[:, :2] - env.scene.env_origins[:, :2]
        r = torch.maximum(d[:, 0].abs(), d[:, 1].abs())
        self.tras_linea = torch.where(r >= LINEA_ESCALON, self.tras_linea + 1, torch.zeros_like(self.tras_linea))
        self.llego |= self.tras_linea >= self.pasos_llegada
        return out


class MetricasEscalon(MetricasDenso):
    def _exito(self, env, ids, rastreo, volco, timeout):
        return rastreo.llego[ids] & ~volco


def _escalon(altura: float) -> EscaleraProgresivaCfg:
    return EscaleraProgresivaCfg(
        proportion=1.0 / 3.0, subidas=(altura,), step_width=5.0, platform_width=3.0,
        flat_patch_sampling={"init_pos": FlatPatchSamplingCfg(num_patches=500, patch_radius=0.45, max_height_diff=0.02, z_range=(-0.05, 0.01))},
    )


ESCALON_CFG = TerrainGeneratorCfg(
    size=(16.0, 16.0), border_width=0.0, num_rows=2, num_cols=3, use_cache=False, curriculum=True,  # True: columnas por proporcion
    sub_terrains={"escalon_5cm": _escalon(0.05), "escalon_7cm": _escalon(0.07), "escalon_9cm": _escalon(0.09)},
)


@configclass
class RoverEscalonEvalCfg(RoverEscaleraEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.terrain.terrain_generator = ESCALON_CFG
        self.commands.base_velocity.class_type = ComandoRadialPendiente  # hacia fuera +/- 20 grados
        self.observations.policy.enable_corruption = False
        self.terminations.rastreo = DoneTerm(func=RastreoLinea)
        self.curriculum.altura = None
        self.curriculum.exito = None
        self.curriculum.metricas = CurrTerm(func=MetricasEscalon)
```

---

## 4. `rover_robot_v1_cfg.py` (tarea nueva y sus evaluaciones)

```python
"""Tarea nueva (puntos 6 y 7 del plan): politica transferible al robot.

Cambios respecto a denso1, y SOLO estos:
- Actor: solo señales medibles a bordo. Fuera las posiciones y velocidades de las 11 articulaciones
  pasivas. Quedan: IMU (velocidad angular, gravedad proyectada), velocidad lineal del chasis
  (estimada a bordo), comandos, posicion de los 4 reductores, velocidad de las 6 ruedas y los 4
  reductores, ultima accion, height scan. 442 -> 414 entradas.
- Critico: privilegiado, con las 442 entradas originales.
- Acciones acotadas: traccion en +/-2.045 rad/s, direccion en +/-90 grados.
"""

import math

import isaaclab.envs.mdp as mdp
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from .agents.rsl_rl_ppo_cfg import RoverPPORunnerCfg
from .rover_denso_env_cfg import RoverDensoEnvCfg
from .rover_denso_eval_cfg import RoverDensoEvalCfg
from .rover_env_cfg import VEL_RUEDA_MAX, height_scan_seguro
from .rover_eval_extra_cfg import RoverEscalonEvalCfg, RoverLlanoEvalCfg
from .rover_pendiente_v1_env_cfg import RoverPendienteV1EvalCfg

SENSORES_POS = SceneEntityCfg("robot", joint_names=["reductor.*"])
SENSORES_VEL = SceneEntityCfg("robot", joint_names=["llanta.*", "reductor.*"])


@configclass
class ObservacionesRobotCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        """Actor: lo que el robot puede medir o estimar."""

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1), clip=(-10.0, 10.0))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2), clip=(-10.0, 10.0))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        pos_direccion = ObsTerm(
            func=mdp.joint_pos_rel, params={"asset_cfg": SENSORES_POS},
            noise=Unoise(n_min=-0.01, n_max=0.01), clip=(-10.0, 10.0),
        )
        vel_ruedas_direccion = ObsTerm(
            func=mdp.joint_vel_rel, params={"asset_cfg": SENSORES_VEL},
            noise=Unoise(n_min=-0.5, n_max=0.5), clip=(-50.0, 50.0),
        )
        actions = ObsTerm(func=mdp.last_action)
        height_scan = ObsTerm(
            func=height_scan_seguro, params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            noise=Unoise(n_min=-0.02, n_max=0.02), clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        """Critico privilegiado: las 442 entradas originales, sin ruido."""

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, clip=(-10.0, 10.0))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, clip=(-10.0, 10.0))
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, clip=(-10.0, 10.0))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, clip=(-50.0, 50.0))
        actions = ObsTerm(func=mdp.last_action)
        height_scan = ObsTerm(
            func=height_scan_seguro, params={"sensor_cfg": SceneEntityCfg("height_scanner")}, clip=(-1.0, 1.0)
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


def aplicar_robot(cfg):
    """Observaciones del robot + acciones acotadas, sobre cualquier configuracion del rover."""
    cfg.observations = ObservacionesRobotCfg()
    cfg.actions.traccion.clip = {".*": (-VEL_RUEDA_MAX, VEL_RUEDA_MAX)}
    cfg.actions.direccion.clip = {".*": (-math.pi / 2, math.pi / 2)}


@configclass
class RoverPPORunnerRobotCfg(RoverPPORunnerCfg):
    experiment_name = "rover_robert_v1"
    obs_groups = {"policy": ["policy"], "critic": ["critic"]}


@configclass
class RoverDensoV1EnvCfg(RoverDensoEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        aplicar_robot(self)


@configclass
class RoverDensoV1EvalCfg(RoverDensoEvalCfg):
    def __post_init__(self):
        super().__post_init__()
        aplicar_robot(self)


@configclass
class RoverLlanoV1EvalCfg(RoverLlanoEvalCfg):
    def __post_init__(self):
        super().__post_init__()
        aplicar_robot(self)


@configclass
class RoverEscalonV1EvalCfg(RoverEscalonEvalCfg):
    def __post_init__(self):
        super().__post_init__()
        aplicar_robot(self)


@configclass
class RoverPendienteV1RobotEvalCfg(RoverPendienteV1EvalCfg):
    def __post_init__(self):
        super().__post_init__()
        aplicar_robot(self)
```

Volcado efectivo de `Isaac-Rover-Robert-Denso-v1` (con `referencia_reproducible.py`):

```text
[policy]  0-2 base_lin_vel | 3-5 base_ang_vel | 6-8 projected_gravity | 9-11 velocity_commands | 12-15 pos_direccion (4)
          | 16-25 vel_ruedas_direccion (10) | 26-35 actions | 36-413 height_scan (378)      total 414
[critic]  como la politica original                                                          total 442
traccion._clip = ±2.045 en las 6;  direccion._clip = ±1.5708 en los 4
```

---

## 5. Registro en `__init__.py`

```python
gym.register(id="Isaac-Rover-Robert-Pendiente-v1", entry_point="isaaclab.envs:ManagerBasedRLEnv", disable_env_checker=True,
    kwargs={"env_cfg_entry_point": f"{__name__}.rover_pendiente_v1_env_cfg:RoverPendienteV1EnvCfg",
            "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RoverPPORunnerCfg"})
gym.register(id="Isaac-Rover-Robert-Pendiente-v1-Eval", entry_point="isaaclab.envs:ManagerBasedRLEnv", disable_env_checker=True,
    kwargs={"env_cfg_entry_point": f"{__name__}.rover_pendiente_v1_env_cfg:RoverPendienteV1EvalCfg",
            "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RoverPPORunnerCfg"})

for _var, _cls in (("A", "RoverDensoEvalCfg"), ("B", "RoverDensoEvalBCfg"), ("C", "RoverDensoEvalCCfg"), ("D", "RoverDensoEvalDCfg")):
    gym.register(id=f"Isaac-Rover-Robert-Denso-Eval-{_var}", entry_point="isaaclab.envs:ManagerBasedRLEnv", disable_env_checker=True,
        kwargs={"env_cfg_entry_point": f"{__name__}.rover_denso_eval_cfg:{_cls}",
                "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RoverPPORunnerCfg"})

for _id, _cls in (("Isaac-Rover-Robert-Llano-Eval", "RoverLlanoEvalCfg"), ("Isaac-Rover-Robert-Escalon-Eval", "RoverEscalonEvalCfg")):
    gym.register(id=_id, entry_point="isaaclab.envs:ManagerBasedRLEnv", disable_env_checker=True,
        kwargs={"env_cfg_entry_point": f"{__name__}.rover_eval_extra_cfg:{_cls}",
                "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RoverPPORunnerCfg"})

for _id, _cls in (
    ("Isaac-Rover-Robert-Denso-v1", "RoverDensoV1EnvCfg"),
    ("Isaac-Rover-Robert-Denso-v1-Eval", "RoverDensoV1EvalCfg"),
    ("Isaac-Rover-Robert-Llano-v1-Eval", "RoverLlanoV1EvalCfg"),
    ("Isaac-Rover-Robert-Escalon-v1-Eval", "RoverEscalonV1EvalCfg"),
    ("Isaac-Rover-Robert-Pendiente-v1-Robot-Eval", "RoverPendienteV1RobotEvalCfg"),
):
    gym.register(id=_id, entry_point="isaaclab.envs:ManagerBasedRLEnv", disable_env_checker=True,
        kwargs={"env_cfg_entry_point": f"{__name__}.rover_robot_v1_cfg:{_cls}",
                "rsl_rl_cfg_entry_point": f"{__name__}.rover_robot_v1_cfg:RoverPPORunnerRobotCfg"})
```

---

## 6. `scripts/referencia_reproducible.py`

```python
"""Lista ordenada de observaciones y acciones, recortes efectivos, semilla y material, por tarea."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="Isaac-Rover-Robert-Denso-v0")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
launcher = AppLauncher(args)  # guardar la referencia: si se pierde, el proceso muere al crear la escena
app = launcher.app

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry

cfg = parse_env_cfg(args.task, device=args.device, num_envs=4)
agent_cfg = load_cfg_from_registry(args.task, "rsl_rl_cfg_entry_point")
env = gym.make(args.task, cfg=cfg)
base = env.unwrapped

out = open(f"/tmp/referencia_{args.task}.txt", "w")  # app.close() pierde stdout
p = lambda *a: print(*a, file=out)

p(f"== {args.task}")
p("seed env:", cfg.seed, " seed agent:", getattr(agent_cfg, "seed", None))
p("episode_length_s:", cfg.episode_length_s, " dt:", cfg.sim.dt, " decimation:", cfg.decimation)
p("runner clip_actions:", getattr(agent_cfg, "clip_actions", None))
p("terreno physics_material:", cfg.scene.terrain.physics_material)
p("robot init pos:", cfg.scene.robot.init_state.pos)

p("\n== OBSERVACIONES (orden real de concatenacion)")
om = base.observation_manager
for grupo, nombres in om.active_terms.items():
    i = 0
    for nombre, dim in zip(nombres, om.group_obs_term_dim[grupo]):
        n = int(torch.tensor(dim).prod())
        p(f"[{grupo}] {i:4d}..{i + n - 1:4d}  {nombre:22s} {tuple(dim)}")
        i += n
    p(f"[{grupo}] total {i}")

p("\n== ACCIONES (orden real)")
am = base.action_manager
i = 0
for nombre in am.active_terms:
    t = am.get_term(nombre)
    p(f"{i:2d}..{i + t.action_dim - 1:2d}  {nombre}")
    p("     joints:", getattr(t, "_joint_names", None))
    for attr in ("_scale", "_offset", "_clip"):
        v = getattr(t, attr, None)
        if torch.is_tensor(v):
            v = v[0].tolist() if v.dim() > 1 else v.tolist()
        p(f"     {attr}:", v)
    i += t.action_dim
p("total acciones:", i)

p("\n== JOINTS DE LA ARTICULACION (orden)")
robot = base.scene["robot"]
for j, n in enumerate(robot.joint_names):
    p(f"{j:2d} {n}")

p("\n== ACTUADORES")
for nombre, act in robot.actuators.items():
    p(f"{nombre}: joints={act.joint_names}")
    for attr in ("effort_limit_sim", "velocity_limit_sim", "stiffness", "damping"):
        v = getattr(act, attr, None)
        if torch.is_tensor(v):
            v = v[0].tolist()
        p(f"     {attr}: {v}")

out.close()
env.close()
app.close()
```

---

## 7. `scripts/evaluar_rover.py`

```python
"""Evaluacion determinista con instrumentacion de actuadores (puntos 3 y 4 del plan).

Ejecuta un checkpoint con la politica determinista (media, sin ruido), semilla fija y N episodios
completos por entorno. Guarda en <salida>/<etiqueta>/:
  resumen.json / resumen.txt   agregados de todos los entornos y pasos
  series.npz                   series temporales por paso de los primeros --registrar_envs entornos

Pares: `estimado_*` viene del modelo de actuador implicito (data.applied_torque, tras recorte;
data.computed_torque, antes). `medido_*` es el modulo del par del wrench de reaccion que devuelve
el solver (data.body_incoming_joint_wrench_b): incluye componentes fuera del eje, asi que es una
COTA SUPERIOR del par motor, no el par motor.
"""

import argparse
import importlib.metadata
import json
import os
import sys
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="Isaac-Rover-Robert-Pendiente-v1-Eval")
parser.add_argument("--num_envs", type=int, default=256)
parser.add_argument("--episodios", type=int, default=2, help="episodios completos por entorno")
parser.add_argument("--load_run", type=str, required=True)
parser.add_argument("--checkpoint", type=str, default=None)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--salida", type=str, default=os.path.expanduser("~/robert_eval"))
parser.add_argument("--etiqueta", type=str, default="")
parser.add_argument("--registrar_envs", type=int, default=8)
parser.add_argument("--max_pasos", type=int, default=200000, help="freno de seguridad")
parser.add_argument("--nivel", type=int, default=None, help="nivel de terreno fijo para todos los entornos (terrenos con curriculum)")
AppLauncher.add_app_launcher_args(parser)
args, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args  # Hydra solo ve lo que no es nuestro
launcher = AppLauncher(args)
app = launcher.app

import gymnasium as gym
import numpy as np
import torch
from packaging import version

import isaaclab_tasks  # noqa: F401
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
from isaaclab_tasks.utils.hydra import hydra_task_config
from isaaclab_tasks.utils.parse_cfg import get_checkpoint_path
from rsl_rl.runners import OnPolicyRunner

VERSION_RSL = importlib.metadata.version("rsl-rl-lib")


@hydra_task_config(args.task, "rsl_rl_cfg_entry_point")
def main(env_cfg, agent_cfg: RslRlOnPolicyRunnerCfg):
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.sim.device = args.device
    env_cfg.seed = args.seed
    agent_cfg.seed = args.seed
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, VERSION_RSL)  # policy -> actor/critic (rsl_rl >= 4)
    torch.manual_seed(args.seed)

    log_root = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))  # absoluta, si no duplica la ruta
    ruta_ckpt = get_checkpoint_path(log_root, args.load_run, args.checkpoint or agent_cfg.load_checkpoint)

    env = gym.make(args.task, cfg=env_cfg)
    base = env.unwrapped
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(ruta_ckpt)
    policy = runner.get_inference_policy(device=base.device)
    con_reset = version.parse(VERSION_RSL) >= version.parse("4.0.0")

    # nivel de terreno fijo: max_init_terrain_level sortea entre 0 y N, aqui se fija N para todos
    if args.nivel is not None:
        terrain = base.scene.terrain
        terrain.terrain_levels[:] = args.nivel
        terrain.env_origins[:] = terrain.terrain_origins[terrain.terrain_levels, terrain.terrain_types]
        env.reset()
        cm = base.curriculum_manager
        for nombre, cfg in zip(cm._term_names, cm._term_cfgs):
            if hasattr(cfg.func, "reiniciar"):
                cfg.func.reiniciar()  # ese reset no es un episodio

    etiqueta = args.etiqueta or f"{args.task}__{os.path.basename(os.path.dirname(ruta_ckpt))}__{os.path.basename(ruta_ckpt)}"
    carpeta = os.path.join(args.salida, etiqueta)
    os.makedirs(carpeta, exist_ok=True)

    robot = base.scene["robot"]
    ids_llanta, nombres_llanta = robot.find_joints(["llanta.*"])
    ids_reductor, nombres_reductor = robot.find_joints(["reductor.*"])
    ids_cuerpo_llanta, nombres_cuerpo_llanta = robot.find_bodies(["llanta.*"])
    try:
        ids_cuerpo_reductor, nombres_cuerpo_reductor = robot.find_bodies(["alma.*"])
    except Exception:  # noqa: BLE001
        ids_cuerpo_reductor, nombres_cuerpo_reductor = [], []
    lim_llanta = robot.data.joint_effort_limits[0, ids_llanta]
    lim_reductor = robot.data.joint_effort_limits[0, ids_reductor]
    t_traccion = base.action_manager.get_term("traccion")
    t_direccion = base.action_manager.get_term("direccion")
    dim_tr = t_traccion.action_dim
    terminos_fin = list(base.termination_manager.active_terms)
    n = base.num_envs
    R = min(args.registrar_envs, n)
    dt = base.step_dt

    acc = {k: 0.0 for k in (
        "traccion_fuera_pm1", "direccion_fuera_pm1", "traccion_cerca_limite", "direccion_cerca_limite",
        "traccion_recortado", "direccion_recortado", "traccion_par_abs", "direccion_par_abs",
        "traccion_par_cuad", "direccion_par_cuad", "traccion_medido_abs", "direccion_medido_abs",
        "err_vx_abs", "err_wz_abs", "cmd_activo_pasos", "vel_rueda_obj_abs", "vel_rueda_real_abs")}
    acc["pasos"] = 0
    fin = {k: 0 for k in terminos_fin}
    episodios_env = torch.zeros(n, dtype=torch.long, device=base.device)
    claves = ["raw", "traccion_obj", "direccion_obj", "vel_rueda", "vel_rueda_obj", "ang_red", "ang_red_obj",
              "par_llanta_est", "par_llanta_calc", "par_red_est", "par_red_calc", "par_llanta_med", "par_red_med",
              "cmd", "vel_base", "angvel_base", "done"]
    series = {k: [] for k in claves}

    obs = env.get_observations()
    t0 = time.time()
    paso = 0
    while paso < args.max_pasos:
        with torch.inference_mode():
            acciones = policy(obs)
            obs, _, dones, _ = env.step(acciones)
            if con_reset:
                policy.reset(dones)
        paso += 1
        d = robot.data
        raw = base.action_manager.action
        par_est_ll = d.applied_torque[:, ids_llanta]
        par_calc_ll = d.computed_torque[:, ids_llanta]
        par_est_red = d.applied_torque[:, ids_reductor]
        par_calc_red = d.computed_torque[:, ids_reductor]
        w = d.body_incoming_joint_wrench_b
        med_ll = torch.norm(w[:, ids_cuerpo_llanta, 3:6], dim=-1)
        med_red = torch.norm(w[:, ids_cuerpo_reductor, 3:6], dim=-1) if len(ids_cuerpo_reductor) else torch.zeros_like(par_est_red)
        cmd = base.command_manager.get_command("base_velocity")
        vb = d.root_lin_vel_b
        wb = d.root_ang_vel_b

        acc["pasos"] += n
        acc["traccion_fuera_pm1"] += (raw[:, :dim_tr].abs() > 1.0).float().sum().item() / dim_tr
        acc["direccion_fuera_pm1"] += (raw[:, dim_tr:].abs() > 1.0).float().sum().item() / (raw.shape[1] - dim_tr)
        acc["traccion_cerca_limite"] += (par_est_ll.abs() >= 0.95 * lim_llanta).float().mean(dim=1).sum().item()
        acc["direccion_cerca_limite"] += (par_est_red.abs() >= 0.95 * lim_reductor).float().mean(dim=1).sum().item()
        acc["traccion_recortado"] += (par_calc_ll.abs() > lim_llanta).float().mean(dim=1).sum().item()
        acc["direccion_recortado"] += (par_calc_red.abs() > lim_reductor).float().mean(dim=1).sum().item()
        acc["traccion_par_abs"] += par_est_ll.abs().mean(dim=1).sum().item()
        acc["direccion_par_abs"] += par_est_red.abs().mean(dim=1).sum().item()
        acc["traccion_par_cuad"] += (par_est_ll ** 2).sum().item()
        acc["direccion_par_cuad"] += (par_est_red ** 2).sum().item()
        acc["traccion_medido_abs"] += med_ll.mean(dim=1).sum().item()
        acc["direccion_medido_abs"] += med_red.mean(dim=1).sum().item()
        activo = cmd[:, 0].abs() > 0.03
        acc["cmd_activo_pasos"] += activo.float().sum().item()
        acc["err_vx_abs"] += ((cmd[:, 0] - vb[:, 0]).abs() * activo).sum().item()
        acc["err_wz_abs"] += (cmd[:, 2] - wb[:, 2]).abs().sum().item()
        acc["vel_rueda_obj_abs"] += d.joint_vel_target[:, ids_llanta].abs().mean(dim=1).sum().item()
        acc["vel_rueda_real_abs"] += d.joint_vel[:, ids_llanta].abs().mean(dim=1).sum().item()
        for k in terminos_fin:
            fin[k] += int(base.termination_manager.get_term(k).sum().item())

        for k, v in (("raw", raw), ("traccion_obj", t_traccion.processed_actions), ("direccion_obj", t_direccion.processed_actions),
                     ("vel_rueda", d.joint_vel[:, ids_llanta]), ("vel_rueda_obj", d.joint_vel_target[:, ids_llanta]),
                     ("ang_red", d.joint_pos[:, ids_reductor]), ("ang_red_obj", d.joint_pos_target[:, ids_reductor]),
                     ("par_llanta_est", par_est_ll), ("par_llanta_calc", par_calc_ll), ("par_red_est", par_est_red),
                     ("par_red_calc", par_calc_red), ("par_llanta_med", med_ll), ("par_red_med", med_red),
                     ("cmd", cmd), ("vel_base", vb), ("angvel_base", wb), ("done", dones)):
            series[k].append(v[:R].cpu().numpy())

        episodios_env += dones.long()
        if bool((episodios_env >= args.episodios).all()):
            break

    P = acc["pasos"]
    par_total = max(acc["traccion_par_cuad"] + acc["direccion_par_cuad"], 1e-9)
    resumen = {
        "task": args.task, "checkpoint": ruta_ckpt, "seed": args.seed, "num_envs": n, "rsl_rl": VERSION_RSL, "nivel": args.nivel,
        "episodios_por_env": args.episodios, "pasos_totales_env": P, "segundos_sim": P / n * dt,
        "traccion_frac_fuera_pm1": acc["traccion_fuera_pm1"] / P,
        "direccion_frac_fuera_pm1": acc["direccion_fuera_pm1"] / P,
        "traccion_frac_tiempo_cerca_limite_par": acc["traccion_cerca_limite"] / P,
        "direccion_frac_tiempo_cerca_limite_par": acc["direccion_cerca_limite"] / P,
        "traccion_frac_tiempo_par_recortado": acc["traccion_recortado"] / P,
        "direccion_frac_tiempo_par_recortado": acc["direccion_recortado"] / P,
        "traccion_par_estimado_abs_medio_Nm": acc["traccion_par_abs"] / P,
        "direccion_par_estimado_abs_medio_Nm": acc["direccion_par_abs"] / P,
        "traccion_par_medido_abs_medio_Nm_COTA": acc["traccion_medido_abs"] / P,
        "direccion_par_medido_abs_medio_Nm_COTA": acc["direccion_medido_abs"] / P,
        "penaliza_par_fraccion_traccion": acc["traccion_par_cuad"] / par_total,
        "penaliza_par_fraccion_direccion": acc["direccion_par_cuad"] / par_total,
        "err_vx_abs_medio_con_cmd_activo": acc["err_vx_abs"] / max(acc["cmd_activo_pasos"], 1),
        "err_wz_abs_medio": acc["err_wz_abs"] / P,
        "vel_rueda_objetivo_abs_media": acc["vel_rueda_obj_abs"] / P,
        "vel_rueda_real_abs_media": acc["vel_rueda_real_abs"] / P,
        "terminaciones": fin,
        "limite_par_llanta": lim_llanta[0].item(), "limite_par_reductor": lim_reductor[0].item(),
        "joints_llanta": nombres_llanta, "joints_reductor": nombres_reductor,
        "cuerpos_llanta": nombres_cuerpo_llanta, "cuerpos_reductor": nombres_cuerpo_reductor,
        "tiempo_real_s": time.time() - t0,
    }
    try:
        cm = base.curriculum_manager
        resumen["metricas"] = dict(zip(cm._term_names, cm._term_cfgs))["metricas"].func.resumen()
    except Exception as e:  # noqa: BLE001
        resumen["metricas"] = f"no disponibles: {e}"

    with open(os.path.join(carpeta, "resumen.json"), "w") as f:
        json.dump(resumen, f, indent=2, default=str)
    with open(os.path.join(carpeta, "resumen.txt"), "w") as f:
        for k, v in resumen.items():
            if isinstance(v, dict):
                f.write(f"{k}:\n")
                for kk, vv in v.items():
                    f.write(f"    {kk}: {vv}\n")
            else:
                f.write(f"{k}: {v}\n")
    np.savez_compressed(os.path.join(carpeta, "series.npz"), dt=dt, **{k: np.stack(v) for k, v in series.items()})
    env.close()


if __name__ == "__main__":
    main()
    app.close()
```

Notas de version (rsl_rl 5.0.1 / isaaclab_rl 0.5.2): sin `handle_deprecated_rsl_rl_cfg` el runner falla con `KeyError: 'class_name'`; `policy.reset(dones)` es obligatorio desde rsl_rl 4; `env.step` devuelve `(obs, rew, dones, extras)`; `get_checkpoint_path` necesita la ruta absoluta.

---

## 8. `scripts/tabla_eval.py` y `scripts/resumen_tb.py`

```python
"""tabla_eval.py: junta los resumen.json de ~/robert_eval en una tabla (no necesita Isaac)."""
import glob
import json
import os
import sys

raiz = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/robert_eval")
for ruta in sorted(glob.glob(os.path.join(raiz, "*", "resumen.json"))):
    r = json.load(open(ruta))
    et = os.path.basename(os.path.dirname(ruta))
    print(f"{et:22s} nivel={r.get('nivel')}  tracc>1: {r['traccion_frac_fuera_pm1']:.2f}  dir>1: {r['direccion_frac_fuera_pm1']:.2f}  "
          f"par_rec tr/dir: {r['traccion_frac_tiempo_par_recortado']:.2f}/{r['direccion_frac_tiempo_par_recortado']:.2f}  "
          f"vrueda_obj: {r['vel_rueda_objetivo_abs_media']:.2f}  err_vx: {r['err_vx_abs_medio_con_cmd_activo']:.3f}  err_wz: {r['err_wz_abs_medio']:.3f}  "
          f"fin: {r['terminaciones']}")
    m = r.get("metricas")
    if isinstance(m, dict):
        for grupo, v in m.items():
            ex, n = v.get("exitos", 0), v.get("intentos", 0)
            extra = "  ".join(f"{k}={val:.2f}" if isinstance(val, float) else f"{k}={val}" for k, val in v.items() if k not in ("exitos", "intentos"))
            print(f"    {grupo:12s} {ex:4d}/{n:<4d} ({100 * ex / max(n, 1):5.1f} %)  {extra}")
    print()
```

```python
"""resumen_tb.py: valor final, maximo y su iteracion de las curvas de un run (no necesita Isaac)."""
import glob
import os
import sys

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

run = sys.argv[1]
ev = glob.glob(os.path.join(run, "events.out.tfevents.*"))
assert ev, f"sin eventos en {run}"
acc = EventAccumulator(ev[0], size_guidance={"scalars": 0})
acc.Reload()
claves = [t for t in acc.Tags()["scalars"] if t.startswith(("Train/", "Episode_Reward/", "Episode_Termination/", "Curriculum/", "Policy/", "Loss/"))]
print(f"{'tag':48s} {'ultimo':>10s} {'max':>10s} {'iter_max':>9s} {'iters':>7s}")
for t in sorted(claves):
    s = acc.Scalars(t)
    vals = [x.value for x in s]
    imax = max(range(len(vals)), key=lambda i: vals[i])
    print(f"{t:48s} {vals[-1]:10.4f} {vals[imax]:10.4f} {s[imax].step:9d} {len(vals):7d}")
```

---

## 9. Comandos

Referencia efectiva de una tarea:

```bash
./isaaclab.sh -p scripts/referencia_reproducible.py --task Isaac-Rover-Robert-Denso-v1 --headless; cat /tmp/referencia_Isaac-Rover-Robert-Denso-v1.txt
```

Evaluacion (siempre `sleep 8` entre instancias de Kit; sin eso, segfault de arranque):

```bash
# denso, nivel fijo, variante A
./isaaclab.sh -p scripts/evaluar_rover.py --task Isaac-Rover-Robert-Denso-Eval-A --num_envs 240 --episodios 2 --nivel 12 --load_run '.*_denso1' --etiqueta denso1_A_n12 --headless
# pendientes, 100 por rampa
./isaaclab.sh -p scripts/evaluar_rover.py --task Isaac-Rover-Robert-Pendiente-v1-Eval --num_envs 400 --episodios 2 --load_run '.*_pendiente2' --checkpoint model_11000.pt --etiqueta pend_p2_11000 --headless
# llano (100 por guion) y escalon (100 por altura)
./isaaclab.sh -p scripts/evaluar_rover.py --task Isaac-Rover-Robert-Llano-Eval --num_envs 500 --episodios 1 --load_run '.*_denso1' --etiqueta denso1_Llano --headless
./isaaclab.sh -p scripts/evaluar_rover.py --task Isaac-Rover-Robert-Escalon-Eval --num_envs 300 --episodios 1 --load_run '.*_denso1' --etiqueta denso1_Escalon --headless
# politica nueva: tareas *-v1-Eval y --load_run '.*_robot_v1' (busca en logs/rsl_rl/rover_robert_v1)
python3 scripts/tabla_eval.py
```

Entrenamiento de `robot_v1` y resumen de sus curvas:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Rover-Robert-Denso-v1 --headless --num_envs 6144 --run_name robot_v1 --max_iterations 10000
./isaaclab.sh -p scripts/resumen_tb.py logs/rsl_rl/rover_robert_v1/*_robot_v1/
./isaaclab.sh -p -m tensorboard.main --logdir logs/rsl_rl --port 6006   # un nivel arriba: ve rover_robert y rover_robert_v1
```
