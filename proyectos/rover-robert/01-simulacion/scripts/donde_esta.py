"""Dice en que tramo de la rampa de anillos esta el rover.

Ejecutar en el Script Editor de Isaac Sim con la simulacion en Play.
Los parametros deben coincidir con los de rampa_anillos.py.

Si los numeros no cambian entre ejecuciones mientras el rover se mueve, la
fisica esta escribiendo en Fabric y no en USD: en Physics > Settings cambiar
Simulation Output a USD.
"""

import math

import omni.usd
from pxr import Usd, UsdGeom

ANGULO_INI = 5.0
ANGULO_FIN = 35.0
NUM_ANILLOS = 7
ANCHO = 1.0
PLANO = 1.0
RADIO_PLAT = 2.0

paso = ANCHO + PLANO
stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache(Usd.TimeCode.Default())
pos = xc.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/cuerpo_suspension/base_link")).ExtractTranslation()
r = math.hypot(pos[0], pos[1])

if r < RADIO_PLAT:
    print("en la plataforma  |  r=" + str(round(r, 2)) + " m")
else:
    k = int((r - RADIO_PLAT) / paso)
    dentro = (r - RADIO_PLAT) % paso
    ang = ANGULO_INI + k * (ANGULO_FIN - ANGULO_INI) / max(NUM_ANILLOS - 1, 1)
    if k >= NUM_ANILLOS:
        print("COMPLETO la rampa  |  z=" + str(round(pos[2], 2)) + " m")
    elif dentro < ANCHO:
        print("SUBIENDO tramo " + str(k) + " (" + str(round(ang, 1)) + " grados)  |  r=" + str(round(r, 2)) + " m  z=" + str(round(pos[2], 2)) + " m")
    else:
        print("en el LLANO tras el tramo " + str(k) + " (" + str(round(ang, 1)) + " grados superado)  |  r=" + str(round(r, 2)) + " m  z=" + str(round(pos[2], 2)) + " m")
