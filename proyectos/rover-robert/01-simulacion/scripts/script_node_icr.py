"""Codigo del Script Node del Action Graph: control de direccion por ICR.

Este archivo NO se ejecuta en el Script Editor. Su contenido se pega en el
campo "script" del nodo Script Node del Action Graph.

Puertos que hay que declarar en el nodo antes de pegar el codigo:
  linear     double    Input
  angular    double    Input
  wheelVel   double[]  Output   -> ArtCtrl_Traccion.Velocity Command
  steerPos   double[]  Output   -> ArtCtrl_Direccion.Position Command

Orden de las ruedas en P, wheelVel y en Joint Names de ArtCtrl_Traccion:
  0 izq delantera, 1 izq media, 2 izq trasera,
  3 der delantera, 4 der media, 5 der trasera
  -> llanta_izq_d, llanta_izq_m, llanta_izq_d_, llanta_der_d_, llanta_der_m_, llanta_der_t

Orden en steerPos y en Joint Names de ArtCtrl_Direccion (solo esquinas):
  -> reductor_izq_d, reductor_izq_t, reductor_der_d, reductor_der_t_

SIGNO_DIR y SIGNO_VEL se calibran joint a joint (ver README, Parte 9.5).
Los valores de abajo son los de este modelo; verificar en cada importacion nueva.
"""

import math


def compute(db):
    R = 0.0815
    V_MAX = 0.167
    v = db.inputs.linear
    w = db.inputs.angular

    if v > V_MAX:
        v = V_MAX
    if v < -V_MAX:
        v = -V_MAX

    P = [(0.5714, 0.1939), (0.2415, 0.3330), (-0.0910, 0.1939),
         (0.5714, -0.1939), (0.2415, -0.3330), (-0.0910, -0.1939)]

    SIGNO_DIR = [1.0, 1.0, -1.0, -1.0, -1.0, -1.0]
    SIGNO_VEL = [1.0, 1.0, 1.0, 1.0, -1.0, 1.0]

    vel = []
    ang = []
    for i in range(6):
        x = P[i][0]
        y = P[i][1]
        vx = v - w * y
        vy = w * x
        s = math.sqrt(vx * vx + vy * vy) / R
        a = math.atan2(vy, vx)
        # Si el angulo pedido supera 90 grados, invertir el sentido de giro
        # en vez de dar media vuelta con la rueda
        if a > math.pi / 2:
            a = a - math.pi
            s = -s
        elif a < -math.pi / 2:
            a = a + math.pi
            s = -s
        vel.append(s * SIGNO_VEL[i])
        ang.append(a * SIGNO_DIR[i])

    db.outputs.wheelVel = vel
    db.outputs.steerPos = [ang[0], ang[2], ang[3], ang[5]]
    return True
