"""Configura todos los joints del rover ROBERT de una vez.

Ejecutar en el Script Editor de Isaac Sim con la simulacion en Stop.
Sirve tanto para la configuracion inicial como para restaurarla si se pierde.

Hace, en orden:
  1. Revierte cualquier joint en purpose=guide a default.
  2. Excluye de la articulacion un joint por lazo (union_sus_der_01, union_sus_izq_01).
  3. Pone eje X en los joints esfericos.
  4. Libera los joints pasivos (stiffness, friccion y armature a 0) y les pone damping.
  5. Configura las ruedas como motor real (4.41 N.m, damping 2.16).
  6. Configura los reductores como drive de posicion y los limita a +-90 grados.

Despues de ejecutarlo, comprobar con 04_diagnostico_joints.py.
"""

from pxr import Sdf, UsdPhysics

stage = omni.usd.get_context().get_stage()

EXCLUIR = ["union_sus_der_01", "union_sus_izq_01"]
DAMPING = {"suspension": 3.0, "hombro_der": 5.0, "hombro_izq": 5.0, "codo_der": 5.0, "codo_izq": 5.0}
TOKENS = ["angular", "linear", "rotX", "rotY", "rotZ"]

# Motor de 45 kg.cm a 10 m/min con rueda de 0.0815 m
PAR_RUEDA = 4.41
DAMPING_RUEDA = 2.16

n_ruedas = 0
n_reductores = 0

for prim in stage.Traverse():
    if not prim.IsA(UsdPhysics.Joint):
        continue
    n = prim.GetName()

    p = prim.GetAttribute("purpose")
    if p and p.Get() == "guide":
        p.Set("default")
        print("guide -> default: " + n)

    if n in EXCLUIR:
        prim.CreateAttribute("physics:excludeFromArticulation", Sdf.ValueTypeNames.Bool).Set(True)
        print("excluido de la articulacion: " + n)

    if n.startswith("union_sus"):
        a = prim.GetAttribute("physics:axis")
        if a:
            a.Set("X")

    if n in DAMPING or n.startswith("union_sus"):
        for t in TOKENS:
            d = UsdPhysics.DriveAPI.Get(prim, t)
            if d and d.GetStiffnessAttr():
                d.GetStiffnessAttr().Set(0.0)
                d.GetDampingAttr().Set(0.0)
        f = prim.GetAttribute("physxJoint:jointFriction")
        if f:
            f.Set(0.0)
        ar = prim.GetAttribute("physxJoint:armature")
        if ar:
            ar.Set(0.0)

    if n in DAMPING:
        d = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not d:
            d = UsdPhysics.DriveAPI.Apply(prim, "angular")
        d.CreateStiffnessAttr().Set(0.0)
        d.CreateDampingAttr().Set(DAMPING[n])
        d.CreateTargetVelocityAttr().Set(0.0)
        print("pasivo " + n + " damping=" + str(DAMPING[n]))

    if n.startswith("llanta"):
        d = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not d:
            d = UsdPhysics.DriveAPI.Apply(prim, "angular")
        d.CreateStiffnessAttr().Set(0.0)
        d.CreateDampingAttr().Set(DAMPING_RUEDA)
        d.CreateMaxForceAttr().Set(PAR_RUEDA)
        n_ruedas += 1

    if n.startswith("reductor"):
        d = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not d:
            d = UsdPhysics.DriveAPI.Apply(prim, "angular")
        d.CreateStiffnessAttr().Set(100000.0)
        d.CreateDampingAttr().Set(10000.0)
        d.CreateMaxForceAttr().Set(10000.0)
        if prim.IsA(UsdPhysics.RevoluteJoint):
            j = UsdPhysics.RevoluteJoint(prim)
            j.CreateLowerLimitAttr().Set(-90.0)
            j.CreateUpperLimitAttr().Set(90.0)
        n_reductores += 1

print("ruedas configuradas: " + str(n_ruedas) + " (deben ser 6)")
print("reductores configurados: " + str(n_reductores) + " (deben ser 4)")
