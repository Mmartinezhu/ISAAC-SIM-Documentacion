"""Muestra el estado real de joints, masas y articulacion.

Ejecutar en el Script Editor de Isaac Sim. Funciona en Stop.

Imprime para cada joint: nombre exacto (con repr, para ver guiones bajos),
tipo, stiffness, damping, maxForce, si esta excluido de la articulacion y
sus limites. Despues las masas de cada cuerpo y donde esta el Articulation Root.

Valores esperados tras 01_configurar_joints.py:
  llanta_*        stiff=0      damp=2.16   force=4.41
  reductor_*      stiff=100000 damp=10000  lim=-90..90
  suspension      stiff=0      damp=3
  hombro_*/codo_* stiff=0      damp=5
  union_sus_*     stiff=0      damp=0      excl=True solo en los dos _01
"""

from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()

print("")
print("=== JOINTS ===")
for prim in stage.Traverse():
    if not prim.IsA(UsdPhysics.Joint):
        continue
    d = UsdPhysics.DriveAPI.Get(prim, "angular")
    s = dm = fm = "-"
    if d and d.GetStiffnessAttr():
        s = str(round(d.GetStiffnessAttr().Get(), 2))
        dm = str(round(d.GetDampingAttr().Get(), 2))
        f = d.GetMaxForceAttr()
        fm = str(round(f.Get(), 2)) if f and f.Get() is not None else "-"
    e = prim.GetAttribute("physics:excludeFromArticulation")
    lo = prim.GetAttribute("physics:lowerLimit")
    hi = prim.GetAttribute("physics:upperLimit")
    lim = "-"
    if lo and hi and lo.Get() is not None:
        lim = str(round(lo.Get(), 1)) + ".." + str(round(hi.Get(), 1))
    print(
        repr(prim.GetName()).ljust(22)
        + str(prim.GetTypeName()).ljust(22)
        + " stiff=" + s.ljust(9) + " damp=" + dm.ljust(9) + " force=" + fm.ljust(9)
        + " excl=" + str(e.Get() if e else False).ljust(6) + " lim=" + lim
    )

print("")
print("=== MASAS ===")
total = 0.0
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.RigidBodyAPI):
        m = UsdPhysics.MassAPI(p)
        a = m.GetMassAttr()
        if a and a.Get():
            total += a.Get()
            print(p.GetName().ljust(28) + str(round(a.Get(), 3)) + " kg")
print("TOTAL: " + str(round(total, 3)) + " kg")

print("")
print("=== ARTICULATION ROOT ===")
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(str(p.GetPath()))
