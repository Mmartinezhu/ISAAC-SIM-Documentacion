"""Mide la posicion de cada rueda respecto a base_link y el radio de rueda.

Ejecutar en el Script Editor de Isaac Sim con la simulacion en Stop y sin haber
dado Play antes (para leer la pose de diseño, no una postura caida).

Los valores (x, y) son los que van en la tabla P del Script Node de direccion.
Comprobar que los pares izquierda/derecha salen simetricos; si no, el frame
base_link esta descentrado y hay que restar el offset (en este rover, 1.85 cm en Y).
"""

from pxr import Usd, UsdGeom, UsdPhysics

stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache(Usd.TimeCode.Default())
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])

base = stage.GetPrimAtPath("/World/cuerpo_suspension/base_link")
inv = xc.GetLocalToWorldTransform(base).GetInverse()

for prim in stage.Traverse():
    if prim.GetName().startswith("llanta") and prim.HasAPI(UsdPhysics.RigidBodyAPI):
        pos = inv.Transform(xc.GetLocalToWorldTransform(prim).ExtractTranslation())
        tam = bc.ComputeWorldBound(prim).ComputeAlignedRange().GetSize()
        radio = max(tam[0], tam[1], tam[2]) / 2.0
        print(
            prim.GetName().ljust(18)
            + " x=" + str(round(pos[0], 4)).ljust(9)
            + " y=" + str(round(pos[1], 4)).ljust(9)
            + " radio=" + str(round(radio, 4))
        )

print("")
print("=== LIMITES DE LOS REDUCTORES ===")
for prim in stage.Traverse():
    if prim.GetName().startswith("reductor"):
        lo = prim.GetAttribute("physics:lowerLimit")
        hi = prim.GetAttribute("physics:upperLimit")
        print(prim.GetName().ljust(18) + " " + str(lo.Get() if lo else "sin limite") + " .. " + str(hi.Get() if hi else "sin limite"))
