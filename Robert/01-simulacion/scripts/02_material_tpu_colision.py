"""Material TPU y colision SDF en las ruedas, material Terreno para el suelo.

Ejecutar en el Script Editor de Isaac Sim con la simulacion en Stop.

Hace, en orden:
  1. Desinstancia las colisiones de las llantas (si no, son de solo lectura).
  2. Crea el material de fisica TPU y lo asigna a las colisiones de las llantas.
  3. Cambia la aproximacion de colision de las llantas a SDF, que conserva los tacos.
  4. Crea el material Terreno y lo asigna al suelo y al cubo si existen.
"""

from pxr import Sdf, UsdGeom, UsdPhysics, UsdShade

stage = omni.usd.get_context().get_stage()

n = 0
for p in stage.Traverse():
    path = str(p.GetPath())
    if p.IsInstanceable() and "llanta" in path and path.endswith("/collisions"):
        p.SetInstanceable(False)
        n += 1
print("colisiones de llanta desinstanciadas: " + str(n) + " (deben ser 6)")

stage.DefinePrim("/World/PhysicsMaterials", "Scope")


def crear_material(nombre, estatica, dinamica, restitucion):
    mat = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/" + nombre)
    prim = mat.GetPrim()
    api = UsdPhysics.MaterialAPI.Apply(prim)
    api.CreateStaticFrictionAttr().Set(estatica)
    api.CreateDynamicFrictionAttr().Set(dinamica)
    api.CreateRestitutionAttr().Set(restitucion)
    prim.CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("multiply")
    prim.CreateAttribute("physxMaterial:improvePatchFriction", Sdf.ValueTypeNames.Bool).Set(True)
    return mat


tpu = crear_material("TPU", 0.9, 0.8, 0.3)
terreno = crear_material("Terreno", 0.8, 0.7, 0.1)


def asignar(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")


n = 0
for p in stage.Traverse():
    path = str(p.GetPath())
    if p.IsA(UsdGeom.Mesh) and "llanta" in path and "/collisions" in path:
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set("sdf")
        p.CreateAttribute("physxSDFMeshCollision:sdfResolution", Sdf.ValueTypeNames.Int).Set(256)
        asignar(p, tpu)
        n += 1
print("llantas con TPU y colision SDF: " + str(n) + " (deben ser 6)")

n = 0
for p in stage.Traverse():
    path = str(p.GetPath())
    if p.HasAPI(UsdPhysics.CollisionAPI) and ("GroundPlane" in path or "Cube" in path or "Rampa" in path):
        asignar(p, terreno)
        n += 1
print("superficies con material Terreno: " + str(n))
