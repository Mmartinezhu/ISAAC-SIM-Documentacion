"""Rampa de anillos concentricos con pendiente creciente, para medir la pendiente maxima.

Ejecutar en el Script Editor de Isaac Sim con la simulacion en Stop.

Crea /World/Rampa: plataforma central plana y, hacia fuera, anillos donde cada
uno es una rampa de ANCHO metros seguida de un tramo llano de PLANO metros.
La pendiente va de ANGULO_INI a ANGULO_FIN en NUM_ANILLOS pasos.

Colores: plataforma azul; rampas de verde (facil) a rojo (dificil); llanos con
el tono de la rampa que acaban de superar, apagado. Una luz lateral marca las
pendientes con sombreado.

Ademas baja el GroundPlane 20 cm (para que no coincida con la plataforma) y
coloca el rover en el centro. Despues: Play y publicar avance recto en /cmd_vel.
Para saber en que tramo esta el rover: donde_esta.py.
"""

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

stage = omni.usd.get_context().get_stage()

ANGULO_INI = 5.0
ANGULO_FIN = 35.0
NUM_ANILLOS = 7
ANCHO = 1.0
PLANO = 1.0
RADIO_PLAT = 2.0
RES = 0.1
MARGEN = 2.0

paso = ANCHO + PLANO
lado = 2.0 * (RADIO_PLAT + NUM_ANILLOS * paso + MARGEN)
n = int(round(lado / RES)) + 1
xs = np.linspace(-lado / 2.0, lado / 2.0, n)
X, Y = np.meshgrid(xs, xs, indexing="ij")
R = np.sqrt(X * X + Y * Y)

tang = np.tan(np.radians(np.linspace(ANGULO_INI, ANGULO_FIN, NUM_ANILLOS)))
verde = np.array([0.15, 0.80, 0.20])
rojo = np.array([0.95, 0.15, 0.10])

Z = np.zeros_like(R)
colores = np.zeros(R.shape + (3,), dtype=np.float32)
colores[R < RADIO_PLAT] = [0.30, 0.45, 0.80]

r0 = RADIO_PLAT
h0 = 0.0
col = verde
for k in range(NUM_ANILLOS):
    t = k / max(NUM_ANILLOS - 1, 1)
    col = verde * (1.0 - t) + rojo * t
    r1 = r0 + ANCHO
    m = (R >= r0) & (R < r1)
    Z[m] = h0 + (R[m] - r0) * tang[k]
    colores[m] = col
    h0 += ANCHO * tang[k]
    r0 = r1
    r1 = r0 + PLANO
    m = (R >= r0) & (R < r1)
    Z[m] = h0
    colores[m] = col * 0.45 + 0.25
    r0 = r1
Z[R >= r0] = h0
colores[R >= r0] = col * 0.45 + 0.25

verts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1).astype(np.float32)
i, j = np.meshgrid(np.arange(n - 1), np.arange(n - 1), indexing="ij")
a = (i * n + j).ravel()
b = a + 1
c = a + n
d = c + 1
faces = np.concatenate([np.stack([a, c, b], 1), np.stack([b, c, d], 1)], 0).astype(np.int32)

path = "/World/Rampa"
if stage.GetPrimAtPath(path):
    stage.RemovePrim(path)
mesh = UsdGeom.Mesh.Define(stage, path)
mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(verts))
mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces), 3, dtype=np.int32)))
mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.ravel()))
mesh.CreateSubdivisionSchemeAttr("none")
pv = mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex)
pv.Set(Vt.Vec3fArray.FromNumpy(colores.reshape(-1, 3)))

prim = mesh.GetPrim()
UsdPhysics.CollisionAPI.Apply(prim)
UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("none")

mat = stage.GetPrimAtPath("/World/PhysicsMaterials/Terreno")
if mat:
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(mat), UsdShade.Tokens.weakerThanDescendants, "physics")
    print("material Terreno aplicado")

luz_path = "/World/LuzRampa"
if stage.GetPrimAtPath(luz_path):
    stage.RemovePrim(luz_path)
luz = UsdLux.DistantLight.Define(stage, luz_path)
luz.CreateIntensityAttr(2500.0)
luz.CreateAngleAttr(1.0)
UsdGeom.Xformable(luz.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 35.0, 0.0))


def poner_en(prim_path, xyz):
    p = stage.GetPrimAtPath(prim_path)
    if not p:
        return
    xf = UsdGeom.Xformable(p)
    tr = None
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            tr = op
    if tr is None:
        tr = xf.AddTranslateOp()
    tr.Set(Gf.Vec3d(*xyz))


poner_en("/World/GroundPlane", (0.0, 0.0, -0.2))
poner_en("/World/cuerpo_suspension", (0.0, 0.0, 0.3))

print("rampa: " + str(NUM_ANILLOS) + " tramos, altura total " + str(round(h0, 2)) + " m, lado " + str(round(lado, 1)) + " m")
for k in range(NUM_ANILLOS):
    ang = ANGULO_INI + k * (ANGULO_FIN - ANGULO_INI) / max(NUM_ANILLOS - 1, 1)
    ini = RADIO_PLAT + k * paso
    print("  tramo " + str(k) + ": " + str(round(ang, 1)) + " grados, rampa en r=" + str(ini) + ".." + str(ini + ANCHO) + " m, llano hasta " + str(ini + paso) + " m")
