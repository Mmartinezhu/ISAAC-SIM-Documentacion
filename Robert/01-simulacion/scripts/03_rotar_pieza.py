"""Gira 180 grados una pieza que llego de Fusion mirando al lado contrario.

Ejecutar en el Script Editor de Isaac Sim con la simulacion en Stop.

Se rota el frame del joint del lado hijo (localRot1), no el prim de la pieza:
asi la nueva orientacion pasa a ser el cero del joint y la fisica no la deshace.

Uso: ajustar JOINT y EJE, ejecutar, dar Play para ver el resultado.
Si el eje no era el correcto, ejecutar de nuevo con el mismo eje (deshace el giro)
y probar con otro.

Ejes:
  Gf.Vec3f(1, 0, 0)  X, longitudinal
  Gf.Vec3f(0, 1, 0)  Y, lateral
  Gf.Vec3f(0, 0, 1)  Z, vertical  (el habitual cuando la pieza mira hacia adentro)
"""

from pxr import Gf

stage = omni.usd.get_context().get_stage()

JOINT = "/World/cuerpo_suspension/joints/reductor_izq_d"
EJE = Gf.Vec3f(0.0, 0.0, 1.0)

p = stage.GetPrimAtPath(JOINT)
a = p.GetAttribute("physics:localRot1")
# Un cuaternion de 180 grados tiene w=0 y el eje en la parte imaginaria
a.Set(a.Get() * Gf.Quatf(0.0, EJE))
print("rotado 180 grados: " + JOINT)
print("el joint se vera en rojo hasta dar Play; es normal")
