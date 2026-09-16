# ROBERT en Isaac Sim e Isaac Lab

Documentacion del flujo completo de ROBERT: desde el modelo en Fusion 360 hasta una politica de aprendizaje reforzado entrenada en Isaac Lab.

A continuacion se presentara el proceso realizado para el aprendizaje reforzado en isaac lab, desde el modelamiento del robot hasta el entrenamiento.

## Partes

1. [Simulacion: de Fusion 360 a un USD controlable por ROS2](01-simulacion/README.md)
2. [Reinforcement Learning en Isaac Lab](02-reinforcement-learning/README.md)

La parte 1 termina con dos archivos USD: uno con Action Graph para teleoperar por ROS2 y otro limpio para entrenar. La parte 2 parte del segundo.

## Resultados hasta la fecha (septiembre de 2026)

| Capacidad | Resultado en simulacion | Donde se mide |
| --- | --- | --- |
| Seguir comandos de velocidad y rumbo en terreno irregular | Nivel 12 de 20 en terreno denso (bloques y fosos de ±10 cm, huecos de 15 cm) | Parte 2, fase 4 |
| Bajar escalones | Hasta 25 cm | Parte 2, fase 1 |
| Subir escalones verticales | 7 a 9 cm; limite geometrico del rocker-bogie (la rueda media no pivota sobre el borde) | Parte 2, fase 2 |
| Subir pendientes | 22 a 24 grados con solvencia, 26 a medias, 30 excepcional (friccion 1.0) | Parte 2, fase 3 |

Politica candidata para el robot: `denso1`. Pendiente: robustez a friccion y masa, exportacion a ONNX y nodo ROS2.

## El robot

ROBERT es un rover de seis ruedas con suspension rocker-bogie, el mismo mecanismo de los rovers de exploracion de la NASA.

| Caracteristica | Valor |
| --- | --- |
| Masa total simulada | 16.27 kg |
| Ruedas | 6, diametro 16.3 cm, ancho 13.2 cm, impresas en TPU |
| Traccion | 6 motores, uno por rueda, 45 kg·cm (4.41 N·m) |
| Direccion | 4 reductores en las esquinas, sin limite mecanico (se limita por software a ±90°) |
| Velocidad maxima | 10 m/min = 0.167 m/s |
| Suspension | Rocker-bogie pasiva con diferencial central |
| Batalla (eje delantero a trasero) | 0.662 m |
| Ancho entre ruedas | 0.39 m en las esquinas, 0.67 m en las medias |

Las ruedas medias no tienen direccion. En un giro sobre el propio eje quedan sobre la linea del centro de rotacion, asi que apuntando rectas ya son tangentes a la circunferencia: no la necesitan. Esa es la razon por la que los rovers de la NASA usan exactamente esta configuracion.

## Cadena cinematica

```text
base_link (chasis)
|-- suspension        -> suspension_cuerpo_v1_1   (diferencial, pasivo)
|     |-- union_sus_der  -> union_der_sus_1       (esferico, pasivo)
|     `-- union_sus_izq  -> union_izq_sus_1       (esferico, pasivo)
|-- hombro_der        -> hombro_der_1             (pasivo)
|     |-- union_sus_der_01 -> union_der_sus_1     (esferico, CIERRA EL LAZO)
|     |-- reductor_der_d -> alma_der_d_1 -> llanta_der_d_ -> llanta_der_d_1
|     `-- codo_der     -> codo_der_1              (pasivo)
|           |-- llanta_der_m_ -> llanta_der_m_1
|           `-- reductor_der_t_ -> alma_der_t_1 -> llanta_der_t -> llanta_der_t_1
`-- hombro_izq        -> hombro_izq_1             (pasivo)
      |-- union_sus_izq_01 -> union_izq_sus_1     (esferico, CIERRA EL LAZO)
      |-- reductor_izq_d -> alma_izq_d__1 -> llanta_izq_d -> llanta_izq_d_1
      `-- codo_izq     -> codo_izq_1              (pasivo)
            |-- llanta_izq_m -> llanta_izq_m_1
            `-- reductor_izq_t -> alma_izq_t_1 -> llanta_izq_d_ -> llanta_izq_t_1
```

Cada `union_*_sus_1` es un tirante que conecta el diferencial central con un hombro. Como el hombro tambien cuelga del chasis, eso forma un **lazo cerrado** por lado. Ese lazo es lo que hace que el diferencial transmita el movimiento entre ambos lados: cuando un hombro sube, el otro baja. Sin el lazo el mecanismo deja de ser un rocker-bogie.

## Mapa de joints

Los nombres vienen de Fusion 360 y son inconsistentes: algunos llevan guion bajo al final y uno de los joints traseros se llama `_d` (de "delantero"). Esta tabla es la referencia definitiva; cualquier lista de nombres en un script o en un Action Graph debe salir de aqui.

| Posicion | Joint de la llanta | Joint del reductor | Cuerpo de la llanta |
| --- | --- | --- | --- |
| Izquierda delantera | `llanta_izq_d` | `reductor_izq_d` | `llanta_izq_d_1` |
| Izquierda media | `llanta_izq_m` | (no tiene) | `llanta_izq_m_1` |
| Izquierda trasera | `llanta_izq_d_` | `reductor_izq_t` | `llanta_izq_t_1` |
| Derecha delantera | `llanta_der_d_` | `reductor_der_d` | `llanta_der_d_1` |
| Derecha media | `llanta_der_m_` | (no tiene) | `llanta_der_m_1` |
| Derecha trasera | `llanta_der_t` | `reductor_der_t_` | `llanta_der_t_1` |

Joints pasivos de la suspension: `suspension`, `hombro_der`, `hombro_izq`, `codo_der`, `codo_izq`, y los cuatro esfericos `union_sus_der`, `union_sus_izq`, `union_sus_der_01`, `union_sus_izq_01`.

Regla practica: en scripts usar expresiones como `startswith("llanta")` o el regex `llanta.*` en vez de listas de nombres. Cuando haga falta el nombre exacto, verificarlo antes con `repr(prim.GetName())`, que hace visible el guion bajo.

## Posiciones de las ruedas

Medidas respecto al centro geometrico del rover, en metros. Se usan en el control de direccion y son las que hay que actualizar si cambia el diseño.

| Rueda | x | y |
| --- | --- | --- |
| Izquierda delantera | 0.5714 | +0.1939 |
| Izquierda media | 0.2415 | +0.3330 |
| Izquierda trasera | -0.0910 | +0.1939 |
| Derecha delantera | 0.5714 | -0.1939 |
| Derecha media | 0.2415 | -0.3330 |
| Derecha trasera | -0.0910 | -0.1939 |

Nota: el frame `base_link` que sale de Fusion esta descentrado 1.85 cm en Y respecto al centro real. Estas coordenadas ya lo compensan.

## Maquina de trabajo

| Elemento | Valor |
| --- | --- |
| Equipo | `talos@IsaacUN`, Ubuntu, RTX 5080 16 GB |
| Isaac Sim | 5.x |
| Isaac Lab | 2.x, en `~/Github/IsaacLab` |
| USD del robot para RL | `/home/talos/IsaacsimManuel/ROBERT/robert_RL.usd` |
| ROS2 | Se abre con `./terminal_b.bash` desde `~/isaac-sim` |

El trabajo se hizo por acceso remoto. Los scripts largos se pegan en la terminal con `cat > archivo << 'EOF'` en vez de descargarlos.
