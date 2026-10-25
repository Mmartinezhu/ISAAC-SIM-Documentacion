# Parte 2b: de una politica que entrena a una politica que se puede usar

La [Parte 2](README.md) explica como montar la tarea, entrenar con PPO y leer TensorBoard. Esta guia cubre lo que viene despues: como medir una politica sin engañarse, por que hay que acotar las acciones, como repartir el trabajo entre un controlador clasico y la red, como hacer que el rover se pare, que limites reales tiene el motor y como sacar la politica del simulador.

Cada seccion es una regla practica con su razon. Los numeros son de ROBERT en simulacion y sirven como orden de magnitud para un rover parecido; ninguno se ha comprobado en el robot real.

## 1. Medir antes de decidir: la evaluacion fija

Las curvas de entrenamiento no sirven para elegir checkpoint. El curriculum mezcla niveles y tipos de terreno, la politica es estocastica y las metricas de curriculum suelen ser medias moviles de muestras pequeñas. Hace falta una evaluacion aparte, siempre igual.

### 1.1 Condiciones de una evaluacion valida

| Condicion | Por que |
| --- | --- |
| Acciones deterministas (la media de la gaussiana) | La politica estocastica vive de su ruido: con recortes de accion, una desviacion grande sale gratis y la entropia la empuja hacia arriba |
| Sin ruido en las observaciones | Si una funcion que reemplaza el grupo de observaciones vuelve a activar `enable_corruption`, hay que desactivarlo despues. El evaluador debe negarse a correr si encuentra ruido |
| Normalizacion congelada | Comprobar que las estadisticas del normalizador no cambian durante la evaluacion |
| Cuota exacta de episodios | Cada entorno aporta sus primeros N episodios y ninguno mas; los que terminan pronto no pueden aportar el doble |
| Captura antes del reset | Tras `env.step()` los entornos terminados ya estan reiniciados: su accion, comando y estado mezclan dos episodios. Excluir esas muestras |
| Posiciones y niveles fijos | El curriculum de entrenamiento decide donde nace cada rover; en evaluacion se fija |
| Misma semilla, y otras dos antes de decidir | Con fisica de GPU dos corridas identicas difieren en 2-3 intentos de 30. Diferencias menores no son señal |

### 1.2 Trampas del terreno

- **Con `curriculum=False` y varios sub-terrenos, el generador reparte los tipos al azar por parcela.** El mapeo columna → tipo deja de valer y las tasas por tipo salen mezcladas. Para evaluar por tipo, `curriculum=True` y nivel fijo escrito en `terrain_levels` y `env_origins`.
- **En un terreno de rampas concentricas el nivel no elige el angulo.** Si las rampas son anillos del mismo parche, el angulo lo decide el radio al que nace el rover. Hay que colocar cada entorno en su rampa por indice (`i % numero_de_rampas`) y comprobar la cuota por rampa.
- **`horizontal_scale` cuantiza.** Con 0.1 m, huecos pedidos de 15 cm salen de 10 o 20. Verificar las dimensiones reales del terreno generado.
- **El rover nace en el aire.** Con el spawn alto que exige el terreno, cae decimetros y tarda en tener las seis ruedas apoyadas. Cualquier medida que empiece en el paso 0 mide la caida: esperar a que se asiente (velocidades angulares por debajo de un umbral sostenido) antes de empezar a contar.

### 1.3 Metricas que dicen lo que parecen

- **Exito** = cruzar una linea de llegada y permanecer al otro lado 1 s, sin vuelco. Separar "llego", "llego y luego volco" y "completo sin fallo".
- **Atasco** independiente del contacto: comando activo y progreso menor que un umbral en una ventana (por ejemplo, menos de 0.10 m en 5 s). En rampa el progreso tiene que tener signo: un rover que resbala hacia atras no avanza.
- **Parada**: medir por separado ruedas detenidas, chasis detenido y ambas cosas, con cumplimiento continuo durante 1 s (no una media movil bajo el umbral). Si una condicion no se cumple nunca, informar "no alcanzado", no un tiempo.
- **Par**: el `applied_torque` del modelo de actuador no sabe que `velocity_limit_sim` ha capado la rueda; el wrench de reaccion de la articulacion es una cota superior, no el par del motor. No presentar ninguno de los dos como par medido.
- **Medias contra medias.** Comparar la media de un controlador con el maximo de otro da conclusiones falsas. Media y percentil 95, y el maximo solo como dato aparte.

### 1.4 Leer la configuracion efectiva

Cada run guarda `params/env.yaml` y `params/agent.yaml`. Es la unica fuente fiable de lo que se entreno. Un recorte de acciones "aplicado" que en `env.yaml` aparece como `clip: null` no existio, aunque el codigo que uno cree haber editado lo tenga. La causa tipica: una copia del archivo de configuracion fuera del paquete de tareas. Regla: ninguna copia de los `*_cfg.py` fuera del paquete, y un script que vuelque la configuracion efectiva (orden de observaciones y acciones, recortes, actuadores) antes de dar un run por bueno.

## 2. Acotar las acciones

La red sale de una gaussiana y produce valores fuera de [-1, 1] sin problema. Sin recorte:

- la traccion pide velocidades imposibles (decenas de rad/s para una rueda limitada a 2) y lo que hace en realidad es mandar par maximo todo el tiempo;
- la direccion pide angulos de cientos de grados y el servo empuja contra el tope del joint; casi toda la penalizacion de par se va en eso.

Recortar los objetivos ya escalados (no la salida cruda) elimina la saturacion. En terreno irregular, recortar una politica ya entrenada no le cuesta exitos medibles. En rampas empinadas si: una politica entrenada sin recorte pierde capacidad al recortarla, asi que el recorte tiene que estar desde el entrenamiento.

El recorte no cambia la red: la salida cruda puede seguir saturada (se han visto ±46 en un espacio donde ±1 es el tope). Lo que cambia es la cota que la contiene. Por eso el diseño del controlador importa mas que el recorte.

## 3. Direccion: limitar la rapidez y separar nominal y residual

### 3.1 Comprobar signos y orden sin RL

Antes de culpar a la politica, mover cada rueda y cada reductor a mano y anotar que signo produce avance fisico y que posicion es "rueda recta". En un modelo que viene de CAD lo normal es que algunos ejes esten invertidos. La comprobacion tiene que ser independiente de la suposicion: comparar el angulo medido con el objetivo enviado, pasando los dos por la misma constante de signo, no detecta una inversion global. Lo que si la detecta es medir la guiñada del eslabon de direccion en el mundo.

### 3.2 Limitador de rapidez del objetivo

Si el objetivo de direccion alterna entre extremos a la frecuencia de control, el angulo real hace un diente de sierra a la velocidad maxima del reductor y nunca llega a ningun sitio. Un limitador lo arregla sin tocar la red:

```text
delta_max = v_max_reductor * dt_control
aplicado  = anterior + clamp(solicitado - anterior, -delta_max, +delta_max)
```

- Una vez por paso de control, despues de escalar y recortar.
- En el reset se inicializa con la posicion real de los reductores.
- El objetivo aplicado entra en las observaciones del actor y del critico, para que la politica sepa donde esta su propio objetivo.
- Si la velocidad real del reductor no se conoce, la que se use es un supuesto de simulacion y hay que decirlo.

Con el limitador los reductores se mueven varias veces menos y el seguimiento mejora en terreno irregular y en rampa.

### 3.3 Nominal cinematico mas residual acotado

Un sintoma que el limitador no arregla: la politica aprende a dejar una rueda de esquina atravesada a 90° para frenar o estabilizarse. Penalizarlo con un termino de recompensa no funciona: la politica encuentra otra forma de hacer lo mismo (por ejemplo, aparcar la rueda y dejarla sin traccion, que un termino de incoherencia longitudinal no ve).

Lo que funciona es cambiar quien decide:

1. Un **controlador cinematico** calcula los angulos y velocidades de cada rueda desde `(vx, wz)`, con el centro de rotacion sobre el eje de las ruedas medias (las que no tienen direccion). Sin RL ya conduce limpio en llano.
2. La politica aporta solo una **correccion acotada** encima: unos pocos grados de direccion y una correccion de traccion.

La ventaja es una cota algebraica, no estadistica: en recta el nominal es cero, asi que el angulo maximo alcanzable es el residual, y la rueda no puede atravesarse aunque la red lo pida. Empezar con un residual estrecho y ampliarlo solo si los datos lo piden.

Una ablacion ayuda a saber si el residual sirve: sustituirlo por cero o por una correccion fija por regimen. En llano una correccion fija iguala al residual aprendido; en terreno irregular el aprendido gana. Es ahi donde la red se gana su sitio.

### 3.4 Traccion comun

Una variante mas simple usa una sola velocidad de traccion para las seis ruedas y deja la direccion en el nominal puro. Tiene menos grados de libertad que aprender y su consigna es directa de estimar a bordo. Su coste: no puede girar sobre el sitio y la rueda interior media desliza algo en los giros. Si una variante se construye a partir de otra, hay que comprobar que conserva lo que el controlador anterior hacia fuera de la red (por ejemplo, el modo de parada de la seccion 4), porque se pierde sin avisar.

## 4. Parada: un modo del controlador, no algo que la red aprende

Con la recompensa de seguimiento de la Parte 2 (`exp(-error²/std²)` con `std = 0.05`), un rover con el chasis casi quieto y las ruedas girando a 1 rad/s cobra casi toda la recompensa. La politica no tiene motivo para detener las ruedas, y una penalizacion especifica de rueda girando con comando cero reduce el giro pero no lo detiene.

Mandar objetivos de traccion cero si detiene ruedas y chasis. Por eso la parada es un modo explicito del controlador:

- se activa solo con las tres componentes del comando por debajo de una tolerancia minima (del orden de 1e-3, muy por debajo del comando lento mas pequeño);
- nunca se activa en un giro sobre el sitio;
- pone a cero la traccion y pasa la direccion por el limitador, manteniendo coherentes el estado del limitador y las observaciones;
- se comprueba entrada, mantenimiento y salida, sobre todo al reanudar a la velocidad mas baja.

Es una decision de arquitectura: no se puede presentar como que la politica aprendio a parar, ni como parada de emergencia.

## 5. Actuadores y limites reales del motor

### 5.1 Que dice el datasheet y que no

Para un motorreductor como el de ROBERT (Pololu 37D, 131.25:1, 12 V):

| Dato | Valor | Lectura correcta |
| --- | --- | --- |
| Velocidad sin carga | 76 rpm = 7.96 rad/s | El maximo de la curva; con carga es menor |
| Carga maxima recomendada | 10 kg·cm = 0.981 N·m | Lo que se puede sostener |
| Par instantaneo recomendado | 25 kg·cm = 2.452 N·m | Picos; el fabricante no da su duracion |
| Par de bloqueo | 45 kg·cm = 4.413 N·m | Extrapolado; puede dañar el motorreductor y baja al calentarse |

Un limite de operacion elegido (por ejemplo 10 m/min, que con este radio son 2.045 rad/s) no es la velocidad del motor. Y usar el par de bloqueo como tope de simulacion produce una politica para un motor que no existe.

### 5.2 Modelo de actuador

- Usar un actuador con curva par-velocidad (`DCMotor` en Isaac Lab) mejor que un drive implicito con un tope fijo: el par disponible cae con la velocidad como en el motor real.
- La inercia reflejada del rotor (`armature` = inercia del rotor × relacion²) importa a baja velocidad. Si no esta en el datasheet, es un supuesto y hay que declararlo.
- La ganancia del servo de velocidad limita el par antes que el tope: par ≈ K · (w_objetivo − w). A baja velocidad, con K pequeña, el rover se queda corto de par aunque el tope sea alto. Elegir K con un barrido y criterios fijados antes (seguimiento, oscilacion, par en giros y arranques), no por cuantos obstaculos cruza.
- Medir "par al limite" contra la curva acotada por el tope, rueda a rueda, y separar los cortes del evaluador de los fallos del rover.

### 5.3 Escalones: la altura depende del par

En la escalera de entrenamiento la rueda media choca de frente con la cara del escalon y no pivota: ese es el modo de fallo que se ve. Pero la altura maxima no es un limite geometrico fijo, sino que se mueve con el par disponible y con K. Con la misma politica y una evaluacion fija (30 intentos por condicion, criterio de cruce completo):

| Tope de par por rueda | Ganancia K | Resultado |
| --- | --- | --- |
| 0.981 N·m (el del entrenamiento) | 1.0 | cruza de forma fiable hasta unos 5 cm |
| 2.452 N·m sostenido (supuesto de simulacion) | 4.4 | 6 cm en 30 de 30 a 0.10, 0.20 y 0.30 m/s; 10 cm una vez en 30 |

Dos consecuencias practicas:

- **Si el limite lo pone el par, entrenar no lo sube.** Un bloque de entrenamiento con un curriculum de escalones con el mismo tope no mejoro el cruce y empeoro el seguimiento en llano. Mirar el par al limite antes de gastar un entrenamiento.
- **Cada resultado va con su fisica.** Un resultado medido con un tope o una K distintos no vale para otra configuracion. Guardar con cada medida el tope, la K y el checkpoint, y descartar al comparar los ficheros que no coincidan.

En rampa el modo de fallo es atascarse, no volcar; en ROBERT empieza hacia los 20° y, con el tope de 0.981 N·m, el par esta al limite casi todo el tiempo en las rampas altas: tambien ahi lo que limita es el tope.

## 6. Observaciones que el robot puede dar

### 6.1 Actor con lo medible, critico privilegiado

El actor solo debe recibir lo que el robot mide: encoders de ruedas y reductores, IMU (velocidad angular y gravedad proyectada), comando, sus propias acciones, una estimacion de la velocidad lineal y el mapa de alturas. Nada de posiciones de la suspension pasiva si no tiene sensores. El critico puede ser privilegiado (estado completo del simulador) porque no viaja al robot. Cuando cambia la dimension de las observaciones hay que entrenar o adaptar el checkpoint (seccion 7.3), nunca rellenar con ceros sin mas.

### 6.2 Sensibilidad a la velocidad estimada

Antes de elegir estimador, perturbar solo la entrada `base_lin_vel` del actor (antes del normalizador, sin tocar la fisica) con sesgo, ruido y retardo por separado. En ROBERT la politica rechaza la mayor parte de un sesgo constante y tolera ruido blanco moderado; lo que de verdad la degrada es la **antiguedad** de la medida. De ahi sale un presupuesto del estimador separado en sesgo, ruido y retardo, que no se resume en un solo numero.

### 6.3 El mapa de alturas

- La rejilla sigue la guiñada del rover, no el cabeceo ni el balanceo (`ray_alignment="yaw"`). En el robot hay que construirla igual desde la nube de puntos.
- **Una celda sin observar no puede valer 0.0.** En entrenamiento 0.0 es una altura concreta; con la referencia vertical de la rejilla puede significar un escalon de medio metro hacia abajo, y el rover se paraliza. Rellenar con la mediana de lo observado borra los agujeros. Ninguna imputacion es segura: hace falta un canal de confianza por celda y que la decision de avanzar sobre terreno no observado se tome fuera de la red.
- Una memoria anclada en el mundo conserva el terreno que queda atras de la camara; al arrancar con la memoria vacia tarda decenas de segundos en llenarse.

### 6.4 La camara simulada

- Un raycaster solo ve las mallas que se le pasan. Si solo traza contra el suelo, los obstaculos hechos con cajas y el propio rover son invisibles para el. Para ver obstaculos hace falta la version multi-malla con sus prims; para la autooclusion, una mascara con la geometria del rover o las mallas del propio robot.
- Comprobar que el centro optico no queda dentro de la carcasa del vehiculo: un raycaster que atraviesa el rover no lo delata.
- Comprobar la nube contra una geometria conocida (suelo plano, cajas de altura conocida) a varias distancias. Un desfase de medio pixel al desproyectar la profundidad produce un sesgo de altura que crece con la distancia; corregido, el error baja a decimas de milimetro en simulacion. Eso es consistencia geometrica en simulacion, no la precision de una camara real.
- El sensor previsto es una camara estereo (ZED 2i). El raycasting no simula la correspondencia estereo, asi que su ruido y sus fallos no estan representados.

## 7. Exportar y transferir

### 7.1 El contrato

Escribir, a partir de la configuracion efectiva y no de memoria:

- observaciones del actor: nombre, orden, indice, unidades, marco, recorte y de donde sale cada una en el robot;
- acciones: orden, signos del CAD, escalas, recortes y, si los hay, el limitador y el modo de parada, que viven fuera de la red;
- el mapa de alturas: orden de las celdas (que eje varia primero), referencia vertical, valor de una celda sin dato;
- frecuencia de control y lo que falta medir (velocidad real de los reductores, latencias, ruido).

Volcarlo con un script. Las suposiciones sobre el orden suelen estar mal: por ejemplo, un grupo de velocidades de ruedas y reductores puede estar intercalado y no agrupado.

### 7.2 ONNX y un nucleo sin simulador

- Comprobar si la normalizacion de observaciones va dentro del checkpoint y del ONNX. Si va, aplicarla otra vez fuera cambia la salida en ordenes de magnitud.
- El ONNX no incluye el limitador ni el modo de parada: reproducirlos en el nodo.
- Un nucleo en numpy, sin Isaac Lab ni ROS, que reproduzca la cadena completa permite probar la transferencia: comparar contra la implementacion interna con secuencias reales (incluidos los reinicios de episodio) y despues en lazo cerrado.
- Isaac Sim y ROS 2 pueden necesitar versiones de Python distintas; entonces la politica corre en otro proceso y los dos lados avanzan sincronizados paso a paso. El controlador ejecuta cuando tiene todas las señales del mismo paso de simulacion.

### 7.3 Adaptar un checkpoint cuando crecen las observaciones

Para no entrenar desde cero al añadir una entrada (por ejemplo, el objetivo aplicado del limitador): añadir columnas con pesos cero en la primera capa del actor y del critico, ampliar el normalizador conservando sus estadisticas y reiniciar el optimizador. Verificar que la politica adaptada da exactamente la misma salida que la original con las entradas antiguas antes de entrenar nada.

## 8. Lista de comprobacion

Antes de entrenar:

- [ ] La configuracion efectiva (`params/env.yaml`) tiene los recortes, actuadores y observaciones que se creen.
- [ ] No hay copias de los archivos de configuracion fuera del paquete.
- [ ] Signos y orden de las acciones comprobados sin RL, con una medida independiente.
- [ ] El tope de par y la ganancia del servo son los del motor que se quiere imitar, y estan anotados.
- [ ] El criterio de exito y los umbrales estan escritos antes de mirar resultados.

Antes de comparar o elegir checkpoint:

- [ ] Evaluacion determinista, sin ruido, normalizacion congelada y cuota exacta.
- [ ] El rover se ha asentado antes de empezar a medir.
- [ ] Posiciones y niveles fijos; en rampas concentricas, rampa asignada por indice.
- [ ] Tres semillas si las diferencias son pequeñas.
- [ ] Cada resultado lleva su fisica y su checkpoint; no se mezclan configuraciones.
- [ ] Si algo falla de forma intermitente y los numeros no lo explican, verlo con interfaz grafica antes de seguir analizando.

## Donde esta cada cosa

| Que | Donde |
| --- | --- |
| Montar la tarea, entrenar y leer metricas | [README.md](README.md) de esta parte |
| Codigo de la tarea base | [referencia-completa.md](referencia-completa.md) |
| Codigo de las fases 2 a 4 | [referencia-fases-2-4.md](referencia-fases-2-4.md) |
| Importar el rover y resolver el lazo cerrado | [Parte 1](../01-simulacion/README.md) |
