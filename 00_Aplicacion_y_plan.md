# Trabajo integrador Robótica II — Aplicación, requisitos y plan

> Documento de trabajo (versión 0.1, 06/11/2026). Es la base de la sección 2 del informe
> («Aplicación y requisitos») y del registro de decisiones que pide el enunciado (§8).
> Los valores marcados **[a medir]** o **[estimado]** se reemplazan cuando haya datos del CAD o del laboratorio.
> Cada cambio de requisito se anota en la tabla de la sección 7 con fecha y motivo.

---

## 1. Aplicación propuesta

**«Celda de abastecimiento de piezas»**: el AGV lleva una pieza desde una **estación de carga (A)** hasta la
**estación del brazo (B)**. Allí, el brazo 6R la toma de la bandeja del AGV y la coloca en un
**alojamiento de una bandeja de trabajo (C)** con una orientación especificada (giro de 90° alrededor de z).
Después, el AGV vuelve vacío a A.

Por qué conviene esta aplicación:

- Usa las dos capacidades que el enunciado quiere ver: el **desplazamiento lateral** del mecanum (aproximación
  lateral a la mesa del brazo) y un **tramo cartesiano con orientación** del brazo (descenso vertical para insertar
  la pieza en el alojamiento).
- La pieza es la misma en los dos robots: el AGV la lleva físicamente (carga real y medible) y el brazo la manipula
  en el entorno virtual con la **misma masa y el mismo centro de masa**. Así se cumple la condición de «datos
  coherentes de pieza, carga y condiciones de entrega».
- Es una tarea chica y repetible, adecuada para el tiempo disponible.

### Esquema de planta (vista superior, marco global {G} del piso)

```
 y_G
  ^
  |                     [C] bandeja de trabajo
  |                          |
  |          +---------------+------+
  |          |   mesa del brazo  (B)|  <- base del brazo {0}
  |          +----------^-----------+
  |                     |  aproximación lateral del AGV (0,6 m en y)
  |  [A] carga          |
  |   o---------------->o
  |   (0,0)   avance x 1,2 m
  +-------------------------------------------> x_G
```

---

## 2. Pieza y carga

| Dato | Valor | Origen |
|---|---|---|
| Forma | Cubo 40 × 40 × 40 mm, PLA impreso con lastre interior | definido |
| Masa | 0,100 kg (ajustar el lastre con balanza) | **[a medir]** |
| Centro de masa | Centro geométrico (lastre centrado) | estimado |
| Inercia central | m·a²/6 = 2,67·10⁻⁵ kg·m² (cubo homogéneo equivalente) | calculado |
| Agarre | Pinza paralela de 2 dedos, sobre dos caras opuestas | definido |
| Apoyo en el AGV | Bandeja con alojamiento de 46 × 46 mm y chaflán de entrada | definido |
| Carga extra para el ensayo del AGV | +0,5 kg conocidos, centrados (arandelas en bolsa) | **[a verificar con el kit]** |

---

## 3. Geometría de la tarea y marcos de referencia

Convenciones (las de la cátedra): SI y radianes en el cálculo, giros positivos antihorarios, superíndice de trama
en las transformaciones (⁰T₆). En la interfaz se pueden mostrar mm y grados con rótulo.

| Marco | Definición |
|---|---|
| {G} | Global del piso. Origen en la marca de la estación A, x_G hacia B, z_G hacia arriba |
| {AGV} | Cuerpo del AGV: origen en el centro geométrico del chasis, x hacia adelante, y a la izquierda |
| {0} | Base del brazo: origen en el eje 1 a la altura del plano de apoyo, z hacia arriba |
| {6} / {TCP} | Brida y punto de herramienta: entre los dedos, en el centro de la pieza tomada |
| {C} | Alojamiento de la bandeja de trabajo |

Base del brazo sobre un pedestal cuya altura iguala la de la bandeja del AGV (h_bandeja **[a medir]**, ≈ 0,10 m).
Así, la toma y la colocación quedan aproximadamente en z₀ ≈ 0.

| Punto | Posición en {0} | Orientación de la herramienta |
|---|---|---|
| Toma (pieza en el AGV) | (0,30 ; 0 ; 0,02) m | z_TCP hacia abajo, RPY = (0, π, 0) |
| Aproximación de la toma | (0,30 ; 0 ; 0,10) m | igual |
| Colocación (C) | (0 ; 0,30 ; 0,02) m | z_TCP hacia abajo, girada 90° en z |
| Aproximación de C | (0 ; 0,30 ; 0,10) m | igual |
| Reposo (HOME) | a definir con el CAD, lejos de singularidades | — |

La AGV llega a B con su bandeja centrada en (0,30 ; 0) de {0}. El error de llegada del AGV es la variable que
conecta los dos desarrollos (ver requisito R-AGV-3).

---

## 4. Prestaciones y restricciones (requisitos)

### 4.1 Brazo antropomórfico 6R

| Id | Requisito | Valor propuesto | Justificación |
|---|---|---|---|
| R-B-1 | Alcance horizontal del TCP con la herramienta hacia abajo | ≥ 0,38 m | La toma está a 0,30 m; se agrega un margen para no trabajar cerca de la singularidad de brazo estirado |
| R-B-2 | Configuración | 6R con muñeca esférica (desacople cinemático) | Permite una cinemática inversa analítica (Tema II) |
| R-B-3 | Carga nominal / máxima | 0,10 kg / 0,25 kg (a confirmar con el cálculo dinámico) | Pieza + margen. La carga máxima se justifica con el par del eje más exigido (§4.4 del enunciado) |
| R-B-4 | Error de posición en la colocación | ≤ 1 mm (simulado) | Holgura del alojamiento: 46 − 44 mm ⇒ ±1 mm por lado con la pieza dentro de la pinza |
| R-B-5 | Error de orientación en la colocación | ≤ 1° (alrededor de z) | Un giro de 1° desplaza la esquina del cubo ≈ 0,5 mm, dentro de la holgura |
| R-B-6 | Error de seguimiento durante el movimiento | ≤ 5 mm | Evitar choques con el borde del alojamiento en la aproximación |
| R-B-7 | Tiempo de ciclo toma–colocación–retorno | ≤ 12 s | Debe ser compatible con el ciclo del AGV (sección 4.3) |
| R-B-8 | Límites articulares, velocidad y aceleración | a definir con el CAD; primera cota: 90°/s, 180°/s² | Accionamientos de tipo hobby (paso a paso NEMA 17 con reducción) |
| R-B-9 | Diseño | Imprimible en PLA/PETG, cama ≤ 220 × 220 mm | Impresora típica disponible |

Referencia comercial que se adapta: **KUKA KR 6 R700** (TP1, Robot 3; también se menciona en el Tema V).
Propuesta de escala simplificada (a1 = a3 = 0 para que la MCI sea más simple):

| Parámetro | KR 6 R700 (mm) | Propuesta (m) |
|---|---|---|
| d1 (altura del hombro) | 400 | 0,18 |
| a1 (desplazamiento del hombro) | 25 | 0 |
| a2 (brazo) | 315 | 0,20 |
| a3 (desplazamiento del codo) | 35 | 0 |
| d4 (antebrazo) | 365 | 0,20 |
| d6 (brida) + herramienta | 80 + — | 0,05 + 0,08 |

Estimación preliminar del par estático del eje 2 con el brazo horizontal y la carga máxima, con masas
**[estimadas]**: ≈ 4–5 N·m. Un NEMA 17 da ~0,4 N·m ⇒ hace falta una reducción de 15:1 a 20:1 en el eje 2.
Esto se rehace con las masas del CAD (es una de las preguntas de investigación).

### 4.2 AGV mecanum OSOYOO

| Id | Requisito | Valor propuesto | Justificación |
|---|---|---|---|
| R-AGV-1 | Recorrido A→B | 1,2 m en x + 0,6 m lateral en y | Muestra el avance y la capacidad lateral (enunciado §5.4) |
| R-AGV-2 | Velocidad / aceleración máximas del cuerpo | 0,20 m/s, 0,30 m/s² **[a ajustar tras identificar]** | Debajo del límite de las ruedas; la pieza no desliza: a ≪ μ·g ≈ 3 m/s² |
| R-AGV-3 | Error de llegada a B (medido con referencia externa) | ≤ 20 mm en posición, ≤ 3° en orientación | Capacidad de captura de la pinza (apertura de 70 mm para una pieza de 40 mm ⇒ ±15 mm) más el chaflán de la bandeja |
| R-AGV-4 | Tiempo de traslado A→B | ≤ 15 s | Ciclo total ≤ 30 s con el brazo y el retorno |
| R-AGV-5 | Período de control de ruedas | 20 ms (50 Hz) **[a verificar]** | Debe ser compatible con la resolución de los encoders a baja velocidad |
| R-AGV-6 | Seguridad | Parada desde la interfaz, parada ante pérdida de comunicación > 300 ms, límite de PWM, reinicio controlado | Enunciado §5.5 |

### 4.3 Ciclo completo

Carga en A → traslado A→B (≤ 15 s) → toma y colocación del brazo (≤ 12 s) → retorno B→A (≤ 15 s): **≤ 45 s**.

---

## 5. Decisiones de arquitectura (propuestas)

1. **Tabla única de parámetros**: un archivo `parametros/parametros.json` que leen Octave (`jsondecode`), Python y el
   script de CoppeliaSim. Así se cumple la coherencia que pide el §7.1 sin copiar valores a mano.
2. **Quién integra la dinámica del brazo**: Octave. Ahí se integran la planta (Newton–Euler recursivo numérico +
   RK4) y el controlador, y se exporta q(t) a CSV. CoppeliaSim **solo muestra** el movimiento: articulaciones en modo
   cinemático, con posiciones impuestas por un script que lee el CSV (o por la API remota ZMQ). Así, el movimiento
   que se ve corresponde al lazo de control calculado, y los controladores internos del simulador quedan
   desactivados (se declara en el informe). Es más seguro que depender del motor físico de CoppeliaSim en modo par.
3. **Control del brazo**: referencia = PD + compensación de gravedad; con compensación dinámica = par calculado.
   Mismo ensayo con la carga nominal, con +100 % de carga sin actualizar el modelo y con una perturbación escalón.
4. **AGV**: lazo interno PI de velocidad por rueda en el Arduino (encoders); lazo externo de pose (odometría) en la
   simulación. En el ensayo físico se realimentan las velocidades de rueda y, si da el tiempo, la odometría. La
   referencia externa es una grilla en el piso + video.
5. **Interfaz**: una sola aplicación Python (PyQt) con dos pestañas. Brazo: escribe los parámetros → ejecuta
   `octave-cli` → lee el CSV → grafica y envía a CoppeliaSim. AGV: puerto serie con el Arduino, modo virtual/físico,
   registro CSV con identificación del ensayo.

---

## 6. Preguntas de investigación (mínimo dos alternativas por robot)

| Robot | Pregunta | Alternativas | Variables observadas |
|---|---|---|---|
| Brazo | ¿Cuánto mejora el seguimiento la compensación dinámica cuando la carga real difiere del modelo? | PD + g vs par calculado; carga 0,10 vs 0,25 kg | Error máximo y RMS, par pico y RMS |
| Brazo | ¿Qué reducción minimiza el par motor del eje 2 sin exceder su velocidad? | N = 10, 20, 30 | Par motor pico/RMS, velocidad del motor, inercia reflejada |
| AGV | ¿Por qué el desplazamiento lateral es menos preciso que el longitudinal? | Avance x vs lateral y, misma distancia y velocidad | Error final, deriva de orientación, odometría vs video |
| AGV | ¿Cuánto reduce el error de llegada el lazo de velocidad de ruedas? | Lazo abierto (PWM fijo) vs PI por rueda | Error final, repetibilidad (n ≥ 3) |

---

## 7. Registro de decisiones

| Fecha | Decisión / cambio | Motivo |
|---|---|---|
| 06/11 | Aplicación «celda de abastecimiento», pieza cubo 40 mm 0,1 kg | Usa el desplazamiento lateral del mecanum y un tramo cartesiano con orientación; es la misma pieza en los dos robots |
| 06/11 | Brazo escalado del KR 6 R700 con a1 = a3 = 0 | Cinemática inversa analítica más simple; dimensiones compatibles con la impresión 3D |
| 06/11 | La dinámica del brazo se integra en Octave; CoppeliaSim solo visualiza | Trazabilidad entre el lazo de control y lo que se muestra |
| 06/11 | AGV = OSOYOO M2.0 metal, con encoders Hall que hay que cablear (A8–A15) | Datos del manual; los encoders no vienen conectados de fábrica |
| 06/11 | CAD del brazo generado por un script paramétrico de Fusion que lee `parametros.json` | La geometría del CAD, Octave y CoppeliaSim sale de la misma tabla; los cambios de medida no rompen la coherencia |
| 06/11 | Tabla DH definitiva v0.1: pose cero con brazo vertical y antebrazo hacia +x₀; α = (−90, 0, −90, 90, −90, 0)°, offset θ₂ = −90° | Verificado en Octave (`verificar_pose_cero.m`): coincide con el CAD y la muñeca es esférica |
| 06/11 | MCI analítica por desacople (8 soluciones, selección por continuidad y límites) y Jacobiano geométrico; det J = −a₂d₄·ρ·cos q₃·sin q₅ | Verificado: ida y vuelta en 2000 poses (error 10⁻¹³) y J contra diferencias finitas (3·10⁻¹⁰). Ver `informe/05a_cinematica_brazo.md` |
| 06/11 | Dinámica: Newton–Euler recursivo numérico (principal) + Lagrange (contraste); masas desde los STL con densidad efectiva de 800 kg/m³ | NE = Lagrange (10⁻¹⁰), energía conservada, volúmenes = Fusion (±0,08 %). Ver `informe/05b_dinamica_brazo.md` |
| 06/11 | **N₂: 20 → 30** | Con 20, la carga admisible estática era 0,221 kg < 0,25 kg (R‑B‑3); con 30 es 0,379 kg y limita el eje 4 |
| 06/11 | AGV: motor GM25-370 75:1 9 V, pinout por rueda, Hall en puerto K decodificado x4, comunicación por USB primero; geometría l_x = 62,2 / l_y = 107,6 mm como estimación a medir | Análisis del proyecto de un compañero y del informe ejemplo (`referencias/analisis_proyecto_koba.md`) |
| 06/11 | Firmware AGV v0.1.0: lazo 10 ms, encoders x4 (PCINT2), PI de ruedas + prealimentación con zona muerta, quíntico + lazo de pose a 50 Hz, watchdog 300 ms; consola de laboratorio en Python con registro CSV/JSON | Compila para la Mega (36 KB / 2,4 KB); sin cargar ni probar. Ver `agv/LEEME_FIRMWARE.md` |
| 06/11 | Trayectorias del brazo: ciclo de 9 tramos (PTP quíntico + LIN con orientación por eje–ángulo), 10,11 s; v_cart,máx 0,10 → 0,15 m/s | Con 0,10 m/s el ciclo duraba 12,31 s > 12 s (R‑B‑7). Todos los límites se cumplen; motor más exigido: eje 2 al 31 % pico. Ver `informe/06a_trayectorias_brazo.md` |
| 06/11 | `dinamica_rapida.m` (M y c + g en una pasada, 5 ms) | Necesaria para simular el control a 1 kHz en tiempos razonables; coincide con rne_brazo hasta 10⁻¹⁶ |
| 07/11 | Control del brazo: PD+g (referencia) vs par calculado; se agrega par calculado + integral con antiwindup como mejora | Con error de modelo (carga 0,25 kg, masas +15 %) ni PD+g ni el par calculado cumplen R‑B‑4 (≤ 1 mm); con la integral, ≤ 0,19 mm en los 3 ensayos. Ver `informe/07a_control_brazo.md` |
| 07/11 | Interfaz única PySide6 + pyqtgraph (`interfaz/robot_gui.py`): brazo virtual vía octave-cli y AGV físico por USB, con registro de ensayos | Cumple §7.2; probada de punta a punta en el brazo y con telemetría sintética en el AGV. Ver `interfaz/LEEME_INTERFAZ.md` |
| 07/11 | Modelo y simulación del AGV en Octave: cinemática mecanum, dinámica plana con motor CC de la hoja GM25-370, deslizamiento, encoders; mismo control que el firmware | Calibración de odometría lleva A→B de 65 mm a 4,9 mm (R‑AGV‑3 ✓). Ver `informe/05c_07b_agv_modelo_control.md` |
| 07/11 | AGV v_máx 0,20 → 0,25 m/s (JSON y firmware) | Con 0,20 el trayecto A→B duraba 16,9 s > 15 s (R‑AGV‑4); con 0,25, 13,7 s |
| 07/11 | Pieza crítica = L2. Se imprime **acostado** (capas normales al eje de la articulación) y **en dos mitades** con solape, 2 × M4 y 2 pasadores Ø4 | L2 (70 × 90 × 267 mm) no entra en 220 × 220 (R‑B‑9). Acostado, la flexión queda en el plano de la capa (parado: −26 a −29 % de FS). FS mín. 5,4 en colisión con el motor 2 saturado; sin pasadores, el juego de los agujeros daría hasta 2,2 mm en el TCP. Solape s = 65–115 mm (antes de la horquilla); la división está en `parametros.json` y la aplica el script de Fusion. Ver `informe/04a_verificacion_estructural_L2.md` |
| 07/11 | Firmware **agv-0.1.1** cargado en el kit con el AFR2 Loader 2.0.0 (compilar y flashear desde el Loader, COM5) | Primera prueba en la placa: lazo de 10 ms (peor período 10,016 ms, ejecución ≈ 0,9 ms), telemetría 20 Hz sin pérdidas. Encontró que en la 0.1.0 ω_máx y α_máx valían 0: un comentario había tapado su asignación al cambiar v_máx. Corregido y verificado con `GET` |
| 07/11 | **CPR medido = 1482 cuentas/vuelta (×4)**, no 3300; reducción efectiva ≈ 34:1, no 75:1. Máscaras `SIGN 15 5` (los 4 motores y los encoders izquierdos invertidos). Firmware agv-0.1.2: `SAVE` ya no dispara el watchdog | 5 vueltas a mano por rueda con nuestro firmware, dispersión 1,8 %. Coincide con el firmware de Mendelevich y Baldoni en el mismo kit (1500 cuentas/vuelta, mismos signos de encoder; de ahí la inversión de los motores, a confirmar con la prueba de sentidos). Con 3300 el lazo de ruedas habría medido 2,2 veces menos velocidad que la real. El modelo del motor (75:1) se corrige con la identificación |
| 07/11 | Sesión 1 en el kit (robot elevado): sentidos confirmados (`SIGN 15 5`); identificación por escalones (`agv/datos_lab/20261107-224139_s1_identificacion`) → `MDL` FL 0,0826/42,8 · FR 0,0795/30,5 · RL 0,0844/79,1 · RR 0,0867/66,4 (rad/s/PWM, PWM), τ 0,12–0,20 s, guardados en EEPROM; PI de ruedas con error de régimen ≤ 0,04 rad/s en avance, lateral y giro (`…_s1_pi_ruedas`) | Las traseras arrancan recién con PWM 120–160 desde el reposo (fricción estática mayor que la que da la recta ajustada); el error de validación de la recta es 2–3 rad/s porque la respuesta no es lineal (salto a PWM 255): el PI lo corrige. A PWM 255 en las 4 ruedas el lazo llega a 5,6 ms de ejecución y un período de 20 ms (carga de interrupciones de los encoders), fuera del rango de uso (ω_rueda ≤ 10 rad/s). El guion de identificación sube la protección de traba a 2 s mientras dura |
| 07/11 | Escena de CoppeliaSim armada por script (`coppelia/construir_escena.py`, API ZMQ) con juntas cinemáticas que reproducen el CSV del lazo de control (q real del brazo con cti + pose real del AGV simulado) | Sin motor físico ni controladores internos: lo que se ve es exactamente lo que calculó Octave. Verificado: \|q_escena − q_csv\| ≤ 1,2·10⁻¹³ rad y TCP = pieza (0,0 mm) durante el agarre |
| 11/11 | Firmware **agv-0.2.0**: enlace **WiFi** (ESP8266 del shield con firmware AT configurado por la Mega, UDP transparente, puerto 4210/4211); ensayos sin cable | La consigna pide operar sin USB; latencia mediana 51 ms (p95 93 ms). Método tomado del firmware BioLog (Mendelevich y Baldoni) |
| 11/11 | Zonas muertas en el piso `MDL` 66/79/104/72 PWM y traba 1,5 s (5 s en modo 0, 2,5 s en modo 2 durante las corridas) | En el piso las ruedas se pegan por debajo de ~1,5 rad/s; con la traba de 0,5 s los movimientos se cortaban |
| 11/11 | Ensayos T1–T5 en el piso por WiFi con medición sobre baldosas de 30 cm (`agv/pc/corrida.py`, `mediciones_piso.csv`); calibración **η = (1,029; 0,805; 0,995)** | Lazo abierto: no completa maniobras (A→B a 52 cm). Modo 2 calibrado: A→B a 15 cm, repetible, orientación exacta; con 0,5 kg gira 8–12° (deslizamiento que los encoders no ven). R‑AGV‑3 no se cumple en el piso. Ver informe 8.B |
| 11/11 | Modelo del AGV ajustado a lo medido (reducción 33,7:1, K_e = 0,22, K_t = 0,09, fricción, dispersión, η) y simulación T1–T5 repetida | La simulación coincide con lo físico en lo que modela (avance, error lateral en y, giro); no tiene fricción estática, acople lateral→avance ni giro por arranques desparejos |
| 11/11 | Firmware **agv-0.2.1/0.2.3**: valores medidos y calibración como valores por defecto | El Loader borra la EEPROM al flashear (EESAVE sin programar); otro grupo usó el kit y su firmware había reemplazado al nuestro |
| 11/11 | Firmware **agv-0.2.2**: **pausa ante cortes de comunicación** en GOTO (congela la trayectoria y frena; falla recién a los 3 s) | Una corrida se cortó por huecos de WiFi de hasta 1,2 s. Probada por USB y WiFi con el robot elevado (`prueba_pausa_com.py`): sigue y termina a 3 mm |
| 11/11 | Interfaz: piso de baldosas con clic → GOTO, auto dibujado mirando a +y y en tiempo real, conexión WiFi que recuerda la IP del robot, avisos de falla/NAK y casilla de vigilancia de batería | Pedido de operación del usuario; la interfaz no encontraba al robot si este ya no mandaba su aviso de IP |
| 06/11 | Accionamientos propuestos: NEMA 17 + reductor impreso (ejes 1–3), NEMA 17 corto/NEMA 14 (ejes 4–6), pinza con micro servo | Imprimible y viable; ajustar con el cálculo de pares |

---

## 8. Plan de trabajo (6 días)

Confirmado: **hay acceso al AGV todos los días**, así que los ensayos físicos se reparten en toda la semana y el
fin de semana sirve para identificar y ajustar. No hay nada avanzado: se parte de cero. El CAD se hace en Fusion.
Se entrega el mié 11 a la noche. Persona 1 = brazo, persona 2 = AGV, con un repaso
cruzado diario de 20 minutos (los dos tienen que poder defender todo).

| Día | Brazo (persona 1) | AGV (persona 2) |
|---|---|---|
| **Vie 6** | Tabla DH, MCD, MCI y Jacobiano en Octave con verificaciones (MCD∘MCI, J numérico vs analítico, singularidades) | **Laboratorio**: relevamiento del kit (medidas, masas, motores, **¿encoders conectados?**, cableado, sentidos), prueba con las ruedas elevadas, firmware mínimo de lectura de encoders y PWM con registro serie |
| **Sáb 7** | CAD simplificado (Inventor) → masas, centros de masa e inercias; dinámica RNE + verificaciones (M simétrica, energía, Lagrange en un caso); dimensionamiento de motores y reducciones | Modelo cinemático mecanum directo/inverso y saturación; modelo dinámico plano + motor; simulación con lazo de pose en Octave |
| **Dom 8** | Trayectorias (PTP quíntico + LIN con orientación), control PD+g vs par calculado, ensayo con variación de carga; exportar CSV | Firmware: PI de rueda, protocolo serie, watchdog y parada; interfaz pestaña AGV (serie, registro) |
| **Lun 9** | Escena de CoppeliaSim (importar STL, articulaciones cinemáticas, reproducir el CSV); comprobación estructural de una pieza; archivos STL/3MF y plan de impresión | **Laboratorio**: identificación PWM→velocidad (zona muerta, constante de tiempo), ajuste de los PI, ensayos longitudinal, lateral y giro (lazo abierto y cerrado, video) |
| **Mar 10** | Interfaz pestaña brazo (parámetros → Octave → CoppeliaSim); figuras y tablas del brazo | **Laboratorio**: trayectoria combinada A→B, ensayo con carga extra, repeticiones (n ≥ 3), comparación con la simulación |
| **Mié 11** | **Los dos**: informe, videos, presentación, prueba de reproducibilidad desde cero, **entrega** | |
| **Jue 12** | **Defensa oral** | |

### Prioridades si hay atrasos

- **P1 (imprescindible)**: modelos del brazo (cinemática, Jacobiano, dinámica) con verificaciones; dos
  controladores comparados; movimiento en CoppeliaSim; cinemática mecanum; PI de rueda; los ensayos físicos 1 a 4;
  la interfaz conectada al Arduino; el informe.
- **P2**: ensayo físico con carga, modelo dinámico detallado del AGV, lazo de pose físico con odometría,
  comprobación estructural por elementos finitos (se puede hacer analítica), lista de materiales con costos.
- **P3**: impresión real de piezas, envío de referencias en línea a CoppeliaSim (alcanza con reproducir el CSV).

---

## 9. Estructura de carpetas propuesta

```
Proyecto Final Integrador - Robotica II/
  00_Aplicacion_y_plan.md      <- este documento
  parametros/parametros.json   <- tabla única (unidades y origen de cada valor)
  brazo/octave/   brazo/cad/   brazo/stl/   brazo/coppelia/   brazo/resultados/
  agv/octave/     agv/arduino/ agv/datos_lab/ agv/resultados/  agv/cad/
  interfaz/
  informe/        presentacion/   videos/
```

## 10. Kit OSOYOO: datos documentados y datos a relevar

Versión identificada: **«M2.0 Metal Mecanum» para Mega2560** (chasis metálico negro, motorreductores metálicos con
encoder Hall). Manual: OSOYOO, *m2.0-metal-mecanum-20260101.pdf*, <https://osoyoo.com/manual/m2.0-metal-mecanum-20260101.pdf>
(sin fecha de edición en la portada; la fecha sale del nombre del archivo). Ficha del producto: <http://osoyoo.com/?p=49235>.
**Confirmar en el laboratorio que la unidad coincide con estas fotos.**

| Dato | Valor | Clasificación | Fuente / método |
|---|---|---|---|
| Ancho total (rueda a rueda, exterior) | 245 mm | documentado | Manual M2.0, p. 1 |
| Placa del chasis | 155 × 200 mm | documentado | Manual M2.0, p. 1 |
| Rueda mecanum | Ø 80 mm, ancho 36 mm, eje Ø 4 mm, rodillos a 45° | documentado | Manual M2.0, p. 1 |
| Trocha 2·l_y (centro a centro de las ruedas) | ≈ 245 − 36 = 209 mm | calculado | **[a medir]** |
| Distancia entre ejes 2·l_x | **[a medir]** | — | regla/calibre |
| Radio efectivo r | ≈ 40 mm (el rodillo apoya en un punto) | estimado | **[calibrar]**: recorrido de N vueltas medidas con encoder |
| Configuración de rodillos (X u O vista desde arriba) | **[a mirar]** | — | Define los signos de la matriz cinemática |
| Motores | Motorreductor DC metálico con encoder Hall de 2 canales (M+, M−, VCC, GND, S1, S2) | documentado | Manual M2.0, p. 2 |
| Relación de reducción y cuentas por vuelta de rueda | **[a medir]** | — | Girar la rueda 10 vueltas a mano y contar flancos (el manual no lo indica) |
| Tensión nominal del motor | 9 V (inferido del nombre del producto) | estimado | **[verificar]** con la etiqueta del motor |
| Driver | OSOYOO «Model Y v2.0» (2 × TB6612, 4 canales) | documentado | Manual V2, p. 4 |
| Pines driver → Mega | M_A IN1..IN4 → D5, D6, D7, D8; M_A ENA/ENB → D11, D12; M_B IN1..IN4 → D22, D24, D26, D28; M_B ENA/ENB → D9, D10 | documentado | Manual M2.0, p. 10 |
| Asignación de motores | Del. der. → BK1/BK2; del. izq. → BK3/BK4; tras. der. → AK1/AK2; tras. izq. → AK3/AK4 | documentado | Manual M2.0, p. 10 |
| Encoders → Mega | **No vienen conectados** (el manual dice atar los cables sin usar) | documentado | Propuesta: los 8 canales a A8–A15 (interrupción por cambio de pin, puerto K) para decodificar en cuadratura ×4 |
| Alimentación | 2 × 18650 (3,7 V, 2000 mAh) en serie ≈ 7,4 V | documentado | Lista de partes; voltímetro incluido |
| Sensores auxiliares | Ultrasonido + servo, seguidor de línea, shield WiFi ESP8266 | documentado | Manual M2.0 |
| Masa total y centro de masa | **[a medir]** | — | Balanza; CM con la balanza bajo cada eje |

Observación: la tensión de batería (7,4 V nominal, ~8,4 V con carga plena) baja durante los ensayos ⇒ registrar la
tensión al inicio de cada ensayo; es una fuente declarada de discrepancia con el modelo.

## 11. Software

| Herramienta | Estado | Uso |
|---|---|---|
| Fusion (Autodesk) | instalado | CAD del brazo (script paramétrico en Python dentro de Fusion) y del AGV; exportar STL/3MF; masas e inercias |
| GNU Octave 10.3 (+ symbolic, control, `jsondecode`) | instalado | Cálculo y simulación |
| Python 3.13 (numpy, scipy, matplotlib) | instalado | Interfaz |
| Arduino IDE | instalado | Firmware del AGV |
| CoppeliaSim Edu 4.10.0 | instalado | Simulación gráfica (lo exige el enunciado). Ver `coppelia/LEEME_COPPELIA.md` |
| Paquetes Python: `pyserial`, `PySide6`, `pyqtgraph`, `coppeliasim-zmqremoteapi-client` | instalados | Serie con el Arduino, interfaz Qt, gráficos, conexión con CoppeliaSim |

Instalación de los paquetes: `py -3.13 -m pip install pyserial PySide6 pyqtgraph coppeliasim-zmqremoteapi-client`.
