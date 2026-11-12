# Análisis del proyecto de un compañero (ProyectoIntegradorKoba) — qué nos sirve

> Revisión del 06/11/2026 de la carpeta `ProyectoIntegradorKoba/ProyectoRobotica2-main` (proyecto «AutoFinalRobo2»,
> de Koba Chajchir). Su aplicación es distinta (brazo montado sobre el auto) y **mantenemos la nuestra**.
> De ese proyecto se toma información de hardware, lógica de programación y criterios de verificación.
> Todo lo que se adopte se cita como fuente de terceros en el informe y se verifica en nuestro kit.

## 0. Qué es esa carpeta

- **Seis módulos:**
  - firmware del ATmega2560 en C puro (avr-libc, sin Arduino);
  - firmware propio para el ESP8266 del shield WiFi (servidor HTTP y WebSocket);
  - interfaz web en React;
  - cargador de firmware para Windows;
  - CAD (Fusion y KiCad: placa OLED + encoder);
  - documentación de contratos e historial de cambios.
- **Gran parte del código es interfaz** (OLED con menú, radar con el ultrasonido, web con modelo 3D). El núcleo
  de movimiento útil ocupa unas 300 líneas (`AutoFinalRobo2-Atmega/src/services/afr2_product.c` y
  `src/platform/afr2_board.c`, `afr2_motor.c`).
- **Estado físico:** el firmware se cargó en la Mega (29/10) y se observaron los sentidos de giro. **No hay
  mediciones de cuentas por vuelta, identificación PWM→velocidad, ensayos de odometría ni de control en lazo
  cerrado.** Lo físico sigue pendiente también para ellos.
- **Incluye además `Documentacion/EjemploInformeTadeo.pdf`**: un informe completo de **otro grupo para este
  mismo trabajo integrador** (BioLog: brazo de 6 ejes + AGV OSOYOO; Mendelevich y Baldoni). Es la mejor
  referencia del nivel y la estructura que se espera. Se analiza en la sección 7.

## 1. Datos de hardware del kit (documentados, a confirmar en nuestra unidad)

### Motorreductor: GM25-370-24140-75-14.5D10

Fuente: `Documentacion/GM25-370-motor specification.pdf` y `Anexo_Tecnico_Motor_GM25-370_OSOYOO.pdf`.

| Magnitud | Valor de ficha | Uso para nosotros |
|---|---|---|
| Tensión nominal | 9 V CC | Con 2 × 18650 tenemos ≈ 7,4–8,4 V, menos la caída del puente H |
| Reducción | 75:1 | — |
| Motor sin carga | 11 500 rpm, ≤ 180 mA | Parámetros del modelo eléctrico |
| Salida sin carga / nominal | 150 rpm / 100 rpm (±10 %) | ω_rueda,máx ≈ 15,7 rad/s a 9 V → ≈ 0,63 m/s ideales con Ø80 |
| Par nominal de salida | 3 kgf·cm = 0,294 N·m, 1,2 A | ≈ 7,4 N por rueda con r = 40 mm |
| Par / corriente con rotor bloqueado | 9,5 kgf·cm = 0,93 N·m, 4,5 A (motor: 160 g·cm) | No es un régimen continuo |
| Eje | Ø4 mm con plano (D3,5), 14,5 mm | — |

**Modelo de motor CC estimado a partir de la ficha** (cálculo nuestro, clasificación E):
- R ≈ 9 V / 4,5 A = 2,0 Ω.
- K_e,salida ≈ (9 − 2·0,2)/15,7 ≈ 0,55 V·s/rad.
- K_t,salida·η ≈ 0,93/4,5 ≈ 0,21 N·m/A, de donde η del reductor ≈ 0,4 en el bloqueo.

Los dos K no cierran entre sí con exactitud (en el lado del motor difieren ≈ 2×), así que el modelo físico sirve
como orden de magnitud. **El modelo que manda es el identificado con ensayos PWM→velocidad**, como permite el
enunciado (§5.3).

### Encoder

- Hall de 2 canales (S1/S2). Los **11 PPR** son un dato del usuario, no de la ficha.
- Hipótesis H1 (encoder en el eje del motor):
  - x1 → 825 cuentas por vuelta de rueda;
  - x2 → 1650;
  - x4 → 3300.
- **Hay que medirlo**: girar la rueda 10 vueltas a mano y contar.
- **Colores** (manual p. 5): blanco = VCC, amarillo = GND, naranja = S1, verde = S2. OSOYOO advierte que hay
  variantes con **blanco y amarillo invertidos**: identificar V/G en la plaquita antes de energizar.
- Alimentación del Hall: **+5 V** regulados y GND común con la Mega. No usar la tensión de batería.

### Pinout por rueda

Coincide con nuestra lectura del manual y agrega la asignación rueda → pines:

| Rueda | Salida Model Y | IN1 | IN2 | PWM | Timer | Hall A | Hall B |
|---|---|---|---|---|---|---|---|
| Delantera izq. (FL) | M_B · BK3/BK4 | D26 | D28 | D10 | OC2A | A9 (PK1) | A13 (PK5) |
| Delantera der. (FR) | M_B · BK1/BK2 | D22 | D24 | D9 | OC2B | A8 (PK0) | A12 (PK4) |
| Trasera izq. (RL) | M_A · AK3/AK4 | D7 | D8 | D12 | OC1B | A11 (PK3) | A15 (PK7) |
| Trasera der. (RR) | M_A · AK1/AK2 | D5 | D6 | D11 | OC1A | A10 (PK2) | A14 (PK6) |

- Los 8 canales Hall están en el **puerto K (PCINT2)**: una sola interrupción atiende las 4 ruedas. Es la misma
  propuesta que ya teníamos.
- **Sensores:** línea IR1..IR5 en A4..A0, ultrasonido TRIG D30 / ECHO D31, servo D13.
- **WiFi:** UART1 D18/D19 con los jumpers E_RX/E_TX del shield. **D5 es un pin de motor**: no usar
  SoftwareSerial en D4/D5 como en los tutoriales para UNO.
- **Batería:** divisor 10 k/10 k + 100 nF a **A5**. A0–A4 están ocupados por la línea.
- **Pines libres** útiles para una parada de emergencia: D2/D3 (INT4/INT5) si no se usa su panel OLED.

### Geometría

- **Koba** (envolvente del CAD, no medición): semidistancia entre ejes l_x = 62,2 mm, semitrocha
  l_y = 107,64 mm, l_x + l_y ≈ 170 mm.
- **Nuestra estimación anterior:** l_y = 104,5 mm.
- **El informe de Tadeo** usa l_x = 95 mm y l_y = 100 mm, también estimados.
- **Hay que medir l_x y l_y**: el término l_x + l_y escala directamente el giro y la odometría angular.
- **Masa del auto:** ≈ 1,5 kg (dato comunicado, no pesado); Tadeo estimó 1,60 kg.

## 2. Lógica de programación que conviene adoptar

### 2.1 Captura de encoders (afr2_board.c)

- Una ISR `PCINT2_vect` lee **una sola vez** `PINK` y procesa las 4 ruedas en código desenrollado, sin bucles
  variables, floats ni nada pesado. Los contadores son `int32_t volatile` y se copian con `ATOMIC_BLOCK`.
- **Koba decodifica en x1:** cuenta el flanco ascendente de A y mira el nivel de B para el sentido.
  - Con 825 cuentas por vuelta y T_s = 10 ms, el cuanto de velocidad es 2π/(825·0,01) = **0,76 rad/s**
    (30 mm/s en el piso). Para controlar a 0,2 m/s eso es un error de cuantización de ~15 %.
  - **Mejora para nosotros:** decodificar en **x4 con tabla de transición** (habilitando PCINT también en
    los canales B), como Tadeo. El cuanto baja a 0,19 rad/s. Además, usar una ventana de 20 ms o un filtro
    para estimar la velocidad.
  - El costo de CPU es aceptable: a 0,25 m/s son ≈ 1 vuelta/s ≈ 3300 eventos/s por rueda.
- **Pull-ups:** Koba deja el puerto K **sin pull-ups internos** («a verificar»). Si las placas Hall son de
  colector abierto, hacen falta. Hay que medirlo con multímetro u osciloscopio y decidir.

### 2.2 Motores (afr2_motor.c)

- PWM en Timer1 (D11/D12) y Timer2 (D9/D10) en modo Fast PWM de 8 bits a ≈ 976 Hz. Timer0 queda para
  `millis()`.
- Tadeo subió la frecuencia a 3,9 kHz para reducir el zumbido y mejorar la linealidad. El TB6612 admite hasta
  100 kHz.
- **Cada motor tiene un bit de inversión de signo** (`motor_signs`) y cada encoder otro (`hall_signs`). Es
  imprescindible: en el auto de Koba, al probarlo, **avance y giro salieron invertidos**
  (`AFR2_FORWARD_WHEEL_SIGN = −1`, `AFR2_YAW_WHEEL_SIGN = −1`) y el lateral no.
  **Primer ensayo obligatorio: con las ruedas elevadas, rueda por rueda, PWM bajo y pulsos breves.**

### 2.3 Cinemática mecanum (patrón X, orden FL, FR, RL, RR)

Las ecuaciones de Koba, en firmware y en web, son las de Lynch y Park §13.2, con L = l_x + l_y:

- Inversa: v_FL = v_x − v_y − Lω, v_FR = v_x + v_y + Lω, v_RL = v_x + v_y − Lω, v_RR = v_x − v_y + Lω.
- Directa: v_x = Σv/4; v_y = (−v_FL + v_FR + v_RL − v_RR)/4; ω = (−v_FL + v_FR − v_RL + v_RR)/(4L).
- **Saturación:** escala común de las 4 ruedas, que conserva la dirección del movimiento (igual que en el
  Tema VI).
- **Odometría:** integración con el rumbo en el punto medio (θ + Δθ/2), que es más exacta que Euler.

### 2.4 Control y seguridad que conviene replicar (y mejorar)

| Función | Koba | Tadeo (informe) | Propuesta para nosotros |
|---|---|---|---|
| Lazo de rueda | PID entero en mm/s, 10 ms, Kp = 0,5 PWM/(mm/s), rampa 4 PWM por ciclo | PI 10 ms, Kp = 6 PWM/(rad/s), Ki = 60 PWM/rad, prealimentación con zona muerta, antiwindup por recálculo | PI + prealimentación identificada (K, u_dz), antiwindup condicional (Tema V), 10 ms |
| Lazo de pose | GOTO proporcional sin perfil | V_b = Rᵀ(ξ̇_d + K(ξ_d − ξ)), 50 Hz, quíntico generado a bordo | Igual que Tadeo, con nuestras trayectorias del Tema IV |
| Watchdog de comunicación | 1,5 s sin pulso → parada | 300 ms (3 latidos perdidos) → FALLA | 300 ms |
| Rueda trabada o encoder roto | PWM ≥ 40 sin cuentas durante 500 ms → falla | Sobrevelocidad > 1,5 ω_máx (encoder invertido) | Ambas |
| Otras | Período > 100 ms → falla; obstáculo < 150 mm (ultrasonido) | Batería < 6,6 V; parada de emergencia a pin; watchdog de hardware 250 ms; estados DESARMADO → ARMADO → FALLA | Máquina de estados de Tadeo + batería en A5 |
| Comunicación | ESP8266 con firmware propio, WebSocket → UART1 (muy costoso de hacer) | ESP-01 como puente UDP transparente; texto `$CMD,args*HH` con XOR; cola TX no bloqueante | **USB serie a 115 200 bit/s primero**; WiFi solo si sobra tiempo |

**Detalle técnico importante (Tadeo):** escribir 170 bytes con `Serial.write` bloquea el lazo unos 15 ms.
Hay que transmitir la telemetría en una cola no bloqueante o mantener los mensajes cortos.

## 3. Lo que NO conviene copiar

- **La arquitectura web + ESP con firmware propio + cargador de Windows.** Es enorme y casi todo es interfaz.
  Para cumplir el enunciado alcanza con Python/Qt por USB.
- **C bare-metal sin Arduino.** Para nosotros, Arduino IDE ya instalado es más rápido de escribir y depurar. Se
  pueden usar registros directos solo en la ISR del puerto K y en la frecuencia de PWM.
- **Los datos de motor del informe de Tadeo** (JGB37-520 1:30, L298N, CPR = 1320): **no corresponden** al kit
  M2.0 con Model Y (TB6612) y GM25-370 75:1 que documenta Koba.
- **La geometría de cualquiera de los dos como dato.** Son estimaciones; se miden.

## 4. Cambios para nuestro plan

1. **Firmware del AGV** (Arduino IDE), con dos capas: núcleo portable (cinemática, odometría, PI, pose,
   protocolo, estados) y capa de hardware (PWM, encoders, batería).
   - Pines de la tabla de la sección 1 y Hall en A8–A15 con x4.
   - Protocolo de texto con suma de verificación.
   - Watchdog de 300 ms, rampa y detección de rueda trabada.
2. **Primer día de laboratorio:**
   - continuidad y colores del Hall;
   - sentidos por rueda con las ruedas elevadas;
   - cuentas por vuelta (10 vueltas a mano);
   - pull-ups;
   - masa con balanza;
   - l_x, l_y y el radio efectivo (recorrido de 10 vueltas).
3. **Identificación** de K, u_dz y τ por rueda con escalones de PWM, registrando la tensión de batería.
4. **Procedimiento de ensayos** de Tadeo (§7.3.1):
   - cuadrícula de 0,25 m;
   - video a 30 fps con una regla de 1 m en cuadro;
   - batería ≥ 7,6 V;
   - 5 repeticiones;
   - factores η_x, η_y, η_ω ajustados con una tanda y verificados con otra.
5. **Modelo del accionamiento:** G(s) = K/(τs + 1) con zona muerta, más el modelo físico del motor CC estimado
   de la ficha como contraste.

## 5. Lo que tienen los informes de referencia y nos falta (brazo)

- **Tadeo**, con la misma consigna:
  - balanceador a gas en J2 (baja el par pico de J2 de 5,9 a 1,0 N·m);
  - motor de J4 detrás del codo como contrapeso (nosotros también);
  - correa GT2 a J5 y reductores cicloidales impresos;
  - 44 láminas de planos.
- **Control del brazo de Tadeo:**
  - ω_n = 25 rad/s en J1–J3 y 50 en J4–J6, ζ = 1;
  - PD+g con ganancias escaladas por M_ii(q_home);
  - Euler semi-implícito a 1 kHz;
  - error de modelo deliberado (masa del frasco y fricción al 70 %);
  - resultados: el par calculado da 0,64 mm de error máximo frente a 6,3 mm del PD+g.
- **Simulación gráfica:** Tadeo usó **Inventor** con movimiento impuesto (el enunciado acepta «Inventor u otro
  software»). Además comparó los pares que da Inventor con los de Newton–Euler como verificación independiente.
  Fusion no tiene un equivalente directo, así que **mantenemos CoppeliaSim**.

## 6. Diferencias de criterio a declarar en nuestro informe

- **Clasificación D/M/C/E** de cada parámetro (Tabla 12 de Tadeo) y la consigna «ajustar con un conjunto de
  ensayos y verificar con otro».
- **Tres tipos de error** separados en la discusión (numérico, de modelo y de control), como pide el §7.3.
- **El margen de tiempo del lazo** se mide con telemetría: tiempo de ejecución y peor período.

## 7. Estructura del informe de ejemplo (Tadeo, entrega 10/11)

Resumen con resultados numéricos y palabras clave; capítulos 1 a 9 que siguen el §10 del enunciado; anexos de
instrucciones, estructura de archivos, planos y registro de decisiones.

**Tablas que conviene replicar:**
- parámetros del AGV;
- límites y uso de la trayectoria;
- campos de la interfaz;
- tiempos del sistema;
- ensayos del brazo;
- ensayos del AGV (simulación frente a laboratorio).

**Su AGV quedó validado solo contra un gemelo SIL**: los resultados físicos estaban «por completar». Tener
ensayos físicos reales es una ventaja concreta para nosotros, porque tenemos acceso al kit todos los días.
