## 4.D Relevamiento del kit OSOYOO y representación del AGV

### 4.D.1 Datos del kit

Versión identificada: **OSOYOO «M2.0 Metal Mecanum» para Arduino Mega 2560**. Cada dato se clasifica en:
- **D** documentado: manual u hoja de datos;
- **M** medido;
- **C** calculado;
- **E** estimado.

Los valores se relevaron en el laboratorio el 07/11 y el 11/11/2026 (secciones 5.C.4 y 8.B) y están en
`parametros.json`, en el bloque `agv`.

Tabla: Parámetros del AGV y su procedencia

| Dato | Valor | Clase | Fuente / método |
|---|---|---|---|
| Ancho total exterior | 245 mm | D | Manual M2.0, p. 1 |
| Placa del chasis | 200 × 155 mm | D | Manual M2.0, p. 1 |
| Rueda mecanum | Ø80 × 36 mm, rodillos a 45° | D | Manual M2.0, p. 1 |
| Semitrocha l_y | 100 mm (±10) | M | App de medición del celular, centro a centro (2l_y = 20 cm). Antes: 107,6 mm estimado |
| Semidistancia entre ejes l_x | 50 mm (±10) | M | Ídem (2l_x = 10 cm). Antes: 62,2 mm estimado |
| Radio efectivo r | 40 mm (avance real/odometría 1,029) | D + M | Manual; la calibración en el piso da el radio efectivo en avance |
| Patrón de rodillos | X | M | Verificado: el lateral y el giro salen con el signo esperado (`GEO … 1`) |
| Motorreductor | GM25‑370, 9 V, encoder Hall de 11 PPR | D | Hoja de datos del motor |
| Reducción efectiva | ~33,7:1 (la ficha dice 75:1) | C | 1482 cuentas / (11 × 4) |
| Cuentas por vuelta de rueda | 1482 (×4) | M | 5 vueltas a mano por rueda: FL 1481, FR 1467, RL 1489, RR 1493 |
| K y zona muerta por rueda | 0,117–0,130 rad/s/PWM; 47–75 PWM elevado, 66–104 en el piso | M | Escalones de PWM (5.C.4) |
| Driver | «Model Y» v2.0, 2 × TB6612 | D | Manual V2, p. 4 |
| Alimentación | 2 × 18650 en serie, 7,0–7,4 V en los ensayos | D + M | Lista de partes; tester |
| Comunicación | USB y WiFi (ESP8266 del shield, UDP) | M | Latencia mediana 51 ms (7.C.2) |
| Masa | 1,5 kg | E | No se pesó: dato del proyecto de referencia del mismo kit |
| Inercia I_z | 0,017 kg·m² | C | Placa de 0,9 kg más 4 conjuntos rueda‑motor de 0,15 kg en (±l_x, ±l_y) |

**Sensores auxiliares.** El kit trae sensor de línea de 5 canales (A0–A4) y ultrasonido con servo. No se
usan: la aplicación no tiene una línea que seguir ni obstáculos en el recorrido A→B. El sensor de línea
sería útil para la aproximación final a B, como referencia externa que corrija la odometría (sección
9.5).

**Conexionado.**
- **Encoders.** No vienen conectados de fábrica. Se propone llevar los 8 canales Hall a A8–A15 (puerto K de
  la Mega), que comparten una sola interrupción por cambio de pin (PCINT2). Así se decodifican en
  cuadratura ×4 sin ocupar las interrupciones externas.
- **Motores.** Mantienen el cableado del manual (p. 10).

Tabla: Conexionado del AGV a la Arduino Mega

| Rueda | PWM | IN1 / IN2 | Encoder A / B |
|---|---|---|---|
| FL delantera izquierda | D10 | D26 / D28 | A9 / A13 |
| FR delantera derecha | D9 | D22 / D24 | A8 / A12 |
| RL trasera izquierda | D12 | D7 / D8 | A11 / A15 |
| RR trasera derecha | D11 | D5 / D6 | A10 / A14 |

### 4.D.2 Representación del AGV

- **Representación geométrica.** Se usa la del entorno gráfico (`coppelia/construir_escena.py`). Está
  construida con las medidas de `parametros.json`:
  - placas de 200 × 155 mm a 55 y 90 mm de altura, con la superior como bandeja;
  - ruedas de Ø80 × 36 mm en (±l_x, ±l_y);
  - motores, columnas y la placa Mega en sus ubicaciones relativas.
- **Coherencia.** El marco {AGV} de esa representación, con origen en el centro geométrico, es el mismo del
  modelo cinemático (5.C) y del firmware.
- **Medidas nuevas.** La escena se armó el 07/11 con l_x y l_y estimados (62,2 y 107,6 mm). Con los medidos
  (50 y 100 mm) se regenera con `coppelia/construir_escena.py`, que los lee del JSON.
- **Pendiente:** un CAD editable del AGV en Fusion con las medidas relevadas. La representación actual
  alcanza para el entorno gráfico y para la ubicación de componentes, pero no es un CAD de piezas.

![Representación del AGV en la escena (llegada a B con la pieza)](../coppelia/capturas/03_agv_en_B.png)
