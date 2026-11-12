# Interfaz de operación (enunciado §7.2)

```bash
py -3.13 interfaz/robot_gui.py
```

Requiere Python 3.13 con PySide6 6.12, pyqtgraph 0.14, numpy y pyserial 3.5:
```bash
py -3.13 -m pip install PySide6 pyqtgraph pyserial
```
La pestaña del brazo además necesita GNU Octave 10.3 (`octave-cli`), que se busca en la ruta de instalación
por defecto o en el PATH.

## Arquitectura

```
                 ┌──────────── Pestaña BRAZO (modo virtual) ────────────┐
 Usuario ──► campos con rango ──► config.json ──► octave-cli correr_desde_gui.m
                                                    │  referencia del ciclo (Tema IV, 1 ms)
                                                    │  controlador muestreado Ts = 1 ms (Tema V)
                                                    │  planta: dinámica 6 ejes, Euler semi-implícito h = 1 ms
                                                    ▼
                         resultado.csv · metricas.json · parametros_usados.json · octave.log
                                                    ▼
                         gráficos (q_ref vs q, error TCP, τ y límite) + tabla comparativa

                 ┌──────────── Pestaña AGV (modo físico) ───────────────┐
 Usuario ──► órdenes ($CMD,…*HH) ──USB 115200──► Arduino Mega (firmware agv_osoyoo)
                 ▲      latido PING cada 100 ms        │ PI de ruedas 100 Hz, pose 50 Hz,
                 │                                     │ odometría, watchdog 300 ms
                 └──── $TEL (5–50 Hz), $ACK/$NAK/$EVT/$PAR ◄┘
                         ▼
     estado + gráficos (10 Hz) · registro telemetria.csv / eventos.csv / ensayo.json
```

**Quién integra la dinámica:**
- Brazo: Octave.
- AGV físico: el robot real. El control corre en el Arduino; la PC solo envía órdenes y registra.
- AGV virtual: Octave, con `agv/octave/simular_agv.m`. Ejecuta el mismo algoritmo de control que el firmware
  sobre la planta simulada.

**Ritmos:**

| Proceso | Ritmo |
|---|---|
| Control del brazo (simulado) | 1 ms |
| Control de ruedas del AGV | 10 ms en el Arduino |
| Lazo de pose del AGV | 20 ms en el Arduino |
| Telemetría | 50 ms por defecto, configurable |
| Latido | 100 ms |
| Watchdog del firmware | 300 ms |
| Refresco de gráficos | 100 ms |

Los gráficos y la PC nunca frenan el lazo de control. El firmware informa el tiempo de ejecución y el peor
período, y la interfaz los muestra en la barra de estado.

## Pestaña Brazo (virtual)

| Campo | Unidad | Rango (defecto) | Efecto / cuándo se aplica |
|---|---|---|---|
| Identificación, notas | texto | ≤ 40 caracteres | Nombre de la carpeta del ensayo |
| Ley de control | — | PC + integral / Par calculado / PD + gravedad | Al pulsar Simular |
| ω_n ejes 1–3 | rad/s | 5–60 (20) | Rigidez del lazo: K_p = ω_n², K_d = 2ζω_n (escaladas por M_ii(q_home) en PD+g) |
| ω_n ejes 4–6 | rad/s | 5–100 (40) | Ídem para la muñeca |
| ζ | — | 0,5–2 (1) | Amortiguamiento del error |
| Fricción conocida por el modelo | fracción | 0–1 (0,7) | Error de modelo del par calculado |
| Masa real de la pieza | kg | 0–0,4 (0,1) | Planta |
| Masa supuesta por el modelo | kg | 0–0,4 (0,1) | Controlador |
| Escala de masas reales | × | 0,7–1,3 (1) | Incertidumbre de parámetros de la planta |
| Perturbación: junta, inicio, duración, amplitud | —, s, s, N·m | 1–6, 0–30, 0,01–2, ±3 | Pulso de par externo en la planta |
| Margen de duración | × | 1,0–2,0 (1,10) | Factor sobre la duración mínima de cada tramo. Con 1,10 se reutiliza la referencia calculada; con otro valor se regenera (~1 min) |

- Cada ensayo se guarda en `interfaz/ensayos/brazo/<fecha>_<id>/`: config.json, resultado.csv, metricas.json,
  parametros_usados.json y octave.log.
- **Superponer** compara hasta 4 ensayos en los mismos gráficos y en la tabla.
- **Cargar ensayo…** vuelve a abrir uno guardado.

## Pestaña AGV

**Modo físico o virtual** (selector *Modo*):

- **Físico.** Las órdenes van al Arduino por USB. Solo se puede conectar en este modo.
- **Virtual.**
  - Se desactiva la conexión y se habilita *Simulación en Octave*. El recorrido puede ser el GOTO de los
    campos, los ensayos T1–T3, T4 A→B o la ida y vuelta.
  - La simulación usa el modo de control de «Ir a pose» (0 abierto, 1 ruedas, 2 pose), la carga del registro
    y los parámetros de la pestaña: ganancias, límites y η.
  - Ejecuta `agv/octave/correr_agv_desde_gui.m` y muestra la odometría, la referencia y el recorrido **real**,
    que solo existe en simulación porque incluye el deslizamiento. También muestra las ruedas, el PWM y el
    error final.
  - Guarda en `interfaz/ensayos/agv_virtual/<fecha>_<id>/`: config.json, resultado.csv, metricas.json y
    parametros_usados.json.
  - Para pasar a virtual hay que desconectar primero el robot.

| Campo | Unidad | Rango (defecto) | Orden enviada / cuándo rige |
|---|---|---|---|
| Velocidad lineal / angular manual | m/s, rad/s | 0,02–0,30 (0,10) / 0,1–2 (0,5) | `VEL` cada 100 ms mientras se mantiene el botón; al soltar, `VEL 0` |
| GOTO x, y, φ, duración, control | m, m, °, s | ±3, ±3, ±180, 0–60 | `GOTO,x,y,φ,T,modo`; el firmware alarga T si viola límites y lo informa en el ACK |
| Kp, Ki de rueda | PWM/(rad/s), PWM/rad | 0–50 (8), 0–500 (80) | `GAIN`: rige en el período siguiente al ACK (≤ 10 ms) |
| K de pose x, y, φ | 1/s | 0–10 (1,6; 1,6; 2,4) | `KPOSE`, ídem |
| vmax, amax, ω_máx, α_máx, ω de rueda máx. | SI | Topes del firmware | `LIM`: también recortan las órdenes manuales |
| η_x, η_y, η_ω | — | 0,5–1,5 (1) | `CAL`: calibración de la odometría |
| Telemetría | Hz | 1–50 (20) | `TEL` |
| Carga, piso, operador, notas | kg, texto | — | Se guardan en ensayo.json |
| Medición externa x, y, φ | m, m, ° | ±5, ±5, ±180 | Se agrega al último ensayo con el error de odometría. El error angular se envuelve con atan2, que respeta la periodicidad |

**Seguridad:**
- **STOP** (botón o tecla Esc): parada con rampa.
- **DIS:** motores libres.
- **CLR + ARM:** salir de una falla.
- Al cerrar la ventana se envía DIS. Si la PC se cuelga o se desconecta, el firmware para solo a los 300 ms.

**Rangos:**
- Los campos de la interfaz ya no admiten valores fuera de rango.
- El firmware vuelve a acotar cada valor, así que un dato inválido no llega al robot.
- `GEO` y `SIGN` solo se aceptan con el robot detenido. Se configuran desde la consola (`agv/pc/agv_consola.py`)
  durante la calibración.

## Verificación realizada (07/11)

- **Arranque:** la ventana se abre sin errores con Python 3.13 y PySide6 6.12.
- **Brazo, de punta a punta:** se simuló un ensayo PD+g con pulso desde la interfaz (52 s en Octave). Se
  comprobaron la barra de progreso, los gráficos, la tabla y los archivos.
- **AGV, sin hardware:** se probó la interpretación de `$TEL`/`$PAR`/`$EVT` con telemetría sintética, el
  registro CSV/JSON y el cálculo del error con medición externa. Esas carpetas de prueba se borraron.
- **AGV virtual, de punta a punta:** T4 A→B con modo 2 y η calibrados dio 4,9 mm en 13,7 s, igual que
  `ensayos_agv.m`.
- **Falta con el kit:** conexión real por USB, latido, movimiento manual, GOTO y comportamiento del watchdog.
