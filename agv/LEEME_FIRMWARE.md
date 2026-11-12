# AGV OSOYOO — firmware, consola y primer laboratorio

Firmware: `agv/firmware/agv_osoyoo/` (Arduino IDE, placa **Arduino Mega or Mega 2560**). Compilado con
arduino-cli 1.2 / AVR core 1.8.6: 36 KB de flash (14 %) y 2,4 KB de RAM (29 %), sin advertencias en el
código propio. **Versión cargada en el kit: agv-0.2.3** (11/11/2026, con el AFR2 Loader por COM5), operada por
WiFi. Puesta en marcha, identificación y ensayos hechos; historial y cuidados en la sección 5.

## 1. Arquitectura

| Archivo | Qué hace |
|---|---|
| `config.h` | Pines (orden lógico FL, FR, RL, RR), tiempos y opciones de hardware |
| `hal.cpp` | PWM a 3,9 kHz (Timer1/2), encoders x4 por PCINT2 (puerto K, una sola ISR para las 8 señales), batería en A5 |
| `nucleo.cpp` | **Sin Arduino.** Cinemática mecanum, odometría, PI de ruedas con prealimentación, trayectoria quíntica, lazo de pose, estados y protecciones |
| `protocolo.cpp` | Órdenes de texto, respuestas, telemetría en una cola no bloqueante y EEPROM |
| `agv_osoyoo.ino` | Lazo fijo de 10 ms con `micros()`, watchdog de comunicación y de hardware |

**Tiempos:**
- Ruedas, odometría y protecciones: **10 ms (100 Hz)**.
- Lazo de pose: 20 ms (50 Hz).
- Telemetría: 0–50 Hz (por defecto 20 Hz).
- Watchdog de comunicación: 300 ms. Watchdog de hardware: 250 ms.

**Estados:** DESARMADO → `ARM` → QUIETO → (PWM | PASO | VEL | IR_A) → QUIETO.
- Cualquier protección lleva a FALLA, con los motores en cero.
- Para salir de FALLA hay que enviar `CLR` y después `ARM`.

**Protecciones:**
- Sin mensajes de la PC durante más de `wd_ms` con el robot armado.
- Rueda trabada o encoder desconectado: PWM ≥ zona muerta + 30 sin cuentas durante 500 ms.
- Sobrevelocidad > 1,5·ω_máx en lazo cerrado. Es lo que pasa típicamente con un encoder de signo invertido.
- Batería baja (solo con `SAFE … bat_check=1`, cuando el divisor esté instalado).
- Período excedido.

## 2. Órdenes

Formato `$CMD,a,b*HH` (la consola lo arma sola). Escritas a mano, sin `$` ni suma, también se aceptan:
`VEL 0.1 0 0`. Unidades SI. Ruedas: 0 = FL, 1 = FR, 2 = RL, 3 = RR, 4 = todas.

| Orden | Efecto |
|---|---|
| `PING` | Latido (la consola lo manda cada 100 ms) → `PONG,versión,t,modo,falla` |
| `ARM` / `DIS` / `STOP` / `CLR` | Armar; desarmar (motores libres); detener con rampa; limpiar falla |
| `PWM u0 u1 u2 u3` | Lazo abierto por rueda (−255..255). Vale `ttl_ms` (500 ms); hay que repetirlo |
| `STEP i u ms` | Escalón de PWM `u` durante `ms` en la rueda `i` (4 = todas): identificación |
| `VEL vx vy wz` | Velocidad del cuerpo (m/s, m/s, rad/s), con rampa y PI de ruedas. Vale `ttl_ms` |
| `GOTO x y phi T modo` | Quíntico hasta la pose en {G}. modo 0 = abierto (solo prealimentación), 1 = PI de ruedas, 2 = PI + lazo de pose. Responde la duración efectiva (la alarga si viola los límites) y al terminar envía `EVT,FIN,ex,ey,ephi` |
| `POSE x y phi` / `ZERO` | Fijar la odometría / poner en cero cuentas y pose |
| `GAIN kp ki` | PI de ruedas [PWM/(rad/s), PWM/rad] |
| `KPOSE kx ky kphi` | Ganancias del lazo de pose [1/s] |
| `LIM vmax amax wmax alfamax wrueda` | Límites del cuerpo y de cada rueda |
| `GEO r lx ly sy` | Geometría [m]; `sy = −1` invierte el eje lateral (si los rodillos resultan en patrón O) |
| `CPR c0 c1 c2 c3` | Cuentas x4 por vuelta de cada rueda |
| `SIGN mot enc` | Máscaras de inversión (bit0 = FL, bit1 = FR, bit2 = RL, bit3 = RR) |
| `MDL i K udz` | Modelo identificado de la rueda `i` (4 = todas): ω = K(u − u_dz) |
| `CAL ex ey ew` | Factores de calibración de la odometría (real/odometría) |
| `FIL a` | Filtro de la velocidad medida (1 = sin filtro) |
| `SAFE ttl wd traba bat_check bat_min` | Seguridad. `wd = 0` desactiva el watchdog: solo para pruebas a mano en el Monitor Serie |
| `TEL hz` | Frecuencia de telemetría (0 apaga; máximo 50) |
| `HALL` | Niveles A/B crudos de las 4 ruedas, contadores y errores (cableado) |
| `GET` / `SAVE` / `LOAD` / `DEF` | Ver / guardar en EEPROM / recargar / volver a los valores por defecto |

**Telemetría `$TEL`** (33 campos): t_ms, modo, falla, x, y, φ, x_ref, y_ref, φ_ref, v_x, v_y, ω_z,
ω_ref[4], ω_med[4], u[4], cuentas[4], V_bat, tiempo de ejecución, peor período, errores de Hall, líneas
perdidas. Unidades: mm, mrad, mm/s, mrad/s, PWM, cuentas, mV, µs.

## 3. Cargar el firmware y abrir la consola

1. Instalar pyserial una sola vez:
   ```bash
   py -3.13 -m pip install pyserial
   ```
2. **Batería apagada.** Conectar la Mega por USB. En el Arduino IDE: abrir `agv_osoyoo.ino`, elegir la placa
   Mega 2560 y el puerto, y cargar.
3. Cerrar el Monitor Serie del IDE y abrir la consola:
   ```bash
   py -3.13 agv/pc/agv_consola.py COM8 --etiqueta prueba_cableado
   ```
   El COM puede ser otro; se ve en el IDE. La consola guarda todo en `agv/datos_lab/<fecha>_<etiqueta>/`.

### Alternativa: compilar y cargar con el AFR2 Loader (programa de Koba Chajchir)

1. Compilar a una ruta fija:
   ```bash
   powershell -ExecutionPolicy Bypass -File agv\firmware\compilar_hex.ps1
   ```
   Deja `agv/firmware/build/agv_osoyoo.ino.hex`.
2. En el Loader 2.0.0 (`AFR2-Loader-dist-2.0.0/…/Iniciar.cmd`): **Configuración → Importar** y elegir
   `agv/firmware/perfil-loader-agv.json`. Después, **Guardar perfil**: queda en la biblioteca local
   (`%LOCALAPPDATA%\AutoFinalRobo2Loader`).
3. El perfil ya trae las rutas de esta PC, comprobadas el 07/11/2026:
   - **Repo**: la carpeta del proyecto (ruta absoluta). Si se mueve el proyecto o se usa otra PC, es el
     único campo a cambiar;
   - avrdude, avrdude.conf y avr‑gcc/ar/objcopy: tomados de `%LOCALAPPDATA%\Arduino15` (Arduino IDE 2);
   - Python 3.13.
4. **Compilar desde el Loader** (*Compilar ATmega*, o `build_mega` por MCP). `AtmegaDir` apunta a
   `{Repo}\agv\firmware`, que tiene un `Makefile` adaptador. El Loader corre una sola llamada,
   `make -B firmware communication-firmware library CC=… AR=… OBJCOPY=…`:
   - `firmware` compila con arduino-cli (`compilar_hex.ps1`) y copia el HEX a `build/afr2.hex`;
   - `communication-firmware` deja `build/afr2-communication.hex` como copia del mismo HEX, porque el
     protocolo está en el mismo sketch;
   - `library` genera `libafr2_ui.a` vacío, en la raíz de `agv/firmware`, para la comprobación de artefactos
     del Loader.

   Probado el 07/11/2026 desde el Loader: «Compilación ATmega completada», `afr2.hex` de 101 067 bytes,
   SHA‑256 `d9c5b7cd…`.
6. **Flashear desde el Loader** (`flash_mega`, puerto de la Mega: **COM5** en esta PC). El Loader solo carga la
   imagen que registró al compilar en la misma sesión, por eso el perfil usa `Hex = {AtmegaDir}\build\afr2.hex`:
   siempre compilar y después flashear, sin cambiar fuentes ni perfil en el medio. Primera carga: 07/11/2026
   22:20, firma 0x1e9801, 35 928 bytes escritos y verificados, fusibles sin cambios (E:FF H:D8 L:FF).
   Los avisos finales del Loader sobre «Hall conectado» y el cableado D18/D19 del ESP son del firmware de
   Koba: no aplican al nuestro.

   `EspDir` y `WebDir` no se usan.
5. El COM se elige en la ventana del Loader, en el momento de cargar. No se guarda en el perfil.

**En esta PC ya está cargado** (07/11/2026): el perfil activo
`%LOCALAPPDATA%\AutoFinalRobo2Loader\profiles\default.json` tiene Repo, HEX, Python 3.13, avrdude y avr‑gcc.
Se configuró por MCP (`loader_request_profile`, aprobado en la ventana) y se verificó con `loader_profile`:
todas las rutas existen.

## 4. Primer laboratorio — orden recomendado

> **Para ir al laboratorio usar `agv/GUIA_LABORATORIO.md`**: checklist de las dos sesiones, preparación
> del piso, guiones de los ensayos T1–T5 (`agv/pc/guiones/piso_*.txt`) y calibración de la odometría
> (`agv/pc/calcular_cal.py`). La consola tiene además `!esperar_fin`, `!pausa` y `!medir` (medición
> externa con plomada, que se guarda en `ensayo.json`). Esta sección queda como referencia detallada.

> El kit no puede salir de la Facultad. Ante cualquier falla, ruido o calentamiento: `DIS`, apagar la batería
> e informar al docente.

**A. Relevamiento, sin energizar motores.**
1. Fotos del kit, la etiqueta de los motores y el driver.
2. Pesar el robot con batería. Medir con calibre o regla:
   - **2·l_x**, la distancia entre los ejes delantero y trasero (centro a centro de las ruedas);
   - **2·l_y**, la distancia entre las ruedas izquierda y derecha (centro a centro del ancho de rueda);
   - el diámetro de las ruedas.
3. Mirar las ruedas desde arriba y anotar si los rodillos forman una **X** o una **O** (foto).

**B. Cableado de encoders.**
1. Identificar en cada plaquita Hall **V/G/S1/S2**. Ojo: hay variantes con blanco y amarillo invertidos.
   **No adivinar: medir continuidad o leer la serigrafía.**
2. Conectar VCC → +5 V y GND → GND de la Mega. S1 (A) y S2 (B):

| Rueda | S1 (A) | S2 (B) |
|---|---|---|
| FL delantera izq. | A9 | A13 |
| FR delantera der. | A8 | A12 |
| RL trasera izq. | A11 | A15 |
| RR trasera der. | A10 | A14 |

3. Con la consola abierta y la batería apagada, enviar `HALL` varias veces mientras se gira **a mano y despacio**
   cada rueda. Los dos bits A/B de esa rueda tienen que alternar (00 → 01 → 11 → 10) y su contador moverse.
   - Si los bits no cambian nunca o quedan fijos en 0, revisar la alimentación. Probar `HALL_PULLUPS` en 1 o 0
     en `config.h` y volver a cargar.
   - Si `errores` crece mucho al girar despacio, hay ruido o un canal flojo.

**C. Cuentas por vuelta y signo del encoder** (batería apagada).
1. `ZERO`. Marcar la rueda con cinta y girarla **10 vueltas exactas hacia adelante** (como si el robot avanzara).
2. Enviar `HALL`. CPR = cuentas/10. Esperado ≈ 3300 si son 11 PPR con 75:1 y x4.
3. Si la cuenta salió **negativa**, esa rueda necesita el bit de signo de encoder.
4. Repetir con las 4 ruedas. Cargar los valores: `CPR c0 c1 c2 c3` y `SIGN 0 <máscara_enc>`.
   La máscara es la suma de 1 (FL), 2 (FR), 4 (RL) y 8 (RR) de las ruedas invertidas.

**D. Sentido de los motores** (**robot elevado sobre una caja, ruedas en el aire**, batería encendida).
1. `!guion agv/pc/guiones/sentidos_ruedas.txt`. Cada rueda gira 0,8 s con PWM +120.
2. Si una rueda gira **hacia atrás**, poner su bit en la máscara de motor: `SIGN <máscara_mot> <máscara_enc>`.
3. Repetir hasta que las cuatro giren hacia adelante **y** su ω_med sea positiva.
4. `SAVE`.

**E. Identificación** (robot elevado).
1. Anotar la tensión: `!nota bateria 8.1 V`.
2. Correr `!guion agv/pc/guiones/identificacion_escalones.txt`.
3. Con esos datos se ajustan K, u_dz y τ por rueda; yo preparo el script de ajuste.

**F. Primeras pruebas en el piso** (zona despejada y mano cerca para `DIS`).
1. `LIM 0.10 0.2 0.5 1 6` para empezar despacio.
2. `VEL 0.1 0 0` repetido (o mantener con la consola) → tiene que avanzar.
3. `VEL 0 0.1 0` → tiene que ir hacia **su izquierda**.
4. `VEL 0 0 0.5` → tiene que girar **antihorario** visto desde arriba.
5. Si el lateral sale invertido: `GEO 0.04 <lx> <ly> -1` (patrón O) y repetir.

**G. Geometría medida.** `GEO r lx ly 1` con los valores medidos y `SAVE`.

## 5. Estado al 11/11/2026 (versión final agv-0.2.3)

**Historial de versiones**

| Versión | Cambio |
|---|---|
| 0.1.1 | `wmax` y `alfamax` por defecto (en la 0.1.0 un comentario los dejaba en 0) |
| 0.1.2 | `SAVE` escribe la EEPROM byte a byte alimentando el watchdog (antes reiniciaba la placa) |
| 0.2.0 | Enlace WiFi: ESP8266 del shield con firmware AT, Serial1 (D18/D19), UDP transparente. El robot manda `$HOLA,WIFI,<ip>,agv` cada 1 s hasta recibir la primera orden por WiFi; las respuestas salen por el enlace de la última orden |
| 0.2.1 | Valores por defecto = parámetros medidos (CPR 1482, `SIGN 15 5`, l_x/l_y, K y zona muerta en el piso, `SAFE 500 600 1500 1 6400`) |
| 0.2.2 | Pausa ante cortes de comunicación en GOTO: congela la trayectoria y frena; falla `F_COM` recién a los 3 s (`WD_PAUSA_MAX_MS`). Evento `EVT,PAUSA_COM,1/0` |
| 0.2.3 | Calibración del piso η = (1,029; 0,805; 0,995) como valor por defecto |

**Hecho:** identificación (`MDL`), PI verificado, ensayos T1–T5 en el piso por WiFi (`agv/pc/corrida.py`),
calibración, tiempos medidos (ejecución ≤ 5,3 ms, peor período ≤ 13,3 ms) y prueba de la pausa
(`agv/pc/prueba_pausa_com.py`). Resultados: informe, sección 8.B.

**Cuidados**
- **El flasheo con el Loader borra la EEPROM** (fusible EESAVE sin programar). Los valores medidos ya
  están como valores por defecto; si se cambia algo con `SAVE`, después de cada flasheo hay que volver a
  cargarlo (por ejemplo, `guiones/calibracion_piso.txt`).
- **Si otro grupo usa el kit:**
  - verificar que la Mega tenga este firmware: `PING` responde `PONG,agv-0.2.3`; si responde `ERROR`, hay
    un puente al ESP cargado;
  - verificar que el ESP tenga el firmware AT: al arrancar debe aparecer `$WIFI,ok,red …` por USB; si da
    `sin respuesta` en todas las velocidades, el ESP tiene otro firmware.
- **`wifi_config.h` tiene la clave de la red:** no se comparte ni se sube.
- **Lectura de batería (divisor 10k/10k en A5):** si da valores imposibles (~2 V con la batería bien),
  revisar el cable. Mientras tanto, usar `corrida.py --sin-bat` o la casilla de la interfaz y medir con
  tester.

**Pendiente (opcional):** parada de emergencia en D3; IMU para el ángulo en el lazo de pose.
