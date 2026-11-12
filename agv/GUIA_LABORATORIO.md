# Guía de laboratorio — AGV OSOYOO

Dos sesiones de unas 2 h. La **sesión 1** deja el robot cableado, con los signos correctos e identificado.
La **sesión 2** hace los 5 ensayos del enunciado (§5.5) con medición externa y video. Los detalles de cada
orden están en `LEEME_FIRMWARE.md`. Todo se corre **desde la carpeta del proyecto**.

> **Seguridad.**
> - El kit no sale de la Facultad. Ante una falla, ruido raro o calentamiento: `DIS`, apagar la batería e
>   informar al docente.
> - Parar: escribir `DIS`, o `Ctrl+C` en la consola (manda `DIS`). Si se corta la comunicación, el robot
>   frena solo a los 300 ms.
> - Tener siempre una mano cerca de la llave de la batería.
> - Después de una FALLA: `CLR` y luego `ARM`.

## Qué llevar

- [ ] Notebook con el proyecto, Arduino IDE, Python (`pyserial` ya instalado) y Octave. Cable USB A‑B.
- [ ] Batería cargada (y el cargador). Multímetro para medir la tensión.
- [ ] Cinta de papel, marcador fino, cinta métrica de 2–3 m, escuadra, calibre.
- [ ] Una plomada: hilo con una tuerca.
- [ ] Balanza (sirve una de cocina) y una carga de ≈ 0,5 kg, por ejemplo una botella de agua.
- [ ] Una caja para apoyar el robot con las ruedas en el aire.
- [ ] Celular con algo para sostenerlo alto (trípode, silla o estante).

## Preparar el piso (una vez; dejarlo marcado si se puede)

Zona libre de al menos 1,8 × 1,2 m, piso liso.
1. **A** = origen: una cruz de cinta.
2. **Eje x**: una línea de cinta de 1,5 m desde A, con marcas cada 100 mm.
3. **Eje y**: una línea de 0,8 m perpendicular en A, hacia la izquierda del robot (+y), con marcas cada 100 mm. Usar la escuadra.
4. **B** = (1200, 600) mm: otra cruz.
5. El robot arranca con **su centro sobre A, mirando hacia +x**.

**Marcas en el robot.** Dos puntos sobre la placa superior, en el eje longitudinal del robot:
- **D** delante y **T** detrás, a la **misma distancia del centro de las ruedas** (≈ 75 mm cada uno);
- el centro de las ruedas es el punto medio entre los ejes delantero y trasero, y entre las ruedas
  izquierda y derecha.

Después de cada corrida se baja cada punto al piso con la plomada, se marca con lápiz y se miden sus
coordenadas (x, y) en mm desde A. La consola calcula sola el centro y la orientación
(`!medir`: pide `x_D y_D x_T y_T`).

---

## Sesión 1 — puesta en marcha e identificación

**1. Cargar el firmware** (batería apagada, Mega por USB).
- Arduino IDE → abrir `agv/firmware/agv_osoyoo/agv_osoyoo.ino`.
- Placa **Arduino Mega or Mega 2560**, el puerto COM que aparezca, **Subir**.
- Cerrar el Monitor Serie del IDE.

**2. Abrir la consola** (el COM puede ser otro):
```bash
py -3.13 agv/pc/agv_consola.py COM8 --etiqueta s1_puesta_en_marcha
```
Cada sesión de la consola guarda telemetría, órdenes y notas en `agv/datos_lab/<fecha>_<etiqueta>/`.

**3. Relevamiento** (sin motores). Anotar todo en la hoja y con `!nota`:
- [ ] Fotos del kit, de la etiqueta de los motores y del driver.
- [ ] Masa del robot con batería.
- [ ] **2·l_x**: distancia entre ejes delantero y trasero, centro a centro.
- [ ] **2·l_y**: distancia entre ruedas izquierda y derecha, centro del ancho de rueda.
- [ ] Diámetro de rueda.
- [ ] Rodillos vistos desde arriba: ¿forman una **X** o una **O**? (foto)

**4. Encoders** (batería apagada).
- Cablear V → 5 V, G → GND y S1/S2 según la tabla de `LEEME_FIRMWARE.md` §4.B.
- Girar cada rueda a mano, despacio, enviando `HALL`: los bits A/B tienen que alternar y el contador moverse.
- [ ] Las 4 ruedas responden.

**5. Cuentas por vuelta y signo.**
- `ZERO`, marcar la rueda con cinta y darle **10 vueltas hacia adelante**, luego `HALL`.
  CPR = cuentas/10 (se espera ≈ 3300).
- Si sale negativa, esa rueda va en la máscara de encoder: 1 = FL, 2 = FR, 4 = RL, 8 = RR, sumadas.
- Cargar `CPR c0 c1 c2 c3` y `SIGN 0 <máscara_enc>`.
- [ ] CPR medidos: ____ ____ ____ ____

**6. Sentido de los motores** (robot **sobre la caja**, batería encendida):
```text
!guion agv/pc/guiones/sentidos_ruedas.txt
```
- Si una rueda gira hacia atrás, su bit va en la máscara de motor: `SIGN <máscara_mot> <máscara_enc>`.
- Repetir hasta que las 4 giren hacia adelante **con ω_med positiva**. Después `SAVE`.

**7. Identificación PWM → velocidad** (sobre la caja).
- `!nota bateria 7.9 V`, con lo que marque el multímetro.
- Correr `!guion agv/pc/guiones/identificacion_escalones.txt` (≈ 50 s) y cerrar la consola con `!salir`.
- Ajustar el modelo (cambiar `<carpeta>` por la que se acaba de crear):
  ```bash
  octave-cli --eval "cd agv/octave; identificar_lab('../datos_lab/<carpeta>')"
  ```
- Abrir de nuevo la consola y pegar las 4 órdenes `MDL …` que imprime Octave. Después `SAVE`.

**8. PI de ruedas** (sobre la caja):
```text
!guion agv/pc/guiones/pi_ruedas_elevado.txt
```
ω_med tiene que seguir a ω_ref sin oscilar. Si oscila: `GAIN 6 50` y repetir.

**9. Primeros movimientos en el piso** (despacio):
- `LIM 0.10 0.2 0.5 1 6`, `ZERO`, `ARM`.
- `GOTO 0.2 0 0 0 1` → avanza 20 cm.
- `GOTO 0.2 0.2 0 0 1` → va 20 cm hacia **su izquierda**.
  Si va a la derecha, son rodillos en O: `GEO 0.04 <lx> <ly> -1` y repetir.
- `GOTO 0.2 0.2 0.5 0 1` → gira **antihorario** visto desde arriba.

**10. Geometría medida:** `GEO <r> <lx> <ly> 1` (r = diámetro/2, lx y ly = mitades, en metros). Después `SAVE`.

Si sobra tiempo, seguir con T1 de la sesión 2.

---

## Sesión 2 — ensayos (§5.5)

> **Cómo se hizo finalmente (11/11/2026): por WiFi y con `corrida.py`.**
> - **Corrida:** `py -3.13 agv/pc/corrida.py <T1|T2|T3|T4|T4C|T5|T5S|T5A> <modo> <rep>` hace CLR, ZERO,
>   ARM, el GOTO y DIS, y guarda la telemetría en `agv/datos_lab/<fecha>_<ensayo>_m<modo>_r<rep>/`.
> - **Medición:** las baldosas de 30 × 30 cm sirven de cuadrícula y se mide con la app de medición del
>   celular. Se carga con `py -3.13 agv/pc/corrida.py --medir <ens> <modo> <rep> xD yD xT yT` (mm), que agrega
>   la fila a `agv/datos_lab/mediciones_piso.csv`.
> - **Opciones,** solo durante la corrida: `--amax 0.15` (rampa), `--wd 1500` (red WiFi congestionada),
>   `--sin-bat` (lectura de A5 inválida; medir con tester) y `--puerto COM5` (por USB).
> - **Corridas fallidas:** renombrar la carpeta con `_INVALIDA_<motivo>` y repetir con el mismo número.
> - **Figuras y tabla:** `py -3.13 agv/python/figuras_piso.py`.
> - **No usar a la vez** la interfaz y los scripts por WiFi: comparten el puerto UDP 4211 de la PC.
>
> Lo que sigue es el procedimiento original con la consola y guiones, que también funciona.

Para cada ensayo hay un guion. Cada corrida sigue la misma secuencia:
1. el guion pide reubicar el robot en A (Enter para seguir);
2. pone la odometría en cero, arma y ejecuta el `GOTO`;
3. espera el aviso de fin;
4. pide la medición con plomada (`!medir`);
5. muestra real, odometría y deriva.

Modos del `GOTO`: **0** abierto (solo el modelo MDL), **1** PI de ruedas, **2** PI + lazo de pose.
Repeticiones: modo 0 ×2, modo 1 ×1, modo 2 ×3, para ver la variabilidad.

**Video:** el celular graba **toda la sesión** desde arriba o en diagonal, con la cuadrícula a la vista. Al
empezar cada corrida, decir en voz alta el nombre que muestra la pantalla (p. ej. «T1 modo 2 rep 3»).

| Paso | Orden | Duración aprox. | Corridas |
|---|---|---|---|
| T1 avance 1,0 m | `py -3.13 agv/pc/agv_consola.py COM8 --etiqueta T1 --guion agv/pc/guiones/piso_T1_avance.txt` | 8 s c/u | 6 |
| T2 lateral 0,6 m | `… --etiqueta T2 --guion agv/pc/guiones/piso_T2_lateral.txt` | 5 s c/u | 6 |
| T3 giro 90° | `… --etiqueta T3 --guion agv/pc/guiones/piso_T3_giro.txt` | 3 s c/u | 6 |
| **Calibración** | `py -3.13 agv/pc/calcular_cal.py agv/datos_lab/<T1> agv/datos_lab/<T2> agv/datos_lab/<T3>` → cargar en la consola la orden `CAL …` que imprime, y `SAVE` | — | — |
| T4 A → B | `… --etiqueta T4 --guion agv/pc/guiones/piso_T4_AB.txt` (pone `CAL 1 1 1`) | 14 s c/u | 6 |
| T4C A → B calibrado | primero volver a cargar `CAL …` y después `… --etiqueta T4C --guion agv/pc/guiones/piso_T4C_AB_calibrada.txt` | 14 s c/u | 3 |
| T5 A → B con carga | pesar la carga, sujetarla al centro de la placa, `!nota carga 0.5 kg`; `… --etiqueta T5 --guion agv/pc/guiones/piso_T5_AB_carga.txt` | 14 s c/u | 4 |

- Antes de cada bloque: `!nota bateria <V>`. Si baja de ≈ 7,0 V, cargar la batería, porque cambia la
  zona muerta.
- Anotar el tipo de piso y cualquier cosa rara: un patinazo, un roce, una rueda floja.

---

## Problemas frecuentes

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| No aparece el puerto COM | Driver USB (CH340) o cable solo de carga | Administrador de dispositivos; probar otro cable |
| `FALLA` apenas arranca una rueda | Encoder desconectado (traba) | `HALL` girando a mano; revisar S1/S2 y alimentación |
| `FALLA` por sobrevelocidad en modo 1 o 2 | Signo de encoder invertido respecto del motor | Repetir los pasos 5 y 6 |
| Una rueda no gira con PWM bajo | Zona muerta | Normal; el modelo MDL la compensa |
| El lateral sale al revés | Rodillos en O | `GEO … -1` |
| Se detiene solo a mitad de camino | Watchdog: la consola se colgó o el cable USB tironea | Revisar el cable; no usar el Monitor Serie del IDE a la vez |
| Por WiFi no aparece el robot | Robot apagado o ESP sin firmware AT, o la interfaz abierta ocupando el puerto 4211 | `ping` a su IP; por USB mirar `$WIFI,…` al arrancar; cerrar la interfaz |
| `PING` por USB responde `ERROR` | En la Mega quedó un puente al ESP (otro grupo grabó el ESP) | Volver a flashear con el Loader |
| Falla de batería con la batería cargada | Cable del divisor de A5 suelto | Revisar el cable; mientras tanto `--sin-bat` o la casilla de la interfaz, y tester |
| Falla por traba al llegar a un punto | Las ruedas se pegan corrigiendo menos de 1° | Normal en el piso; `corrida.py` usa traba de 2,5 s en modo 2 |

## Qué hay que traer de vuelta

- La carpeta **`agv/datos_lab/`** completa: cada sesión tiene `telemetria.csv`, `eventos.csv` y `ensayo.json` con notas y mediciones.
- Fotos y videos, con el nombre del ensayo.
- La hoja con medidas, masas, CPR, máscaras de signo y tensiones de batería.

Con eso armo la comparación física vs simulación (mismas referencias T1–T5), las figuras y la sección de
ensayos del informe.
