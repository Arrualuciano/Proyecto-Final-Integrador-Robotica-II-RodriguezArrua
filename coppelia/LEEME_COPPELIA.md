# Escena de CoppeliaSim — celda AGV + brazo 6R

Escena: `celda_integrador.ttt` (CoppeliaSim Edu 4.10.0). Reproduce **un ciclo completo de la celda**:
el AGV lleva la pieza de la estación A a la B, el brazo la toma de la bandeja del AGV y la deja en la
bandeja C de la mesa.

## Cómo verla

1. Abrir CoppeliaSim → *File → Open scene…* → `coppelia/celda_integrador.ttt`.
2. Play (▶). La animación dura 26,8 s y queda en pausa al final. Stop (■) vuelve al estado inicial.

`reproduccion.csv` tiene que estar **en la misma carpeta que el .ttt** (el script lo busca ahí).

## Qué se muestra (y qué no)

| Tramo | Tiempo | Origen de los datos |
|---|---|---|
| Espera | 0 – 1 s | — |
| AGV A → B con la pieza | 1 – 15,2 s | Pose **real** del AGV simulado en Octave (ensayo T4, control de pose + odometría calibrada; llega a (1,2000; 0,5951) m) |
| Espera | 15,2 – 15,7 s | — |
| Ciclo del brazo (tomar → colocar → volver) | 15,7 – 26,8 s | q **real** del brazo con el controlador par calculado + integral (caso nominal, error máx. 0,46 mm) |

- Las juntas J1…J6 están en **modo cinemático**: CoppeliaSim no integra dinámica ni usa sus
  controladores internos; se fija la posición articular leída del CSV en cada paso (20 ms).
- Todas las formas son **estáticas y no respondientes**: no interviene el motor de física.
- La dinámica y el control se resuelven en Octave (`brazo/octave/simular_control_brazo.m`,
  `agv/octave/simular_agv.m`); CoppeliaSim solo visualiza. Así lo que se ve es exactamente lo que
  se calculó, con trazabilidad completa (decisión registrada en `00_Aplicacion_y_plan.md` §7).
- La pieza viaja sobre el AGV hasta que la pinza se cierra (tramo 4 del ciclo), sigue al TCP
  hasta que se abre (tramo 8) y queda apoyada en la bandeja C.

## Geometría

| Elemento | Origen | Ubicación en la celda {G} |
|---|---|---|
| Brazo 6R | STL exportados por el script de Fusion (`brazo/stl`, mm, pose cero, marco {0}); cadena L0 → J1 → L1 → … → J6 → L6 con las juntas en las tramas DH | Base en (1,2; 0,9; 0,09) m, girada −90° (pedestal a la altura de la bandeja del AGV) |
| AGV OSOYOO | Primitivas con las medidas de `parametros.json` (placas 200 × 155 mm, ruedas Ø80 × 36 mm en (±lx, ±ly), motores, columnas, Mega2560) | Parte de A = (0, 0), llega a B = (1,2; 0,6) |
| Pieza | Cubo 40 mm | Sobre la bandeja del AGV |
| Mesa + bandeja C | Prismas | (1,5; 0,9) m |

Los puntos de toma (0,30; 0; 0,02) y colocación (0; 0,30; 0,02) de `parametros.json`, expresados en
{0}, caen en B y en la bandeja C respectivamente (`escena_config.py`).

## Cómo regenerarla (si cambian parámetros, trayectorias o resultados)

Desde la carpeta del proyecto, con CoppeliaSim abierto (el servidor ZMQ arranca solo):

```bash
octave-cli --eval "addpath('brazo/octave'); correr_desde_gui('coppelia/datos_brazo/config.json')"
```
```bash
py -3.13 coppelia/preparar_reproduccion.py
```
```bash
py -3.13 coppelia/construir_escena.py
```
```bash
py -3.13 coppelia/verificar_escena.py
```

1. Corre el lazo de control del brazo con `coppelia/datos_brazo/config.json` → `resultado.csv`.
2. Junta el AGV (`agv/resultados/ensayos/T4_E4.csv`) y el brazo en `reproduccion.csv` (50 Hz).
3. Reconstruye la escena desde cero y la guarda en `celda_integrador.ttt` (borra la escena abierta).
4. Verifica la escena abierta en modo paso a paso y guarda capturas en `coppelia/capturas/`.

## Verificación (07/11/2026)

```
t =  0.50 s | max|q_escena − q_csv| = 0.0e+00 rad | AGV (0.000, 0.000) m | pieza (0.000, 0.000, 0.110)
t =  8.00 s | max|q_escena − q_csv| = 0.0e+00 rad | AGV (1.107, 0.000) m | pieza (1.107, 0.000, 0.110)
t = 15.40 s | max|q_escena − q_csv| = 8.9e-14 rad | AGV (1.200, 0.595) m | pieza (1.200, 0.595, 0.110)
t = 19.50 s | max|q_escena − q_csv| = 1.2e-13 rad | AGV (1.200, 0.595) m | pieza (1.213, 0.600, 0.190) | |TCP − pieza| = 0.0 mm
t = 26.60 s | max|q_escena − q_csv| = 0.0e+00 rad | pieza (1.500, 0.900, 0.110)  → en la bandeja C
```

- Las juntas de la escena coinciden con el CSV a precisión de máquina.
- Con la pieza en la pinza, el TCP calculado con la MCD de Python a partir de las juntas **leídas de
  la escena** coincide con la pieza: la cadena de CoppeliaSim y el modelo DH son consistentes.

## Archivos

| Archivo | Función |
|---|---|
| `escena_config.py` | Ubicación de la base, altura de bandeja, MCD en Python, utilidades |
| `datos_brazo/` | Configuración y resultado del lazo de control usado en la escena |
| `preparar_reproduccion.py` | Arma `reproduccion.csv` (t, q1..q6, pose AGV, pose pieza, fase) |
| `reproduccion.lua` | Script de simulación embebido en la escena (lee el CSV y mueve juntas, AGV y pieza) |
| `construir_escena.py` | Construye y guarda la escena por la API remota ZMQ |
| `verificar_escena.py` | Verificación numérica y capturas |
| `capturas/*.png` | Imágenes de la escena para el informe |

Nota técnica: para el modo paso a paso con el cliente ZMQ 2.x hay que usar `client.setStepping(True)`
y `client.step()`; llamar `sim.setStepping`/`sim.step` desde el cliente deja a CoppeliaSim bloqueado.
