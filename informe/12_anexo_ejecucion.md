### A.0 Repositorio del proyecto

Todo el proyecto está publicado en GitHub. Incluye este informe, el código, los modelos, el firmware, la
interfaz, la escena de CoppeliaSim y los datos de los ensayos:

**<https://github.com/Arrualuciano/Proyecto-Final-Integrador-Robotica-II-RodriguezArrua>**

- La organización de las carpetas está en el `README.md` de la raíz y en la sección A.3.
- **No se publican:**
  - los proyectos de otros grupos que se usaron solo como referencia;
  - `agv/firmware/agv_osoyoo/wifi_config.h`, que tiene la clave de la red del laboratorio. Para compilar se
    copia `wifi_config.ejemplo.h` como `wifi_config.h` y se completa.

### A.1 Software y versiones

| Herramienta | Versión usada | Para qué |
|---|---|---|
| GNU Octave | 10.3.0 (`octave-cli`) | Modelos, trayectorias, control y simulación del AGV |
| Python | 3.13, con numpy, scipy y matplotlib | Interfaz, figuras, CoppeliaSim, consola de laboratorio |
| Paquetes de Python | PySide6 6.12, pyqtgraph 0.14, pyserial 3.5, coppeliasim‑zmqremoteapi‑client 2.0.4, trimesh, manifold3d, imageio e imageio‑ffmpeg | `py -3.13 -m pip install PySide6 pyqtgraph pyserial coppeliasim-zmqremoteapi-client trimesh manifold3d markdown imageio imageio-ffmpeg` |
| CoppeliaSim Edu | 4.10.0 | Entorno gráfico |
| Autodesk Fusion | versión en la nube (nov. 2026) | CAD paramétrico (script `fusion_brazo`) |
| Arduino IDE / arduino‑cli | 2.x / 1.2, núcleo AVR 1.8.6 | Firmware para la Arduino Mega 2560 |

### A.2 Orden de ejecución (desde la carpeta del proyecto)

Todos los parámetros están en `parametros/parametros.json`. Cada paso lee lo que produjo el anterior.

**1. CAD** (solo si cambia la geometría). En Fusion, `Shift+S` → `fusion_brazo` → Ejecutar. Escribe
`brazo/stl/` y `brazo/cad/propiedades_fusion.csv`.

**2. Cinemática y dinámica del brazo.**

```bash
octave-cli --eval "cd brazo/octave; verificar_cinematica; verificar_dinamica; dimensionamiento"
```

**3. Trayectorias.**

```bash
octave-cli --eval "cd brazo/octave; generar_ciclo_brazo; verificar_trayectoria_brazo"
py -3.13 brazo/python/figuras_ciclo.py
```

**4. Control.** Cada ensayo tarda unos 4 minutos, así que se corren por separado con la variable
`AFR_ENSAYOS` = 1, 2, 3, N o I.

```bash
octave-cli --eval "setenv('AFR_ENSAYOS','1'); cd brazo/octave; ensayos_control_brazo"
py -3.13 brazo/python/figuras_control.py
```

**5. Verificación estructural.**

```bash
octave-cli --eval "cd brazo/octave; verificacion_estructural"
py -3.13 brazo/python/figuras_estructural.py
py -3.13 brazo/python/division_L2.py
```

**6. AGV en simulación.**

```bash
octave-cli --eval "cd agv/octave; verificar_cinematica_agv; identificacion_simulada; ensayos_agv"
py -3.13 agv/python/figuras_agv.py
py -3.13 agv/python/figuras_piso.py
```

`figuras_piso.py` compara los ensayos físicos (`agv/datos_lab/mediciones_piso.csv`) con la simulación.

**7. CoppeliaSim** (con CoppeliaSim abierto). Para solo verla, basta abrir `coppelia/celda_integrador.ttt` y
apretar Play.

```bash
octave-cli --eval "addpath('brazo/octave'); correr_desde_gui('coppelia/datos_brazo/config.json')"
py -3.13 coppelia/preparar_reproduccion.py
py -3.13 coppelia/construir_escena.py
py -3.13 coppelia/verificar_escena.py
py -3.13 coppelia/grabar_video.py
```

`grabar_video.py` graba el ciclo en `videos/simulacion_celda_coppelia.mp4` (requiere `imageio` e
`imageio-ffmpeg`).

**7 bis. CAD del AGV.** En Fusion, `Shift+S` → `fusion_agv` (carpeta `agv/cad/fusion_agv`) → Ejecutar.
Escribe `agv/cad/agv_osoyoo.step`, los STL y las propiedades.

**7 ter. Interferencias del brazo.**

```bash
py -3.13 brazo/python/interferencias_ciclo.py
```

**8. Interfaz.**

```bash
py -3.13 interfaz/robot_gui.py
```

**9. Firmware y laboratorio.**
- **Firmware** `agv/firmware/agv_osoyoo/` (versión final agv‑0.2.3). Se compila y se carga con el AFR2 Loader
  (`build_mega` y `flash_mega`, con aprobación en su ventana), que usa `agv/firmware/Makefile`. También se
  puede cargar con el Arduino IDE o compilar con `agv/firmware/compilar_hex.ps1`.
- **El flasheo borra la EEPROM** (fusible EESAVE sin programar). Por eso los valores medidos del kit y la
  calibración están también como valores por defecto del firmware.
- **Datos de la red WiFi:** van en `agv/firmware/agv_osoyoo/wifi_config.h`, que contiene la clave y no se
  distribuye. El ESP8266 tiene que tener el firmware AT de fábrica.
- **Laboratorio:** seguir `agv/GUIA_LABORATORIO.md`. Ensayos en el piso:

```bash
py -3.13 agv/pc/corrida.py T4C 2 1
py -3.13 agv/pc/corrida.py --medir T4C 2 1 xD yD xT yT
```

- **Prueba de la pausa por corte de comunicación,** con el robot elevado:

```bash
py -3.13 agv/pc/prueba_pausa_com.py udp
```

**10. Informe.** Genera `informe/informe_integrador.html` y `.pdf`.

```bash
py -3.13 informe/figuras_informe.py
py -3.13 informe/armar_informe.py
```

### A.3 Organización de los archivos

| Carpeta | Contenido |
|---|---|
| `parametros/` | Tabla única de parámetros, con unidades y procedencia |
| `brazo/cad/`, `brazo/stl/` | Script de Fusion, STL y 3MF, propiedades de masa |
| `brazo/octave/`, `brazo/python/`, `brazo/resultados/` | Modelos, trayectorias, control, verificación estructural; figuras y registros |
| `agv/firmware/`, `agv/pc/`, `agv/octave/`, `agv/python/`, `agv/resultados/`, `agv/datos_lab/` | Firmware, consola y guiones, modelos y simulación, figuras, datos de laboratorio |
| `coppelia/` | Escena, scripts de construcción, reproducción y verificación |
| `interfaz/` | Interfaz Qt y ensayos registrados |
| `agv/cad/` | Script de Fusion del AGV (`fusion_agv`) y, al ejecutarlo, STEP y STL |
| `videos/` | Video del ensayo con la interfaz por WiFi y video de la simulación en CoppeliaSim |
| `informe/` | Fuentes del informe (Markdown), figuras propias y armado del PDF |
| `referencias/` | Análisis de fuentes de terceros |

**Recursos de terceros.**
- El análisis del proyecto de un compañero (`referencias/analisis_proyecto_koba.md`) se usó como fuente de
  datos del kit: pinout y hoja del motor.
- El cargador AFR2 Loader, del mismo compañero, se usa sin cambios en los ejecutables. Solo se quitaron de
  su documentación las rutas absolutas de su computadora.
- Del firmware BioLog de Mendelevich y Baldoni, para el mismo kit, se tomaron dos ideas, reescritas en
  el código propio:
  - la configuración del ESP8266 con comandos AT desde la Mega, en UDP transparente (`wifi.cpp`);
  - la pausa ante cortes breves de comunicación en los GOTO.
- Los sentidos de los motores se contrastaron con ese mismo firmware.
- No se incorporó código de terceros a los modelos.

### A.4 Organización del trabajo

El trabajo se dividió en dos frentes, uno por integrante:

Tabla: Distribución de tareas

| Integrante | Responsable de | Comprende |
|---|---|---|
| Nicolás Rodriguez | La simulación | Brazo 6R (CAD, cinemática, dinámica, trayectorias, control y verificación estructural), modelo y simulación del AGV en Octave y escena de CoppeliaSim |
| Luciano Arrua | El kit OSOYOO | Relevamiento del kit, firmware, puesta en marcha, identificación, enlace WiFi y ensayos físicos en el laboratorio |

Cada uno se ocupó de su frente, y las dudas que surgían se resolvieron entre los dos. El registro de
decisiones está en `00_Aplicacion_y_plan.md` (sección 7, con el plan de trabajo en la 8) y las versiones
quedan en el historial del repositorio.
