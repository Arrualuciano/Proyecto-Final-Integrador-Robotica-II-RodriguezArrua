# Trabajo integrador — Robótica II (FCAL‑UNER, 2026)

**Celda de abastecimiento de piezas con un AGV omnidireccional y un brazo antropomórfico 6R.**

Alumnos: Nicolás Rodriguez · Luciano Arrua — Docentes: Ing. Nahuel Medina · Patricio Silva

- Un **AGV OSOYOO con ruedas mecanum** (Arduino Mega 2560) lleva una pieza de la estación A a la B.
- Un **brazo 6R imprimible**, de diseño propio, la toma de la bandeja del AGV y la coloca girada 90°.
- Brazo: diseño, cálculo y simulación. AGV: modelo, firmware, control y **ensayos físicos en el piso**.

📄 **Informe completo:** [`informe/informe_integrador.pdf`](informe/informe_integrador.pdf)
· Presentación de la defensa: [`Defensa Integrador Robótica II.pptx`](Defensa%20Integrador%20Rob%C3%B3tica%20II.pptx)

## Organización

| Carpeta | Contenido |
|---|---|
| `parametros/` | Tabla única de parámetros (`parametros.json`), con unidades y procedencia |
| `brazo/` | CAD paramétrico de Fusion, STL, modelos en Octave (cinemática, dinámica, trayectorias, control, verificación estructural), figuras y resultados |
| `agv/firmware/` | Firmware `agv_osoyoo` (agv‑0.2.3) para la Arduino Mega, `Makefile` para el AFR2 Loader |
| `agv/pc/` | Consola de laboratorio, corridas de ensayo por WiFi (`corrida.py`), guiones y pruebas |
| `agv/octave/`, `agv/python/` | Modelo y simulación del AGV con el mismo algoritmo del firmware; figuras |
| `agv/datos_lab/` | Datos de los ensayos físicos (telemetría, eventos, mediciones externas) |
| `coppelia/` | Escena de CoppeliaSim y scripts para construirla, reproducirla y verificarla |
| `interfaz/` | Interfaz Qt (`robot_gui.py`): brazo virtual y AGV físico (WiFi/USB) o virtual |
| `informe/` | Fuentes del informe (Markdown) y armado del PDF |
| `presentacion/` | Imágenes de la presentación |
| `AFR2-Loader-dist-2.0.0/` | Cargador AFR2 Loader (programa de Koba Chajchir, usado sin cambios) |

## Cómo se ejecuta

El orden completo está en el anexo del informe (sección 12). En resumen:

```bash
py -3.13 -m pip install PySide6 pyqtgraph pyserial numpy scipy matplotlib markdown coppeliasim-zmqremoteapi-client
octave-cli --eval "cd agv/octave; ensayos_agv"        # simulación del AGV
py -3.13 interfaz/robot_gui.py                         # interfaz
py -3.13 informe/armar_informe.py                      # informe (HTML y PDF)
```

**Firmware:** copiar `agv/firmware/agv_osoyoo/wifi_config.ejemplo.h` como `wifi_config.h` y completar la
red, la clave y la IP de la PC. El archivo real no se publica porque contiene la clave del laboratorio.
