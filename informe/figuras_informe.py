#!/usr/bin/env python3
"""figuras_informe.py — Figuras propias del informe:
  * brazo/resultados/figuras/cad_fusion.png      vista 3D del brazo desde los STL exportados por Fusion
  * informe/figuras/arquitectura_software.png    diagrama de la arquitectura de software (enunciado §7.1)
Uso: py -3.13 informe/figuras_informe.py
"""
import os
import struct

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, ".."))
STL = os.path.join(RAIZ, "brazo", "stl")
FIG_B = os.path.join(RAIZ, "brazo", "resultados", "figuras")
FIG_I = os.path.join(AQUI, "figuras")
os.makedirs(FIG_I, exist_ok=True)


def leer_stl(p):
    b = open(p, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("x", "<u2")]))
    return a["v"].astype(float)


# ------------------------------------------------------------------ CAD
fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection="3d")
colores = {"L0": "#9a9aa0", "L1": "#e8892a", "L2_brazo__estructura_a": "#7fa7d9", "L2_brazo__estructura_b": "#3d6fb5",
           "L3": "#e8892a", "L4": "#9a9aa0", "L5": "#e8892a", "L6": "#9a9aa0", "Pieza": "#d33"}
luz = np.array([0.4, -0.5, 0.8]); luz /= np.linalg.norm(luz)
for f in sorted(os.listdir(STL)):
    if not f.endswith(".stl"):
        continue
    t = leer_stl(os.path.join(STL, f))
    base = f[:-4]
    col = "#303034" if "__motor" in f else next((c for k, c in colores.items() if base.startswith(k)), "#999")
    nrm = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    sombra = 0.55 + 0.45 * np.clip(nrm @ luz, 0, 1)
    rgb = np.array(matplotlib.colors.to_rgb(col))
    pc = Poly3DCollection(t, facecolors=np.clip(rgb[None, :] * sombra[:, None], 0, 1), edgecolors="none")
    ax.add_collection3d(pc)
ax.set_xlim(-150, 350); ax.set_ylim(-250, 250); ax.set_zlim(0, 500)
ax.set_box_aspect((1, 1, 1))
ax.view_init(elev=22, azim=-60)
ax.set_xlabel("x₀ [mm]"); ax.set_ylabel("y₀ [mm]"); ax.set_zlabel("z₀ [mm]")
ax.set_title("Brazo 6R en la pose cero (STL de Fusion)\nL2 en dos mitades: hombro (celeste) y codo (azul)")
fig.tight_layout()
fig.savefig(os.path.join(FIG_B, "cad_fusion.png"), dpi=150)
plt.close(fig)

# ------------------------------------------------------------------ arquitectura de software
fig, ax = plt.subplots(figsize=(13, 8.2))
ax.set_xlim(0, 130); ax.set_ylim(0, 82); ax.axis("off")


def caja(x, y, w, h, texto, color="#eef3fb", borde="#3d6fb5", fs=8.5, peso="normal"):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.4,rounding_size=1.2", fc=color, ec=borde, lw=1.2))
    ax.text(x + w / 2, y + h / 2, texto, ha="center", va="center", fontsize=fs, weight=peso, wrap=True)


def flecha(x1, y1, x2, y2, texto="", lado=0.0, estilo="-|>", col="#333"):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle=estilo, mutation_scale=11, lw=1.1, color=col,
                                 connectionstyle=f"arc3,rad={lado}"))
    if texto:
        ax.text((x1 + x2) / 2, (y1 + y2) / 2 + 1.2, texto, ha="center", va="bottom", fontsize=7.3, color="#333")


# carriles
for y0, alto, nombre, c in [(55, 25, "BRAZO 6R (virtual)", "#f6f9fe"), (28, 25, "AGV OSOYOO (físico)", "#fbf6ef"),
                            (2, 24, "AGV OSOYOO (virtual) y datos comunes", "#f4f8f2")]:
    ax.add_patch(FancyBboxPatch((1, y0), 128, alto, boxstyle="round,pad=0.2,rounding_size=1.5", fc=c, ec="#bbb", lw=0.8))
    ax.text(2.5, y0 + alto - 2.2, nombre, fontsize=9.5, weight="bold", color="#555")

# brazo
caja(4, 60, 17, 12, "Interfaz Qt\n(pestaña Brazo)\ncampos con rango\n→ config.json")
caja(27, 66, 22, 8, "Referencias\nquíntico PTP / LIN + MCI\nmuestreo 1 ms")
caja(53, 66, 20, 8, "Controlador\nPD+g / PC / PC+I\nT_s = 1 ms (ZOH)")
caja(77, 66, 22, 8, "Planta (Octave)\nNewton–Euler + fricción\nEuler semi-impl. h = 1 ms")
caja(53, 57, 20, 6.5, "Mediciones q, q̇\n(exactas: hipótesis)", color="#fff")
caja(103, 66, 23, 8, "Registro\nresultado.csv (5 ms)\nmetricas.json")
caja(103, 56.5, 23, 7.5, "CoppeliaSim 4.10\njuntas cinemáticas\nreproducción 20 ms")
flecha(21, 66, 27, 70, "octave-cli")
flecha(49, 70, 53, 70); flecha(73, 70, 77, 70, "τ"); flecha(99, 70, 103, 70)
flecha(88, 66, 73, 60.5, "", 0.0); flecha(53, 60.5, 63, 66, "", 0.3)
flecha(114.5, 66, 114.5, 64, "preparar_reproduccion.py")
flecha(103, 72, 21, 72.5, "", 0.12); ax.text(62, 78.3, "gráficos y tabla comparativa", ha="center", fontsize=7.3, color="#333")

# AGV físico
caja(4, 33, 17, 13, "Interfaz Qt / consola\nórdenes $CMD*HH\nlatido PING 100 ms\nregistro CSV/JSON")
caja(30, 40, 20, 8, "Trayectoria quíntica\nGOTO (50 Hz)")
caja(54, 40, 18, 8, "Lazo de pose\nodometría (50 Hz)")
caja(76, 40, 20, 8, "PI de ruedas + FF\n(100 Hz), saturación\nescala común")
caja(100, 40, 26, 8, "Driver TB6612 → 4 motores\nGM25‑370 → ruedas", color="#fff")
caja(76, 30, 20, 7, "Encoders Hall ×4\n(PCINT2, 3300 c/v)", color="#fff")
caja(54, 30, 18, 7, "Odometría\nH⁺Δφ, η calibrado")
caja(100, 30, 26, 7, "Medición externa\nplomada / video", color="#fff", borde="#a33")
ax.text(64, 49.6, "Arduino Mega 2560 · watchdog 300 ms · protecciones", ha="center", fontsize=7.5, color="#8a5a00")
flecha(21, 42, 30, 44, "USB 115200")
flecha(50, 44, 54, 44); flecha(72, 44, 76, 44, "v_b → ω_ref"); flecha(96, 44, 100, 44, "PWM")
flecha(113, 40, 96, 34, "", -0.1); flecha(76, 33.5, 72, 33.5); flecha(63, 37, 63, 40, "ξ̂")
flecha(54, 32, 21, 36, "$TEL 5–50 Hz", 0.1)
flecha(100, 32, 21, 34.5, "", -0.15, col="#a33"); ax.text(84, 27.2, "medición externa con !medir (manual)", ha="center", fontsize=7.3, color="#a33")

# AGV virtual y datos
caja(4, 6, 17, 13, "Interfaz Qt\n(modo virtual)\n→ config.json")
caja(30, 7, 32, 11, "Octave: simular_agv.m\nmismo control que el firmware\n+ planta: base, motor CC, deslizamiento,\nencoders (paso 1 ms)")
caja(68, 7, 20, 11, "resultado.csv\nmetricas.json\n(pose real y odometría)")
caja(94, 5, 32, 15, "parametros/parametros.json\ntabla única: lo leen el script de\nFusion (CAD, STL), Octave, Python\n(interfaz, CoppeliaSim) y define los\nvalores por defecto del firmware", color="#fff7d6", borde="#b08a00", peso="bold")
flecha(21, 12.5, 30, 12.5, "octave-cli"); flecha(62, 12.5, 68, 12.5)
flecha(78, 18, 12.5, 19, "", 0.18); ax.text(45, 24.2, "gráficos", ha="center", fontsize=7.3, color="#333")
fig.tight_layout()
fig.savefig(os.path.join(FIG_I, "arquitectura_software.png"), dpi=150)
print("Figuras:", os.path.join(FIG_B, "cad_fusion.png"), os.path.join(FIG_I, "arquitectura_software.png"))
