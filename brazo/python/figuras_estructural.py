#!/usr/bin/env python3
"""figuras_estructural.py — Figuras de la verificación estructural de L2
(lee brazo/resultados/verificacion_estructural.mat que genera brazo/octave/verificacion_estructural.m).
Uso: py -3.13 brazo/python/figuras_estructural.py
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, "..", ".."))
FIG = os.path.join(RAIZ, "brazo", "resultados", "figuras")
os.makedirs(FIG, exist_ok=True)
d = loadmat(os.path.join(RAIZ, "brazo", "resultados", "verificacion_estructural.mat"))

fig, ax = plt.subplots(1, 2, figsize=(12, 4.6), gridspec_kw={"width_ratios": [1, 1.5]})

# a) momentos internos a lo largo de L2 en la peor postura estática
s = d["s_diag"].ravel() * 1000
M = d["M_diag"]
for i, (nom, col) in enumerate([("T (torsión, eje x₂)", "tab:green"), ("M_y (flexión, eje y₂)", "tab:orange"),
                                ("M_z (flexión, eje de la articulación)", "tab:blue")]):
    ax[0].plot(s, M[i], color=col, label=nom)
for x, txt in [(35, "raíz"), (75, "solape"), (125, ""), (160, "horquilla")]:
    ax[0].axvline(x, color="0.6", ls=":", lw=1)
    ax[0].text(x + 2, ax[0].get_ylim()[1] * 0.9 if txt else 0, txt, fontsize=8, color="0.4")
q2, q3 = np.degrees(d["q_C2"].ravel()[1:3])
ax[0].set_title(f"(a) Esfuerzos internos en L2, peor postura estática\n(q₂, q₃) = ({q2:.0f}°, {q3:.0f}°), carga 0,25 kg")
ax[0].set_xlabel("s: distancia desde el hombro [mm]")
ax[0].set_ylabel("momento [N·m]")
ax[0].grid(alpha=0.3)
ax[0].legend(fontsize=8, loc="lower left")

# b) factor de seguridad a lo largo del ciclo
t = d["t_ciclo"].ravel()
FS = d["FS1"]
for i, (nom, col) in enumerate([("raíz, impresa acostada", "tab:blue"), ("raíz, impresa parada", "tab:red"),
                                ("media sección del solape (acostada)", "tab:purple")]):
    ax[1].semilogy(t, FS[i], color=col, label=nom)
ax[1].axhline(3, color="k", ls="--", lw=1, label="FS requerido (dinámico) = 3")
ax[1].set_ylim(1, 1000)
ax[1].set_title("(b) Factor de seguridad durante el ciclo con 0,25 kg")
ax[1].set_xlabel("t [s]")
ax[1].set_ylabel("FS (criterio cuadrático, resistencia según capas)")
ax[1].grid(alpha=0.3, which="both")
ax[1].legend(fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(FIG, "estructural_L2.png"), dpi=150)
print("Figura:", os.path.join(FIG, "estructural_L2.png"))
