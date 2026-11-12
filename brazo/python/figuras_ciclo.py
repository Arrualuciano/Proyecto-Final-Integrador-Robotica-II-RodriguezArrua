#!/usr/bin/env python3
"""figuras_ciclo.py — Figuras del ciclo del brazo a partir de los CSV que genera
brazo/octave/verificar_trayectoria_brazo.m (resultados/ciclo_brazo.csv y ciclo_brazo_tcp.csv).
Uso: py -3.13 brazo/python/figuras_ciclo.py
"""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, "..", ".."))
RES = os.path.join(RAIZ, "brazo", "resultados")
FIG = os.path.join(RES, "figuras")
os.makedirs(FIG, exist_ok=True)

P = json.load(open(os.path.join(RAIZ, "parametros", "parametros.json"), encoding="utf-8"))
ac = P["brazo"]["accionamientos"]
N = np.array(ac["N"], float); eta = np.array(ac["eta"], float); tmax = np.array(ac["tau_motor_max"], float)

d = np.loadtxt(os.path.join(RES, "ciclo_brazo.csv"), delimiter=",")
t = d[:, 0]; q = d[:, 1:7]; qd = d[:, 7:13]; qdd = d[:, 13:19]; mp = d[:, 19]; seg = d[:, 20]; tau = d[:, 21:27]
p = np.loadtxt(os.path.join(RES, "ciclo_brazo_tcp.csv"), delimiter=",")
ptcp = p[:, 1:4]

cambios = t[np.r_[False, np.diff(seg) != 0]]
fig, ax = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
etiquetas = [f"q{i+1}" for i in range(6)]
for a, y, lab in zip(ax, [np.degrees(q), np.degrees(qd), tau, 100 * np.abs(tau) / (eta * N) / tmax],
                     ["q [°]", "q̇ [°/s]", "τ articular [N·m]", "uso del motor [%]"]):
    a.plot(t, y)
    a.set_ylabel(lab); a.grid(True, alpha=0.4)
    for c in cambios:
        a.axvline(c, color="0.75", lw=0.8, ls="--")
ax[0].legend(etiquetas, ncol=6, fontsize=8, loc="upper right")
ax[0].set_title("Ciclo toma–colocación: referencias, pares y uso de los motores (pieza 0,1 kg)")
ax[3].set_xlabel("t [s]")
fig.tight_layout()
fig.savefig(os.path.join(FIG, "ciclo_brazo_articular.png"), dpi=130)

fig = plt.figure(figsize=(7, 6))
a = fig.add_subplot(111, projection="3d")
a.plot(ptcp[:, 0], ptcp[:, 1], ptcp[:, 2], lw=1.8, label="TCP")
a.scatter([0.30, 0.0], [0.0, 0.30], [0.02, 0.02], c="r", s=40, label="toma / colocación")
a.scatter([0], [0], [0], c="k", s=30, label="base {0}")
a.set_xlabel("x₀ [m]"); a.set_ylabel("y₀ [m]"); a.set_zlabel("z₀ [m]")
a.set_box_aspect((1, 1, 0.8)); a.view_init(25, 35); a.legend(fontsize=8)
a.set_title("Recorrido del TCP en el ciclo")
fig.tight_layout()
fig.savefig(os.path.join(FIG, "ciclo_brazo_tcp.png"), dpi=130)
print("Figuras guardadas en", FIG)
