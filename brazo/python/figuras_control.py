#!/usr/bin/env python3
"""figuras_control.py — Figuras de los ensayos de control del brazo
(lee brazo/resultados/control/E*_{pdg,ct}.csv que genera brazo/octave/ensayos_control_brazo.m).
Columnas de cada CSV (cada 5 ms): t, q_ref(6), q(6), qdot(6), tau(6), e_pos_mm, e_ang_deg.
Uso: py -3.13 brazo/python/figuras_control.py
"""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, "..", ".."))
DIR_C = os.path.join(RAIZ, "brazo", "resultados", "control")
FIG = os.path.join(RAIZ, "brazo", "resultados", "figuras")
os.makedirs(FIG, exist_ok=True)
P = json.load(open(os.path.join(RAIZ, "parametros", "parametros.json"), encoding="utf-8"))
ac = P["brazo"]["accionamientos"]
tau_lim = np.array(ac["eta"]) * np.array(ac["N"]) * np.array(ac["tau_motor_max"])

ENSAYOS = {1: "E1 nominal (0,10 kg)", 2: "E2 carga 0,25 kg + pulso 1 N·m en J2", 3: "E3 masas reales +15 %"}
CTRL = {"pdg": ("PD + gravedad", "tab:orange"), "ct": ("Par calculado", "tab:blue"),
        "cti": ("Par calculado + integral", "tab:green")}


def leer(e, c):
    d = np.loadtxt(os.path.join(DIR_C, f"E{e}_{c}.csv"), delimiter=",")
    return dict(t=d[:, 0], qr=d[:, 1:7], q=d[:, 7:13], qd=d[:, 13:19], tau=d[:, 19:25], ep=d[:, 25], ea=d[:, 26])


# 1) error de posicion del TCP, tres ensayos
fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
for a, e in zip(ax, ENSAYOS):
    for c, (nom, col) in CTRL.items():
        r = leer(e, c)
        a.plot(r["t"], r["ep"], color=col, label=nom)
    a.axhline(1.0, color="k", ls=":", lw=1, label="requisito en toma/colocación (1 mm)")
    a.set_ylabel("error TCP [mm]"); a.set_title(ENSAYOS[e]); a.grid(True, alpha=0.4)
    a.set_yscale("log"); a.set_ylim(1e-3, 50)
ax[0].legend(fontsize=8, loc="upper right")
ax[-1].set_xlabel("t [s]")
fig.tight_layout(); fig.savefig(os.path.join(FIG, "control_error_tcp.png"), dpi=130)

# 2) seguimiento y par de J2 en E2 (el eje mas exigido, con el pulso)
fig, ax = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
for c, (nom, col) in CTRL.items():
    r = leer(2, c)
    ax[0].plot(r["t"], np.degrees(r["q"][:, 1] - r["qr"][:, 1]), color=col, label=nom)
    ax[1].plot(r["t"], r["tau"][:, 1], color=col, label=nom)
ax[1].axhline(-tau_lim[1], color="r", ls="--", lw=1, label="límite del accionamiento")
ax[1].axhline(tau_lim[1], color="r", ls="--", lw=1)
ax[0].axvspan(5.0, 5.2, color="0.85", label="pulso de 1 N·m")
ax[0].set_ylabel("q₂ − q₂,ref [°]"); ax[1].set_ylabel("τ₂ aplicado [N·m]"); ax[1].set_xlabel("t [s]")
ax[0].set_title("E2: error y par de la junta 2 (carga real 0,25 kg, modelo 0,10 kg)")
for a in ax:
    a.grid(True, alpha=0.4); a.legend(fontsize=8)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "control_J2_E2.png"), dpi=130)
print("Figuras guardadas en", FIG)
