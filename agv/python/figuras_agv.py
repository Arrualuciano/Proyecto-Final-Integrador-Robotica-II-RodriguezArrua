#!/usr/bin/env python3
"""figuras_agv.py — Figuras de los ensayos simulados del AGV (lee agv/resultados/ensayos/T<k>_E<s>.csv
que genera agv/octave/ensayos_agv.m). Columnas: t, pose real(3), odometría(3), referencia(3), w_ref(4),
w_med(4), u(4). Uso: py -3.13 agv/python/figuras_agv.py
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RES = os.path.abspath(os.path.join(AQUI, "..", "resultados"))
FIG = os.path.join(RES, "figuras")
os.makedirs(FIG, exist_ok=True)
ESTR = {1: ("0 · lazo abierto", "tab:red"), 2: ("1 · PI de ruedas", "tab:orange"),
        3: ("2 · PI + pose (odometría nominal)", "tab:blue"), 4: ("2C · pose + odometría calibrada", "tab:green")}
ENS = {1: "T1 avance 1,0 m", 2: "T2 lateral 0,6 m", 3: "T3 giro 90°", 4: "T4 A→B", 5: "T5 A→B + 0,5 kg"}
METAS = {1: (1.0, 0, 0), 2: (0, 0.6, 0), 3: (0, 0, np.pi / 2), 4: (1.2, 0.6, 0), 5: (1.2, 0.6, 0)}
RUEDAS = ["FL", "FR", "RL", "RR"]


def leer(k, s):
    d = np.loadtxt(os.path.join(RES, "ensayos", f"T{k}_E{s}.csv"), delimiter=",")
    return dict(t=d[:, 0], real=d[:, 1:4], od=d[:, 4:7], ref=d[:, 7:10], wr=d[:, 10:14], wm=d[:, 14:18], u=d[:, 18:22])


# 1) recorridos del trayecto A -> B
fig, ax = plt.subplots(figsize=(8, 6))
for s, (nom, col) in ESTR.items():
    r = leer(4, s)
    ax.plot(r["real"][:, 0], r["real"][:, 1], color=col, lw=2, label=f"{nom}: real")
    ax.plot(r["od"][:, 0], r["od"][:, 1], color=col, lw=1, ls="--")
r = leer(4, 4)
ax.plot(r["ref"][:, 0], r["ref"][:, 1], "k:", lw=1, label="referencia")
ax.plot([0, 1.2, 1.2], [0, 0, 0.6], "ks", ms=7, label="estaciones A, intermedia, B")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_aspect("equal"); ax.grid(True, alpha=0.4)
ax.set_title("T4 A→B: recorrido real (línea) y estimado por odometría (rayas)")
ax.legend(fontsize=8, loc="upper left")
fig.tight_layout(); fig.savefig(os.path.join(FIG, "agv_T4_recorridos.png"), dpi=130)

# 2) error final por ensayo y estrategia
fig, ax = plt.subplots(figsize=(9, 4.5))
ancho = 0.2
for j, (s, (nom, col)) in enumerate(ESTR.items()):
    e = []
    for k in ENS:
        r = leer(k, s); m = METAS[k]
        if k == 3:
            e.append(np.degrees(abs(np.arctan2(np.sin(r["real"][-1, 2] - m[2]), np.cos(r["real"][-1, 2] - m[2])))))
        else:
            e.append(1000 * np.hypot(r["real"][-1, 0] - m[0], r["real"][-1, 1] - m[1]))
    ax.bar(np.arange(len(ENS)) + (j - 1.5) * ancho, e, ancho, color=col, label=nom)
ax.axhline(20, color="k", ls=":", lw=1, label="requisito 20 mm (T3: 3°)")
ax.set_xticks(range(len(ENS))); ax.set_xticklabels(ENS.values(), fontsize=9)
ax.set_yscale("log"); ax.set_ylabel("error final real [mm]  (T3: [°])"); ax.grid(True, axis="y", alpha=0.4)
ax.legend(fontsize=8); ax.set_title("Error final medido externamente (simulación), por estrategia")
fig.tight_layout(); fig.savefig(os.path.join(FIG, "agv_errores_finales.png"), dpi=130)

# 3) ruedas en T4 con odometría calibrada
r = leer(4, 4)
fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
for i in range(4):
    ax[0].plot(r["t"], r["wm"][:, i], label=f"{RUEDAS[i]} medida")
    ax[0].plot(r["t"], r["wr"][:, i], ls="--", color=ax[0].lines[-1].get_color(), lw=1)
    ax[1].plot(r["t"], r["u"][:, i], label=RUEDAS[i])
ax[0].set_ylabel("ω rueda [rad/s]"); ax[1].set_ylabel("PWM"); ax[1].set_xlabel("t [s]")
ax[0].set_title("T4 (estrategia 2C): velocidad de ruedas medida (línea) y referencia (rayas), y comando PWM")
for a in ax:
    a.grid(True, alpha=0.4); a.legend(fontsize=8, ncol=4)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "agv_T4_ruedas.png"), dpi=130)
print("Figuras en", FIG)
