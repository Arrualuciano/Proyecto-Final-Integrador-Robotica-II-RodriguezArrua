#!/usr/bin/env python3
"""figuras_piso.py — Figuras y tabla de los ensayos físicos en el piso (11/11/2026) frente a la simulación.

Lee agv/datos_lab/mediciones_piso.csv (medición externa sobre las baldosas de 30 cm, agregada con
corrida.py --medir) y la telemetría de cada corrida (odometría del robot), y los CSV simulados de
agv/resultados/ensayos/T<k>_E<s>.csv (ensayos_agv.m con los parámetros medidos).
Salida: agv/resultados/figuras/agv_piso_*.png y agv/resultados/ensayos_piso_resumen.csv
Uso: py -3.13 agv/python/figuras_piso.py
"""
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, "..", ".."))
LAB = os.path.join(RAIZ, "agv", "datos_lab")
RES = os.path.join(RAIZ, "agv", "resultados")
FIG = os.path.join(RES, "figuras")
os.makedirs(FIG, exist_ok=True)
B = 0.30                                    # lado de baldosa [m]

# ensayo físico -> (nombre, archivo simulado comparable T<k>_E<s>) ; E1 abierto, E3 pose nominal, E4 pose calibrada
GRUPOS = [("T1", "0", "T1 avance · modo 0", (1, 1)), ("T1", "2", "T1 avance · modo 2", (1, 3)),
          ("T2", "0", "T2 lateral · modo 0", (2, 1)), ("T2", "2", "T2 lateral · modo 2", (2, 3)),
          ("T3", "0", "T3 giro · modo 0", (3, 1)), ("T3", "2", "T3 giro · modo 2", (3, 3)),
          ("T4", "0", "T4 A→B · modo 0", (4, 1)), ("T4C", "2", "T4 A→B · modo 2 cal.", (4, 4)),
          ("T5", "2", "T5 A→B + carga · modo 2 cal.", (5, 4)),
          ("T5S", "2", "T5 + carga, rampa suave (amax 0,15)", None)]


def mediciones():
    return list(csv.DictReader(open(os.path.join(LAB, "mediciones_piso.csv"), encoding="utf-8")))


def odometria(carpeta):
    """Trayectoria de odometría de la corrida: desde el primer instante en movimiento (modo 5 = IR_A)."""
    tel = list(csv.DictReader(open(os.path.join(LAB, carpeta, "telemetria.csv"), encoding="utf-8")))
    i0 = next((i for i, r in enumerate(tel) if r["modo"] == "5"), 0)
    tel = tel[i0:]
    return (np.array([float(r["x_mm"]) for r in tel]) / 1000, np.array([float(r["y_mm"]) for r in tel]) / 1000,
            tel)


def simulado(k, s):
    d = np.loadtxt(os.path.join(RES, "ensayos", f"T{k}_E{s}.csv"), delimiter=",")
    return d


def err_sim(k, s, meta):
    d = simulado(k, s)
    f = d[-1, 1:4]
    ep = 1000 * np.hypot(f[0] - meta[0], f[1] - meta[1])
    ea = np.degrees(np.arctan2(np.sin(f[2] - meta[2]), np.cos(f[2] - meta[2])))
    return ep, ea


def envolver_deg(a):
    return (a + 180) % 360 - 180


M = mediciones()

# ------------------------------------------------------------------ tabla resumen
filas = []
for ens, modo, nom, sim in GRUPOS:
    rs = [r for r in M if r["ensayo"] == ens and r["modo"] == modo]
    if not rs:
        continue
    meta = (float(rs[0]["meta_x_mm"]) / 1000, float(rs[0]["meta_y_mm"]) / 1000, np.radians(float(rs[0]["meta_phi_deg"])))
    ep = np.array([float(r["error_real_mm"]) for r in rs])
    ea = np.array([envolver_deg(float(r["real_phi_deg"]) - float(r["meta_phi_deg"])) for r in rs])
    dr = np.array([float(r["deriva_odom_mm"]) for r in rs])
    eps, eas = err_sim(*sim, meta) if sim else (np.nan, np.nan)
    filas.append(dict(grupo=nom, n=len(rs), error_mm_media=ep.mean(), error_mm_min=ep.min(), error_mm_max=ep.max(),
                      error_deg_media=ea.mean(), deriva_mm_media=dr.mean(), sim_error_mm=eps, sim_error_deg=eas))
with open(os.path.join(RES, "ensayos_piso_resumen.csv"), "w", newline="", encoding="utf-8") as fh:
    w = csv.DictWriter(fh, fieldnames=list(filas[0]))
    w.writeheader()
    for f in filas:
        w.writerow({k: (round(v, 1) if isinstance(v, float) else v) for k, v in f.items()})
for f in filas:
    print(f"{f['grupo']:38s} n={f['n']}  error {f['error_mm_media']:6.1f} mm ({f['error_mm_min']:.0f}-{f['error_mm_max']:.0f})"
          f"  {f['error_deg_media']:6.1f}°  deriva {f['deriva_mm_media']:6.1f} mm | sim {f['sim_error_mm']:6.1f} mm {f['sim_error_deg']:5.1f}°")

# ------------------------------------------------------------------ 1) trayecto A -> B en el piso
fig, ax = plt.subplots(figsize=(9, 5.6))
for x in np.arange(-1, 7) * B:
    ax.axvline(x, color="#c9b89a", lw=0.8, zorder=0)
for y in np.arange(-2, 5) * B:
    ax.axhline(y, color="#c9b89a", lw=0.8, zorder=0)
ax.set_facecolor("#f6f1e7")
d = simulado(4, 4)
ax.plot(d[:, 7], d[:, 8], color="k", ls=":", lw=1.2, label="referencia (quíntica)")
ax.plot(d[:, 1], d[:, 2], color="0.45", lw=2.5, alpha=0.6, label="simulado, modo 2 calibrado (real)")
ESTILO = {"T4": ("tab:red", "T4 modo 0 (lazo abierto)"), "T4C": ("tab:blue", "T4 modo 2 calibrado"),
          "T5": ("tab:purple", "T5 modo 2 cal. + 0,5 kg"), "T5S": ("tab:green", "T5 + 0,5 kg, rampa suave")}
vistos = set()
for r in M:
    if r["ensayo"] not in ESTILO:
        continue
    col, nom = ESTILO[r["ensayo"]]
    ox, oy, _ = odometria(r["carpeta"])
    ax.plot(ox, oy, color=col, lw=1, ls="--", alpha=0.8)
    xr, yr, fr = float(r["real_x_mm"]) / 1000, float(r["real_y_mm"]) / 1000, np.radians(float(r["real_phi_deg"]))
    ax.plot(xr, yr, "o", color=col, ms=8, mec="k", label=(nom + ": pose real medida") if nom not in vistos else None)
    ax.annotate("", xy=(xr + 0.12 * np.cos(fr), yr + 0.12 * np.sin(fr)), xytext=(xr, yr),
                arrowprops=dict(arrowstyle="->", color=col, lw=1.8))
    vistos.add(nom)
ax.plot([], [], color="0.3", ls="--", lw=1, label="odometría del robot (rayas, color de cada ensayo)")
for (x, y, t) in ((0, 0, "A"), (1.2, 0.6, "B")):
    ax.plot(x, y, "ks", ms=9)
    ax.text(x - 0.05, y + 0.04, t, fontsize=12, weight="bold", ha="right")
ax.set_xlim(-0.25, 1.75); ax.set_ylim(-0.25, 0.95); ax.set_aspect("equal")
ax.set_xticks(np.arange(0, 7) * B); ax.set_yticks(np.arange(0, 4) * B)
ax.set_xlabel("x [m] (baldosas de 30 cm; x adelante en A)"); ax.set_ylabel("y [m] (izquierda en A)")
ax.set_title("Trayecto A→B en el piso: pose final medida (punto y flecha de orientación) y odometría")
ax.legend(fontsize=7.5, loc="upper left", framealpha=0.95)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "agv_piso_AB.png"), dpi=150); plt.close(fig)

# ------------------------------------------------------------------ 2) errores físicos vs simulados
fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.6))
y = np.arange(len(filas))[::-1]
for j, f in enumerate(filas):
    rs = [r for r in M if f["grupo"].startswith(r["ensayo"] + " ") or False]
for j, (ens, modo, nom, sim) in enumerate([g for g in GRUPOS if any(r["ensayo"] == g[0] and r["modo"] == g[1] for r in M)]):
    rs = [r for r in M if r["ensayo"] == ens and r["modo"] == modo]
    ep = [float(r["error_real_mm"]) for r in rs]
    ea = [envolver_deg(float(r["real_phi_deg"]) - float(r["meta_phi_deg"])) for r in rs]
    yy = y[j]
    a1.barh(yy + 0.18, np.mean(ep), height=0.34, color="tab:blue", label="físico (media)" if j == 0 else None)
    a1.plot(ep, [yy + 0.18] * len(ep), "k.", ms=6, label="físico (cada corrida)" if j == 0 else None)
    a2.barh(yy + 0.18, np.mean(ea), height=0.34, color="tab:blue")
    a2.plot(ea, [yy + 0.18] * len(ea), "k.", ms=6)
    if sim:
        es, eas = filas[j]["sim_error_mm"], filas[j]["sim_error_deg"]
        a1.barh(yy - 0.18, es, height=0.34, color="0.65", label="simulado" if j == 0 else None)
        a2.barh(yy - 0.18, eas, height=0.34, color="0.65")
a1.set_yticks(y); a1.set_yticklabels([f["grupo"] for f in filas], fontsize=8)
a2.set_yticks(y); a2.set_yticklabels([])
a1.set_xlabel("error final de posición respecto de la meta [mm]"); a2.set_xlabel("error final de orientación [°]")
a2.axvline(0, color="k", lw=0.8)
for a in (a1, a2):
    a.grid(True, axis="x", alpha=0.4)
a1.legend(fontsize=8, loc="lower right")
a1.set_title("Posición"); a2.set_title("Orientación (+ = antihorario)")
fig.tight_layout(); fig.savefig(os.path.join(FIG, "agv_piso_errores.png"), dpi=150); plt.close(fig)

# ------------------------------------------------------------------ 3) arranque del tramo lateral con carga (T5 rep. 1)
r = next(r for r in M if r["ensayo"] == "T5" and r["rep"] == "1")
_, _, tel = odometria(r["carpeta"])
k = next(i for i, q in enumerate(tel) if float(q["yr_mm"]) > 1 and float(q["xr_mm"]) >= 1195)
seg = tel[max(0, k - 5):k + 60]
t = (np.array([float(q["t_ms"]) for q in seg]) - float(tel[k]["t_ms"])) / 1000
fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5.4), sharex=True)
COL = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
for i, n in enumerate(["FL", "FR", "RL", "RR"]):
    a1.plot(t, [abs(float(q[f"wm{i}_mrad_s"])) / 1000 for q in seg], color=COL[i], lw=2, label=f"{n} medida")
    a1.plot(t, [abs(float(q[f"wr{i}_mrad_s"])) / 1000 for q in seg], color=COL[i], lw=1, ls=":")
    a2.plot(t, [abs(float(q[f"u{i}"])) for q in seg], color=COL[i], lw=2, label=n)
a1.set_ylabel("|ω| rueda [rad/s]"); a2.set_ylabel("|PWM|"); a2.set_xlabel("tiempo desde el inicio del tramo lateral [s]")
a1.set_title("T5 (0,5 kg), arranque del tramo lateral: las ruedas despegan en momentos distintos")
a1.legend(fontsize=8, ncol=4); a2.legend(fontsize=8, ncol=4)
for a in (a1, a2):
    a.grid(True, alpha=0.4)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "agv_piso_arranque_lateral.png"), dpi=150); plt.close(fig)
print("Figuras en", FIG)
