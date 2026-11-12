#!/usr/bin/env python3
"""preparar_reproduccion.py — Arma coppelia/reproduccion.csv (50 Hz) con el movimiento que muestra la escena.

Fuentes (resultados de simulación, no referencias):
  - AGV: agv/resultados/ensayos/T4_E4.csv  → pose REAL del trayecto A→B con PI de ruedas + lazo de pose y
    odometría calibrada (agv/octave/ensayos_agv.m).
  - Brazo: coppelia/datos_brazo/resultado.csv → q REAL del lazo de par calculado + integral sobre el ciclo
    (brazo/octave/correr_desde_gui.m con coppelia/datos_brazo/config.json).
Secuencia: 1 s de reposo → AGV de A a B → 0,5 s → ciclo del brazo (toma de la pieza sobre el AGV y
colocación en la mesa) → 1,5 s de reposo.
Columnas: t, q1..q6 [rad], agv_x, agv_y [m], agv_phi [rad], pieza_x, pieza_y, pieza_z [m],
pieza_qx, pieza_qy, pieza_qz, pieza_qw, fase (0 reposo, 1 AGV, 2 brazo).
Uso: py -3.13 coppelia/preparar_reproduccion.py
"""
import math
import os

import numpy as np

from escena_config import H_BANDEJA, LADO_PIEZA, RAIZ, T_MUNDO_BASE, cuaternion, mcd, rot_z

DT = 0.02
agv = np.loadtxt(os.path.join(RAIZ, "agv", "resultados", "ensayos", "T4_E4.csv"), delimiter=",")
br = np.loadtxt(os.path.join(RAIZ, "coppelia", "datos_brazo", "resultado.csv"), delimiter=",")

# ---- AGV: hasta que deja de moverse (+0,5 s)
ta, pa = agv[:, 0], agv[:, 1:4]
mov = np.where(np.linalg.norm(np.diff(pa, axis=0), axis=1) > 1e-6)[0]
ta_fin = ta[mov[-1] + 1] + 0.5
# ---- brazo: q real y tramos; agarre al empezar la subida con pieza (tramo 4), suelta al empezar el 8
tb, qb, seg = br[:, 0], br[:, 7:13], br[:, -1]
t_agarre = tb[np.argmax(seg == 4)]
t_suelta = tb[np.argmax(seg == 8)]

T0, T1 = 1.0, 1.0 + ta_fin                    # inicio del AGV y del brazo
T_fin = T1 + tb[-1] + 1.5
t = np.arange(0.0, T_fin + 1e-9, DT)
filas = []
pieza_suelta = None
for tk in t:
    # AGV
    s = min(max(tk - T0, 0.0), ta[-1])
    ax, ay, aphi = (np.interp(s, ta, pa[:, j]) for j in range(3))
    # brazo
    sb = min(max(tk - T1, 0.0), tb[-1])
    q = np.array([np.interp(sb, tb, qb[:, j]) for j in range(6)])
    fase = 0 if tk < T0 else (1 if tk < T1 else 2)
    # pieza
    if tk < T1 + t_agarre:                    # sobre la bandeja del AGV
        pos = np.array([ax, ay, H_BANDEJA + LADO_PIEZA / 2])
        quat = cuaternion(rot_z(aphi))
    elif tk < T1 + t_suelta:                  # en la pinza: pose del TCP en el mundo
        Tt, _ = mcd(q)
        Tw = T_MUNDO_BASE @ Tt
        pos, quat = Tw[:3, 3], cuaternion(Tw[:3, :3])
    else:                                     # quieta donde se soltó
        if pieza_suelta is None:
            Tt, _ = mcd(q)
            Tw = T_MUNDO_BASE @ Tt
            pieza_suelta = (Tw[:3, 3].copy(), cuaternion(Tw[:3, :3]))
        pos, quat = pieza_suelta
    filas.append([tk, *q, ax, ay, aphi, *pos, *quat, fase])

M = np.array(filas)
salida = os.path.join(RAIZ, "coppelia", "reproduccion.csv")
cab = "t,q1,q2,q3,q4,q5,q6,agv_x,agv_y,agv_phi,pieza_x,pieza_y,pieza_z,pieza_qx,pieza_qy,pieza_qz,pieza_qw,fase"
np.savetxt(salida, M, delimiter=",", header=cab, comments="", fmt="%.6f")
Tt, _ = mcd(qb[np.argmax(seg == 4)])
p_agarre = (T_MUNDO_BASE @ Tt)[:3, 3]
print(f"{len(t)} muestras, {T_fin:.1f} s. AGV en B a los {T0 + ta_fin:.1f} s (pose real final "
      f"{pa[-1, 0]:.4f}, {pa[-1, 1]:.4f} m, {math.degrees(pa[-1, 2]):.2f}°). TCP al agarrar: {np.round(p_agarre, 4)} m")
print("Guardado en", salida)
