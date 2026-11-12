"""ajuste_identificacion.py — Ajuste w = K (|u| - udz) por rueda con los escalones de PWM de la identificación
(robot elevado, guion identificacion_escalones.txt). Usa solo los escalones en régimen con la rueda girando
(|w| > 0,5 rad/s): identificar_lab.m separaba pares/impares y a RL le quedaba un solo punto con giro.
Uso: py -3.13 agv/pc/ajuste_identificacion.py agv/datos_lab/<carpeta de identificación>/telemetria.csv
"""
import csv, collections, sys
import numpy as np

f = list(csv.DictReader(open(sys.argv[1])))
g = collections.OrderedDict()
for r in f:
    u = int(r["u0"])
    if u:
        g.setdefault(u, []).append(r)
NOM = ["FL", "FR", "RL", "RR"]
for i in range(4):
    U, W = [], []
    for u, rs in g.items():
        s = rs[int(len(rs) * 0.6):]
        w = sum(int(r[f"wm{i}_mrad_s"]) for r in s) / len(s) / 1000
        if abs(w) > 0.5:
            U.append(abs(u)); W.append(abs(w))
    A = np.column_stack([U, np.ones(len(U))])
    (k, b), *_ = np.linalg.lstsq(A, np.array(W), rcond=None)
    udz = -b / k
    rms = np.sqrt(np.mean((k * (np.array(U) - udz) - np.array(W)) ** 2))
    arranque = min(U)
    print(f"{NOM[i]}: K = {k:.4f} rad/s/PWM | u_dz = {udz:5.1f} PWM | rms {rms:.2f} rad/s | n = {len(U)} | "
          f"menor |PWM| con giro: {arranque} | MDL {i} {k:.4f} {udz:.1f}")
