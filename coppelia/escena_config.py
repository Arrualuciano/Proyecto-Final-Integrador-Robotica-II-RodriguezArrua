"""escena_config.py — Geometría de la celda compartida por preparar_reproduccion.py y construir_escena.py.

Marco del mundo de CoppeliaSim = marco {G} de la celda (piso), con origen en la estación A del AGV.
El AGV termina el trayecto en B = (1,2; 0,6) m con la pieza en el centro de su bandeja superior.
La base del brazo {0} se ubica de modo que el punto de toma (0,30; 0; 0,02) de la tarea coincida con la
pieza sobre el AGV en B: base en (1,2; 0,9) a la altura de la bandeja, con x0 apuntando a −y del mundo.
La colocación (0; 0,30; 0,02) en {0} queda en (1,5; 0,9) del mundo, sobre la mesa de trabajo.
"""
import json
import math
import os

import numpy as np

RAIZ = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
P = json.load(open(os.path.join(RAIZ, "parametros", "parametros.json"), encoding="utf-8"))

H_BANDEJA = 0.090          # m, altura de la placa superior del AGV sobre el piso [E, a medir]
BASE_POS = np.array([1.2, 0.9, H_BANDEJA])
BASE_YAW = -math.pi / 2    # rotación de {0} respecto del mundo alrededor de z
LADO_PIEZA = P["pieza"]["lado"]


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


T_MUNDO_BASE = np.eye(4)
T_MUNDO_BASE[:3, :3] = rot_z(BASE_YAW)
T_MUNDO_BASE[:3, 3] = BASE_POS


def dh_A(th, d, a, al):
    ct, st, ca, sa = math.cos(th), math.sin(th), math.cos(al), math.sin(al)
    return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1.0]])


def mcd(q):
    """^0T_TCP y las tramas ^0T_i (i = 0..6) del brazo (misma convención que brazo/octave/mcd_brazo.m)."""
    T = np.eye(4)
    tramas = [T.copy()]
    for qi, f in zip(q, P["brazo"]["dh"]):
        T = T @ dh_A(qi + math.radians(f["theta_offset_deg"]), f["d"], f["a"], math.radians(f["alpha_deg"]))
        tramas.append(T.copy())
    Th = np.eye(4)
    Th[2, 3] = P["brazo"]["herramienta"]["tcp_z6"]
    return T @ Th, tramas


def cuaternion(R):
    """Matriz de rotación -> (qx, qy, qz, qw)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)
