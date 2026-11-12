#!/usr/bin/env python3
"""interferencias_ciclo.py — Revisión de interferencias del brazo a lo largo del ciclo completo.

Coloca las mallas STL que exporta Fusion (brazo/stl, mm, pose cero, marco {0}) en la pose de cada instante
del ciclo simulado (q REAL del lazo de control: coppelia/datos_brazo/resultado.csv) y calcula, con
operaciones booleanas exactas (manifold3d), el volumen de intersección:
  - entre eslabones NO contiguos (L0..L6, cada uno con su estructura y su motor);
  - entre cada eslabón y el entorno de la celda: piso, mesa con la bandeja C y bandeja del AGV en B.
Los eslabones contiguos comparten el eje de su articulación por diseño y no se comparan; la pieza tampoco,
porque el contacto con la pinza es el agarre. Umbral de interferencia: 1 mm³.
Salida: resumen por consola y brazo/resultados/interferencias_ciclo.csv (separación mínima no se calcula:
solo presencia/volumen de intersección).
Uso: py -3.13 brazo/python/interferencias_ciclo.py
"""
import csv
import glob
import os
import sys

import manifold3d as mf
import numpy as np
import trimesh

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, "..", ".."))
sys.path.insert(0, os.path.join(RAIZ, "coppelia"))
from escena_config import H_BANDEJA, T_MUNDO_BASE, mcd  # noqa: E402

UMBRAL = 1e-9          # m³ (1 mm³)
CADA = 10              # muestras del CSV entre instantes revisados (CSV cada 5 ms -> cada 0,05 s)


def a_manifold(m):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32),
                               tri_verts=np.asarray(m.faces, np.uint32)))


def caja_en_0(centro_mundo, tam):
    """Caja alineada con el mundo, expresada en el marco {0} del brazo."""
    b = trimesh.creation.box(extents=tam)
    T = np.eye(4)
    T[:3, 3] = centro_mundo
    b.apply_transform(np.linalg.inv(T_MUNDO_BASE) @ T)
    return b


# ---- mallas de los eslabones en la pose cero, en metros
eslabones = {}
for ruta in sorted(glob.glob(os.path.join(RAIZ, "brazo", "stl", "L*__*.stl"))):
    nombre = os.path.basename(ruta).split("__")[0]
    m = trimesh.load(ruta, force="mesh")
    m.apply_scale(0.001)
    eslabones.setdefault(nombre, []).append(m)
nombres = sorted(eslabones)                          # L0_base, L1_hombro, ..., L6_brida_pinza
indice = {n: int(n[1]) for n in nombres}
base = {n: trimesh.util.concatenate(eslabones[n]) for n in nombres}

# ---- entorno en {0}
mesa = T_MUNDO_BASE @ np.array([0, 0.30, 0, 1.0])
entorno = {
    "piso": caja_en_0([1.2, 0.9, -0.05], [3.0, 3.0, 0.1]),
    "mesa_y_bandeja_C": caja_en_0([mesa[0], mesa[1], H_BANDEJA / 2 + 0.002], [0.30, 0.30, H_BANDEJA + 0.004]),
    "bandeja_AGV_en_B": caja_en_0([1.2, 0.6, H_BANDEJA - 0.0015], [0.200, 0.155, 0.003]),
}
entorno_mf = {k: a_manifold(v) for k, v in entorno.items()}

# ---- ciclo
d = np.loadtxt(os.path.join(RAIZ, "coppelia", "datos_brazo", "resultado.csv"), delimiter=",")
t, Q = d[::CADA, 0], d[::CADA, 7:13]
_, T0 = mcd(np.zeros(6))
inv0 = {i: np.linalg.inv(T0[i]) for i in range(7)}

peor = {}
filas = []
for tk, q in zip(t, Q):
    _, Tq = mcd(q)
    pose = {}
    for n in nombres:
        m = base[n].copy()
        m.apply_transform(Tq[indice[n]] @ inv0[indice[n]])
        pose[n] = a_manifold(m)
    pares = [(a, b) for i, a in enumerate(nombres) for b in nombres[i + 1:] if indice[b] - indice[a] >= 2]
    pares += [(n, e) for n in nombres if n != "L0_base" for e in entorno_mf]
    for a, b in pares:
        B = pose[b] if b in pose else entorno_mf[b]
        v = (pose[a] ^ B).volume()
        if v > UMBRAL:
            filas.append([round(tk, 2), a, b, round(v * 1e9, 1)])
            peor[(a, b)] = max(peor.get((a, b), 0.0), v)

os.makedirs(os.path.join(RAIZ, "brazo", "resultados"), exist_ok=True)
with open(os.path.join(RAIZ, "brazo", "resultados", "interferencias_ciclo.csv"), "w", newline="", encoding="utf-8") as fh:
    w = csv.writer(fh)
    w.writerow(["t_s", "cuerpo_a", "cuerpo_b", "volumen_interseccion_mm3"])
    w.writerows(filas)
print(f"Instantes revisados: {len(t)} (cada {t[1] - t[0]:.2f} s, {t[0]:.1f}–{t[-1]:.1f} s); "
      f"eslabones: {', '.join(nombres)}; entorno: {', '.join(entorno)}")
if not peor:
    print("Sin interferencias (> 1 mm³) entre eslabones no contiguos ni con el entorno en todo el ciclo.")
else:
    for (a, b), v in sorted(peor.items(), key=lambda x: -x[1]):
        ts = [f[0] for f in filas if f[1] == a and f[2] == b]
        print(f"INTERFERENCIA {a} – {b}: hasta {v * 1e9:.0f} mm³, entre t = {min(ts):.1f} y {max(ts):.1f} s")
