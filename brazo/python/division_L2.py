#!/usr/bin/env python3
"""division_L2.py — División de L2 en dos mitades imprimibles y su verificación.

La división "oficial" la hace el script de Fusion (brazo/cad/fusion_brazo) con los datos de
parametros.json → brazo.cad.eslabones[L2_brazo].division. Este script:
  * si en brazo/stl todavía está L2 entero (L2_brazo__estructura.stl, versión anterior del CAD),
    aplica la MISMA operación sobre la malla (agujeros + corte con solape) y escribe
    L2_brazo__estructura_a.stl / _b.stl y L2_brazo_a.3mf / _b.3mf (provisorios hasta volver a
    correr Fusion);
  * verifica las mitades (de Fusion o provisorias): volumen, envolvente en la orientación de
    impresión, que entren en la cama, área que necesita soporte, área de contacto del solape.
Uso: py -3.13 brazo/python/division_L2.py
"""
import json
import os

import numpy as np
import trimesh

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, "..", ".."))
STL = os.path.join(RAIZ, "brazo", "stl")
P = json.load(open(os.path.join(RAIZ, "parametros", "parametros.json"), encoding="utf-8"))
esl = next(e for e in P["brazo"]["cad"]["eslabones"] if e["nombre"] == "L2_brazo")
div = esl["division"]
cama = np.array(P["brazo"]["material"]["impresion"]["cama_max"]) * 1000   # mm
d1 = P["brazo"]["dh"][0]["d"]
S = np.array([0.0, 0.0, d1]) * 1000                                       # hombro en la pose cero [mm]
assert div["ref"] == "S"


def caja(xmin, xmax, ymin, ymax, zmin, zmax):
    b = trimesh.creation.box(extents=[xmax - xmin, ymax - ymin, zmax - zmin])
    b.apply_translation([(xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2])
    return b


def dividir(m):
    for h in div["agujeros"]:
        c = S + np.array(h["centro"]) * 1000
        cil = trimesh.creation.cylinder(radius=h["d"] * 500, height=400, sections=48)
        cil.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))   # eje y0
        cil.apply_translation(c)
        m = m.difference(cil, engine="manifold")
    G = 1000.0
    z0, z1 = S[2] + div["s_inicio"] * 1000, S[2] + div["s_fin"] * 1000
    region_a = caja(-G, G, -G, G, -G, z0).union(caja(-G, G, -G, S[1], z0, z1), engine="manifold")
    return (m.intersection(region_a, engine="manifold"), m.difference(region_a, engine="manifold"))


entero = os.path.join(STL, "L2_brazo__estructura.stl")
fa, fb = (os.path.join(STL, f"L2_brazo__estructura_{k}.stl") for k in "ab")
if os.path.exists(entero):
    m = trimesh.load(entero)
    V0 = m.volume
    a, b = dividir(m)
    a.export(fa); b.export(fb)
    a.export(os.path.join(STL, "L2_brazo_a.3mf")); b.export(os.path.join(STL, "L2_brazo_b.3mf"))
    os.remove(entero)
    if os.path.exists(os.path.join(STL, "L2_brazo.3mf")):
        os.remove(os.path.join(STL, "L2_brazo.3mf"))
    print(f"Generadas las mitades PROVISORIAS a partir de L2 entero (V = {V0 / 1000:.1f} cm³). "
          "Volver a correr el script de Fusion las reemplaza por las definitivas.")
a, b = trimesh.load(fa), trimesh.load(fb)

# orientación de impresión: construcción a lo largo de y0. Mitad a apoyada en su cara y mínima;
# mitad b dada vuelta (apoyada en su cara y máxima) para que el solape quede arriba.
print("\nPieza        volumen   estanca  envolvente en la cama (x0 × z0) × alto   ¿entra?   soporte")
for nom, m, sentido in [("L2_brazo_a", a, +1), ("L2_brazo_b", b, -1)]:
    lo, hi = m.bounds
    ext = hi - lo
    base = lo[1] if sentido > 0 else hi[1]
    # caras que miran hacia la cama (normal · dirección de construcción < −cos 45°) y no están en ella
    n = m.face_normals[:, 1] * sentido
    altura = (m.triangles_center[:, 1] - base) * sentido
    sop = m.area_faces[(n < -np.cos(np.radians(45))) & (altura > 0.5)].sum()
    entra = (max(ext[0], ext[2]) <= cama[0] and min(ext[0], ext[2]) <= cama[1] and ext[1] <= cama[2])
    print(f"{nom:12s} {m.volume / 1000:6.1f} cm³  {str(m.is_watertight):6s}  "
          f"{ext[0]:5.1f} × {ext[2]:5.1f} × {ext[1]:4.1f} mm        {'sí' if entra else 'NO':3s}    "
          f"{sop / 100:5.1f} cm²")

# contacto del solape: caras horizontales (normal ±y0) en y0 = 0 dentro del solape
z0, z1 = S[2] + div["s_inicio"] * 1000, S[2] + div["s_fin"] * 1000
c = a.triangles_center
sel = (np.abs(a.face_normals[:, 1]) > 0.99) & (np.abs(c[:, 1]) < 1e-3) & (c[:, 2] > z0) & (c[:, 2] < z1)
print(f"\nÁrea de contacto del solape: {a.area_faces[sel].sum() / 100:.1f} cm² "
      f"({len(div['agujeros'])} agujeros: "
      + ", ".join(f"Ø{1000 * h['d']:.1f}" for h in div["agujeros"]) + ")")
uni = a.union(b, engine="manifold")
print(f"Volumen a + b = {(a.volume + b.volume) / 1000:.1f} cm³ | unión = {uni.volume / 1000:.1f} cm³ "
      f"(si coinciden, las mitades no se superponen)")
