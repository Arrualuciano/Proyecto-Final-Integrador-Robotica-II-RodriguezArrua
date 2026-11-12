#!/usr/bin/env python3
"""calcular_cal.py — Factores de calibración de la odometría a partir de los ensayos T1, T2 y T3.

Lee las mediciones externas (!medir) de las sesiones de agv_consola.py y calcula
    eta_x = x_real / x_odometría    (T1, avance)
    eta_y = y_real / y_odometría    (T2, lateral)
    eta_w = φ_real / φ_odometría    (T3, giro)
promediando las repeticiones en el modo indicado (por defecto 2). Los ensayos tienen que haberse hecho
con CAL 1 1 1 (los guiones piso_T1..T3 lo fijan). Imprime la orden CAL para el firmware.

Uso:  py -3.13 agv/pc/calcular_cal.py <carpeta T1> <carpeta T2> <carpeta T3> [--modo 2]
      (las carpetas están en agv/datos_lab/)
"""
import argparse
import json
import os
import statistics as st

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("t1"); ap.add_argument("t2"); ap.add_argument("t3")
ap.add_argument("--modo", type=int, default=2)
a = ap.parse_args()

eta = []
for carpeta, eje, nombre in [(a.t1, "x_mm", "eta_x (T1 avance)"), (a.t2, "y_mm", "eta_y (T2 lateral)"),
                             (a.t3, "phi_deg", "eta_w (T3 giro)")]:
    meta = json.load(open(os.path.join(carpeta, "ensayo.json"), encoding="utf-8"))
    cal = [p for p in meta.get("parametros", []) if p.startswith("PAR,CAL")]
    r = []
    for m in meta.get("mediciones", []):
        if f"_m{a.modo}_" not in m["etiqueta"] or "odometria" not in m:
            continue
        o = m["odometria"][eje]
        if abs(o) < 1e-6:
            continue
        r.append(m["real"][eje] / o)
        print(f"  {m['etiqueta']:12s} real {m['real'][eje]:9.2f} | odometría {o:9.2f} | cociente {r[-1]:.4f}")
    if not r:
        raise SystemExit(f"No hay mediciones en modo {a.modo} en {carpeta}")
    e = st.mean(r)
    dispersion = f" ± {st.stdev(r):.4f}" if len(r) > 1 else ""
    print(f"{nombre}: {e:.4f}{dispersion}  (n = {len(r)}; CAL al inicio de la sesión: {cal[-1] if cal else '?'})\n")
    eta.append(e)

print(f"Orden para el firmware:\nCAL {eta[0]:.4f} {eta[1]:.4f} {eta[2]:.4f}")
