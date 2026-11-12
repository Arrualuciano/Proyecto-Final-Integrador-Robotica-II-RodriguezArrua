#!/usr/bin/env python3
"""corrida.py — Una corrida de ensayo en el piso (sesión 2): ZERO, ARM, GOTO(s) en el modo pedido, espera el fin,
DIS. Guarda la sesión de la consola (telemetría, eventos) en agv/datos_lab/<fecha>_<ensayo>_m<modo>_r<rep>/ y
muestra la odometría final. La medición externa se agrega después con --medir.

Uso:
  py -3.13 agv/pc/corrida.py T1 2 1                  # ensayo T1, modo 2, repetición 1 (por WiFi)
  py -3.13 agv/pc/corrida.py T4 0 1 --puerto COM5    # por USB
  py -3.13 agv/pc/corrida.py --medir T1 2 1 xD yD xT yT   # agrega la medición (mm, origen en A)
Ensayos (mismas referencias que la simulación, agv/octave/ensayos_agv.m):
  T1 avance 1,0 m | T2 lateral 0,6 m | T3 giro 90° | T4 y T5 A->B (1,2; 0) -> (1,2; 0,6)
"""
import csv, glob, json, math, os, subprocess, sys, tempfile

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, "..", ".."))
DATOS = os.path.join(RAIZ, "agv", "datos_lab")
METAS = {"T1": ["1.0 0 0"], "T2": ["0 0.6 0"], "T3": ["0 0 1.5708"],
         "T4": ["1.2 0 0", "1.2 0.6 0"], "T5": ["1.2 0 0", "1.2 0.6 0"], "T4C": ["1.2 0 0", "1.2 0.6 0"],
         # T5A: como T5 (con carga) pero con más aceleración (--amax 0.6) para que las cuatro ruedas despeguen casi
         # juntas al arrancar el tramo lateral (en T5 FR despegaba 150 ms tarde y el chasis giraba 10-12°)
         "T5A": ["1.2 0 0", "1.2 0.6 0"],
         # T5S: como T5 pero con rampa más suave (--amax 0.15): prueba si arrancar más despacio reduce el giro
         "T5S": ["1.2 0 0", "1.2 0.6 0"]}
TABLA = os.path.join(DATOS, "mediciones_piso.csv")


def ultima_carpeta(etq):
    c = sorted(glob.glob(os.path.join(DATOS, f"*_{etq}")))
    return c[-1] if c else None


def medir(ens, modo, rep, xd, yd, xt, yt):
    etq = f"{ens}_m{modo}_r{rep}"
    carpeta = ultima_carpeta(etq)
    tel = list(csv.DictReader(open(os.path.join(carpeta, "telemetria.csv"))))[-1]
    xc, yc = (xd + xt) / 2, (yd + yt) / 2
    phi = math.degrees(math.atan2(yd - yt, xd - xt))
    ox, oy, of = float(tel["x_mm"]), float(tel["y_mm"]), math.degrees(float(tel["phi_mrad"]) / 1000)
    meta = [float(v) for v in METAS[ens][-1].split()]
    fila = {"ensayo": ens, "modo": modo, "rep": rep, "carpeta": os.path.basename(carpeta),
            "meta_x_mm": meta[0] * 1000, "meta_y_mm": meta[1] * 1000, "meta_phi_deg": round(math.degrees(meta[2]), 2),
            "real_x_mm": xc, "real_y_mm": yc, "real_phi_deg": round(phi, 2),
            "odom_x_mm": ox, "odom_y_mm": oy, "odom_phi_deg": round(of, 2),
            "error_real_mm": round(math.hypot(xc - meta[0] * 1000, yc - meta[1] * 1000), 1),
            "error_real_deg": round(math.degrees(math.atan2(math.sin(math.radians(phi) - meta[2]), math.cos(math.radians(phi) - meta[2]))), 2),
            "deriva_odom_mm": round(math.hypot(xc - ox, yc - oy), 1),
            "vbat_mV": tel["vbat_mV"], "marcas_mm": f"{xd} {yd} {xt} {yt}"}
    nuevo = not os.path.exists(TABLA)
    with open(TABLA, "a", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(fila));
        if nuevo: w.writeheader()
        w.writerow(fila)
    print(f"{etq}: real ({xc:.0f}, {yc:.0f}) mm {phi:.1f}° | odometría ({ox:.0f}, {oy:.0f}) mm {of:.1f}° | "
          f"error real {fila['error_real_mm']} mm {fila['error_real_deg']}° | deriva odometría {fila['deriva_odom_mm']} mm")


def correr(ens, modo, rep, puerto, sin_bat=False, amax=0.30, wd=600):
    etq = f"{ens}_m{modo}_r{rep}"
    # --sin-bat: protección de batería desactivada durante la corrida (no se guarda en EEPROM). Se usó desde el
    # 11/11 17:54 porque la lectura de A5 quedó en ~2,2 V con la batería en 7,2 V (tester): divisor movido.
    b = "0" if sin_bat else "1"
    # modo 2: al llegar, el lazo de pose sigue corrigiendo hasta 5 mm / 0,57° o hasta T + 2 s; en el piso las ruedas
    # se pegan con errores de ~0,6° y la traba (1,5 s) cortaba antes de los 2 s (T4C_m2_r2, 11/11 18:09).
    # Durante la corrida la traba pasa a 2,5 s: si no logra la tolerancia, termina por tiempo con FIN.
    traba = 2500 if str(modo) == "2" else 1500
    # --wd <ms>: watchdog de comunicación durante la corrida (600 ms guardado). Desde T5A (11/11 18:30) se usó
    # 1500 ms porque la red del laboratorio tenía huecos de 300-1200 ms; el operador junto al robot.
    lineas = ["TEL 50", f"SAFE 500 {wd} {traba} {b} 6400", "CLR", "DIS", f"LIM 0.25 {amax:.2f} 1.0 2.0 10", "ZERO", "ARM",
              f"!nota {etq}" + (" (sin proteccion de bateria: lectura de A5 invalida)" if sin_bat else "")]
    if str(modo) == "0":
        # lazo abierto: a baja velocidad las ruedas se pegan al piso y nada lo corrige; con la traba normal
        # (1,5 s) la corrida se corta. Solo durante la corrida se lleva a 5 s (no se guarda en EEPROM).
        lineas.insert(5, f"SAFE 500 {wd} 5000 {b} 6400")
    for m in METAS[ens]:
        lineas += [f"GOTO {m} 0 {modo}", "!esperar_fin 30"]
    lineas += ["DIS", f"SAFE 500 600 1500 {b} 6400", "LIM 0.25 0.30 1.0 2.0 10", "TEL 20"]
    g = tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False, encoding="utf-8")
    g.write("\n".join(lineas) + "\n"); g.close()
    p = subprocess.run(["py", "-3.13", os.path.join(AQUI, "agv_consola.py"), puerto, "--etiqueta", etq, "--guion", g.name],
                       input="!esperar 1\n!salir\n", capture_output=True, text=True, encoding="utf-8", cwd=RAIZ)
    os.remove(g.name)
    for l in p.stdout.splitlines():
        if any(k in l for k in ("EVT", "NAK", "ATENCION", "Robot en")):
            print(l.strip())
    if p.returncode:
        print(p.stderr[-600:])
    carpeta = ultima_carpeta(etq)
    if not carpeta:
        sys.exit("No se registró la corrida (sin enlace con el robot).")
    tel = list(csv.DictReader(open(os.path.join(carpeta, "telemetria.csv"))))
    if not tel:
        sys.exit(f"Sin telemetría: el robot no respondió ({os.path.basename(carpeta)}).")
    u = max(abs(int(r[f"u{i}"])) for r in tel for i in range(4))
    f = tel[-1]
    print(f"{etq}: odometría final ({f['x_mm']}, {f['y_mm']}) mm {math.degrees(float(f['phi_mrad']) / 1000):.1f}° | "
          f"PWM máx {u} | V_bat {f['vbat_mV']} mV | fallas {sorted({r['falla'] for r in tel})} | {os.path.basename(carpeta)}")


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "--medir":
        medir(a[1], a[2], a[3], *map(float, a[4:8]))
    elif a and a[0] == "--medir-baldosas":
        # x_centro, y_delantero, y_trasero [mm]: y medidos perpendiculares a la junta de partida (eje x), en los
        # centros de los bordes delantero y trasero del chasis (separados L_CHASIS); se convierten a marcas D/T
        L = 200.0
        xc, yd, yt = map(float, a[4:7])
        phi = math.atan2(yd - yt, L)
        medir(a[1], a[2], a[3], xc + L / 2 * math.cos(phi), yd, xc - L / 2 * math.cos(phi), yt)
    else:
        puerto = a[a.index("--puerto") + 1] if "--puerto" in a else "udp"
        amax = float(a[a.index("--amax") + 1]) if "--amax" in a else 0.30
        wd = int(a[a.index("--wd") + 1]) if "--wd" in a else 600
        correr(a[0], a[1], a[2], puerto, sin_bat="--sin-bat" in a, amax=amax, wd=wd)
