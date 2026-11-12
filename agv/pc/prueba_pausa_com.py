#!/usr/bin/env python3
"""prueba_pausa_com.py — Prueba de la pausa ante cortes de comunicación (firmware agv-0.2.2), ROBOT ELEVADO.

1) GOTO 0,5 m (modo 2); a 1,0 s la PC deja de mandar mensajes 1,5 s y después sigue: se espera
   EVT,PAUSA_COM,1 a los ~0,6 s del corte, la referencia congelada y las ruedas frenando, EVT,PAUSA_COM,0 al
   volver y el FIN normal.
2) GOTO de vuelta; corte de 4 s: se espera la pausa y, a los 3 s sin mensajes, EVT,FALLA,1 (comunicación).
La protección de batería se desactiva solo en RAM (lectura de A5 inválida). Guarda la telemetría en
agv/datos_lab/<fecha>_prueba_pausa_com/.
Uso: py -3.13 agv/pc/prueba_pausa_com.py [COM5]
"""
import csv, os, sys, time
import serial

PUERTO = sys.argv[1] if len(sys.argv) > 1 else "COM5"
RAIZ = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
CARPETA = os.path.join(RAIZ, "agv", "datos_lab", time.strftime("%Y%m%d-%H%M%S") + "_prueba_pausa_com")
CAMPOS = (["t_ms", "modo", "falla", "x_mm", "y_mm", "phi_mrad", "xr_mm", "yr_mm", "phir_mrad",
           "vx_mm_s", "vy_mm_s", "wz_mrad_s"] + [f"wr{i}_mrad_s" for i in range(4)] +
          [f"wm{i}_mrad_s" for i in range(4)] + [f"u{i}" for i in range(4)] + [f"c{i}" for i in range(4)] +
          ["vbat_mV", "ejec_us", "peor_periodo_us", "err_hall", "tx_perdidas"])


def cs(c):
    x = 0
    for ch in c:
        x ^= ord(ch)
    return f"${c}*{x:02X}\n".encode()


if PUERTO.lower().startswith("udp"):                 # por WiFi: "udp" o "udp:<ip del robot>"
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from agv_consola import EnlaceUDP
    s = EnlaceUDP(robot=PUERTO.split(":", 1)[1] if ":" in PUERTO else None)
    CARPETA += "_wifi"
else:
    s = serial.Serial(PUERTO, 115200, timeout=0.02)
    time.sleep(2.3)
    s.read(65536)
os.makedirs(CARPETA, exist_ok=True)
f_tel = open(os.path.join(CARPETA, "telemetria.csv"), "w", newline="", encoding="utf-8")
w = csv.writer(f_tel)
w.writerow(["t_pc_s", "prueba"] + CAMPOS)
eventos = []
buf = ""
T0 = time.time()


def leer(prueba):
    global buf
    buf += s.read(4096).decode("ascii", "replace")
    while "\n" in buf:
        l, buf = buf.split("\n", 1)
        l = l.strip()
        if not l.startswith("$"):
            continue
        cuerpo = l[1:].split("*")[0]
        t = round(time.time() - T0, 3)
        if cuerpo.startswith("TEL,"):
            d = cuerpo.split(",")[1:]
            if len(d) == len(CAMPOS):
                w.writerow([t, prueba] + d)
        elif not cuerpo.startswith("PONG"):
            eventos.append((t, prueba, cuerpo))
            if cuerpo.startswith(("EVT", "NAK")):
                print(f"{t:7.2f} s  [{prueba}] {cuerpo}", flush=True)


def mandar(o):
    s.write(cs(o))
    eventos.append((round(time.time() - T0, 3), "PC", o))


def correr(prueba, goto, t_corte, dur_corte, t_max):
    """Manda el GOTO, latido cada 100 ms salvo durante el corte; termina con FIN, FALLA o t_max."""
    print(f"--- {prueba}: {goto}, corte de {dur_corte} s a los {t_corte} s", flush=True)
    mandar(goto)
    t0 = time.time()
    ultimo = 0
    while time.time() - t0 < t_max:
        dt = time.time() - t0
        cortado = t_corte <= dt < t_corte + dur_corte
        if not cortado and time.time() - ultimo > 0.1:
            mandar("PING") if False else s.write(cs("PING"))
            ultimo = time.time()
        leer(prueba)
        if any(p == prueba and (c.startswith("EVT,FIN") or c.startswith("EVT,FALLA,1")) for _, p, c in eventos):
            break
    t_fin = time.time() + 0.5
    while time.time() < t_fin:
        if time.time() - ultimo > 0.1:
            s.write(cs("PING")); ultimo = time.time()
        leer(prueba)


for o in ["TEL,50", "SAFE,500,600,1500,0,6400", "CLR", "DIS", "ZERO", "ARM"]:
    mandar(o); time.sleep(0.1); leer("inicio")
correr("1_corte_1,5s", "GOTO,0.5,0,0,0,2", 1.0, 1.5, 12)
for o in ["CLR", "ARM"]:
    mandar(o); time.sleep(0.1); leer("inicio2")
correr("2_corte_4s", "GOTO,0,0,0,0,2", 1.0, 4.0, 12)
for o in ["CLR", "DIS", "SAFE,500,600,1500,1,6400", "TEL,20"]:
    mandar(o); time.sleep(0.1); leer("fin")
s.close(); f_tel.close()
with open(os.path.join(CARPETA, "eventos.csv"), "w", newline="", encoding="utf-8") as fh:
    ew = csv.writer(fh); ew.writerow(["t_pc_s", "prueba", "linea"]); ew.writerows(eventos)
print("Datos en", CARPETA)
