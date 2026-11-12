#!/usr/bin/env python3
"""
agv_consola.py — Consola de laboratorio para el AGV OSOYOO (firmware agv_osoyoo).

- Abre el puerto serie (115200 8N1), envía PING cada 100 ms (alimenta el watchdog del firmware)
  y agrega '$' y la suma XOR a cada orden.
- Muestra las respuestas ($ACK, $NAK, $EVT, $PAR, $HALL, $PONG); la telemetría $TEL se guarda en CSV
  y se resume en pantalla una vez por segundo.
- Cada sesión crea una carpeta en agv/datos_lab/AAAAMMDD-HHMMSS_<etiqueta>/ con:
    telemetria.csv   todas las líneas $TEL con hora de la PC
    eventos.csv      órdenes enviadas y respuestas recibidas, con hora
    ensayo.json      etiqueta, puerto, notas y parámetros leídos con GET al inicio
- Órdenes locales (no se envían al robot):
    !esperar <s>        pausa (útil en guiones)
    !esperar_fin <s>    espera el aviso EVT,FIN de un GOTO (o una FALLA), como mucho <s> segundos
    !pausa <texto>      muestra el texto y espera Enter (reubicar el robot, prender la cámara...)
    !medir <etiqueta>   pide la medición externa de la pose (marcas delantera y trasera del robot sobre
                        la cuadrícula del piso), calcula centro y orientación, la compara con la odometría
                        y la guarda en ensayo.json → "mediciones"
    !nota <texto>       agrega una nota al ensayo.json
    !guion <archivo>    ejecuta un archivo de órdenes, una por línea (# = comentario)
    !salir              DIS, cierra y guarda

Uso:  py -3.13 agv_consola.py COM8 --etiqueta identificacion_ruedas
Requiere: py -3.13 -m pip install pyserial
"""
import argparse
import csv
import datetime as dt
import json
import math
import os
import queue
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("Falta pyserial:  py -3.13 -m pip install pyserial")

CAMPOS_TEL = (["t_ms", "modo", "falla", "x_mm", "y_mm", "phi_mrad", "xr_mm", "yr_mm", "phir_mrad",
               "vx_mm_s", "vy_mm_s", "wz_mrad_s"]
              + [f"wr{i}_mrad_s" for i in range(4)] + [f"wm{i}_mrad_s" for i in range(4)]
              + [f"u{i}" for i in range(4)] + [f"c{i}" for i in range(4)]
              + ["vbat_mV", "ejec_us", "peor_periodo_us", "err_hall", "tx_perdidas"])
MODOS = ["DESARMADO", "QUIETO", "PWM", "PASO", "VEL", "IR_A", "FALLA"]
AQUI = os.path.dirname(os.path.abspath(__file__))


def con_suma(cuerpo: str) -> str:
    x = 0
    for ch in cuerpo:
        x ^= ord(ch)
    return f"${cuerpo}*{x:02X}\n"


def verificar(linea: str):
    """Devuelve el cuerpo sin '$' ni suma si es válido; None si la suma no coincide."""
    if not linea.startswith("$") or "*" not in linea:
        return None
    cuerpo, suma = linea[1:].rsplit("*", 1)
    x = 0
    for ch in cuerpo:
        x ^= ord(ch)
    try:
        return cuerpo if int(suma[:2], 16) == x else None
    except ValueError:
        return None


class EnlaceUDP:
    """Enlace WiFi con el robot (firmware agv-0.2.0 o posterior): misma interfaz que serial.Serial para
    write/read/close. Escucha en el puerto de la PC y aprende la dirección del robot del primer datagrama
    (el robot manda $HOLA,WIFI,<ip>,agv cada 1 s mientras nadie le habla por WiFi)."""

    def __init__(self, puerto_pc=4211, robot=None, puerto_robot=4210, espera_s=30.0):
        import socket
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(("0.0.0.0", puerto_pc))
        self.s.settimeout(0.05)
        self.robot = (robot, puerto_robot) if robot else None
        self.pendiente = b""
        t0 = time.time()
        print(f"Esperando al robot por WiFi (UDP {puerto_pc})...", flush=True)
        while self.robot is None and time.time() - t0 < espera_s:
            try:
                datos, dir_ = self.s.recvfrom(2048)
                self.robot, self.pendiente = dir_, datos
            except OSError:
                pass
        if self.robot is None:
            raise SystemExit("No llegó ningún aviso del robot por WiFi: ¿está prendido, en la misma red, "
                             "y el firewall de Windows deja pasar Python por UDP 4211?")
        print(f"Robot en {self.robot[0]}:{self.robot[1]}", flush=True)

    def write(self, datos):
        self.s.sendto(datos, self.robot)

    def read(self, n):
        if self.pendiente:
            d, self.pendiente = self.pendiente, b""
            return d
        try:
            d, dir_ = self.s.recvfrom(2048)
            return d if dir_[0] == self.robot[0] else b""
        except OSError:
            return b""

    def close(self):
        self.s.close()


class Consola:
    def __init__(self, puerto, etiqueta):
        if puerto.lower().startswith("udp"):         # "udp" o "udp:<ip del robot>"
            ip = puerto.split(":", 1)[1] if ":" in puerto else None
            self.ser = EnlaceUDP(robot=ip)
        else:
            self.ser = serial.Serial(puerto, 115200, timeout=0.05)
            time.sleep(2.0)                          # la Mega se reinicia al abrir el puerto
        sello = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.dir = os.path.join(AQUI, "..", "datos_lab", f"{sello}_{etiqueta}")
        os.makedirs(self.dir, exist_ok=True)
        self.f_tel = open(os.path.join(self.dir, "telemetria.csv"), "w", newline="", encoding="utf-8")
        self.w_tel = csv.writer(self.f_tel)
        self.w_tel.writerow(["t_pc_s"] + CAMPOS_TEL)
        self.f_evt = open(os.path.join(self.dir, "eventos.csv"), "w", newline="", encoding="utf-8")
        self.w_evt = csv.writer(self.f_evt)
        self.w_evt.writerow(["t_pc_s", "sentido", "linea"])
        self.meta = {"etiqueta": etiqueta, "puerto": puerto, "inicio": sello, "notas": [], "parametros": [],
                     "mediciones": []}
        self.t0 = time.time()
        self.vivo = True
        self.lock = threading.Lock()
        self.ultima_tel = None
        self.malas = 0
        self.fin = threading.Event()             # se activa con EVT,FIN o EVT,FALLA
        self.ultimo_evt = ""
        threading.Thread(target=self._leer, daemon=True).start()
        threading.Thread(target=self._latido, daemon=True).start()
        threading.Thread(target=self._resumen, daemon=True).start()

    def t(self):
        return round(time.time() - self.t0, 4)

    def enviar(self, cuerpo: str, registrar=True):
        with self.lock:
            self.ser.write(con_suma(cuerpo).encode("ascii"))
        if registrar:
            self.w_evt.writerow([self.t(), "PC->AGV", cuerpo])

    def _latido(self):
        while self.vivo:
            try:
                self.enviar("PING", registrar=False)
            except serial.SerialException:
                break
            time.sleep(0.1)

    def _leer(self):
        buf = b""
        while self.vivo:
            try:
                buf += self.ser.read(512)
            except serial.SerialException:
                print("\n[puerto cerrado]")
                break
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                linea = raw.decode("ascii", "replace").strip()
                if not linea:
                    continue
                cuerpo = verificar(linea)
                if cuerpo is None:
                    self.malas += 1
                    continue
                t = self.t()
                if cuerpo.startswith("TEL,"):
                    datos = cuerpo.split(",")[1:]
                    self.w_tel.writerow([t] + datos)
                    self.ultima_tel = datos
                elif cuerpo.startswith("PONG"):
                    pass
                else:
                    self.w_evt.writerow([t, "AGV->PC", cuerpo])
                    if cuerpo.startswith("PAR,"):
                        self.meta["parametros"].append(cuerpo)
                    if cuerpo.startswith("EVT,FIN") or (cuerpo.startswith("EVT,FALLA") and cuerpo != "EVT,FALLA,0"):
                        self.ultimo_evt = cuerpo
                        self.fin.set()
                    if not cuerpo.startswith("HOLA,WIFI"):   # el aviso de WiFi se repite cada 1 s
                        print(f"  < {cuerpo}")

    def _resumen(self):
        while self.vivo:
            time.sleep(1.0)
            d = self.ultima_tel
            if not d or len(d) < len(CAMPOS_TEL):
                continue
            v = dict(zip(CAMPOS_TEL, d))
            modo = MODOS[int(v["modo"])] if v["modo"].isdigit() and int(v["modo"]) < 7 else v["modo"]
            print(f"  [{modo} falla={v['falla']}] pose=({v['x_mm']},{v['y_mm']}) mm {v['phi_mrad']} mrad | "
                  f"w=({v['wm0_mrad_s']},{v['wm1_mrad_s']},{v['wm2_mrad_s']},{v['wm3_mrad_s']}) mrad/s | "
                  f"u=({v['u0']},{v['u1']},{v['u2']},{v['u3']}) | c=({v['c0']},{v['c1']},{v['c2']},{v['c3']}) | "
                  f"Vbat={v['vbat_mV']} mV | ejec={v['ejec_us']} us peor={v['peor_periodo_us']} us")

    def orden(self, linea: str):
        linea = linea.strip()
        if not linea or linea.startswith("#"):
            return True
        if linea.startswith("!"):
            partes = linea[1:].split(maxsplit=1)
            cmd = partes[0].lower()
            arg = partes[1] if len(partes) > 1 else ""
            if cmd == "esperar":
                time.sleep(float(arg or 1))
            elif cmd == "esperar_fin":
                ok = self.fin.wait(float(arg or 30))
                print(f"  [fin: {self.ultimo_evt}]" if ok else "  [ATENCION: no llegó EVT,FIN a tiempo]")
                self.fin.clear()
                time.sleep(0.5)                  # medio segundo más de telemetría con el robot quieto
            elif cmd == "pausa":
                input(f"  >>> {arg}  [Enter para seguir] ")
            elif cmd == "medir":
                self.medir(arg)
            elif cmd == "nota":
                self.meta["notas"].append({"t": self.t(), "texto": arg})
            elif cmd == "guion":
                with open(arg, encoding="utf-8") as fh:
                    for l in fh:
                        print(f"> {l.rstrip()}")
                        if not self.orden(l):
                            return False
            elif cmd == "salir":
                return False
            else:
                print("  orden local desconocida")
            return True
        cuerpo = ",".join(linea.replace(",", " ").split())
        if cuerpo.split(",")[0].upper() == "GOTO":
            self.fin.clear()                     # el próximo !esperar_fin espera ESTE movimiento
        self.enviar(cuerpo)
        return True

    def medir(self, etiqueta):
        """Medición externa: marcas delantera (D) y trasera (T) del robot, sobre su eje x y a la misma
        distancia del centro, medidas en la cuadrícula del piso (origen en la estación A)."""
        while True:
            txt = input(f"  [{etiqueta}] x_D y_D x_T y_T en mm (vacío = omitir): ").replace(",", " ").split()
            if not txt:
                print("  medición omitida")
                return
            try:
                xd, yd, xt, yt = map(float, txt)
                break
            except ValueError:
                print("  hacen falta 4 números")
        xc, yc = (xd + xt) / 2, (yd + yt) / 2
        phi = math.degrees(math.atan2(yd - yt, xd - xt))
        m = {"t": self.t(), "etiqueta": etiqueta, "marcas_mm": [xd, yd, xt, yt],
             "real": {"x_mm": round(xc, 1), "y_mm": round(yc, 1), "phi_deg": round(phi, 2)},
             "separacion_marcas_mm": round(math.hypot(xd - xt, yd - yt), 1)}
        d = self.ultima_tel
        if d and len(d) >= len(CAMPOS_TEL):
            v = dict(zip(CAMPOS_TEL, d))
            ox, oy, of = float(v["x_mm"]), float(v["y_mm"]), math.degrees(float(v["phi_mrad"]) / 1000)
            m["odometria"] = {"x_mm": ox, "y_mm": oy, "phi_deg": round(of, 2)}
            m["referencia"] = {"x_mm": float(v["xr_mm"]), "y_mm": float(v["yr_mm"]),
                               "phi_deg": round(math.degrees(float(v["phir_mrad"]) / 1000), 2)}
            print(f"  real ({xc:.0f}, {yc:.0f}) mm {phi:.1f}° | odometría ({ox:.0f}, {oy:.0f}) mm {of:.1f}° | "
                  f"deriva {math.hypot(xc - ox, yc - oy):.0f} mm")
        self.meta["mediciones"].append(m)
        with open(os.path.join(self.dir, "ensayo.json"), "w", encoding="utf-8") as fh:   # no perderla si algo falla
            json.dump(self.meta, fh, indent=2, ensure_ascii=False)

    def cerrar(self):
        try:
            self.enviar("DIS")
            time.sleep(0.2)
        except serial.SerialException:
            pass
        self.vivo = False
        time.sleep(0.2)
        self.meta["fin"] = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.meta["lineas_con_suma_invalida"] = self.malas
        with open(os.path.join(self.dir, "ensayo.json"), "w", encoding="utf-8") as fh:
            json.dump(self.meta, fh, indent=2, ensure_ascii=False)
        self.f_tel.close()
        self.f_evt.close()
        self.ser.close()
        print(f"Datos guardados en {os.path.abspath(self.dir)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("puerto", help="p. ej. COM5 (USB) o udp (WiFi; también udp:<ip del robot>)")
    ap.add_argument("--etiqueta", default="sesion")
    ap.add_argument("--guion", help="archivo de órdenes a ejecutar al inicio")
    a = ap.parse_args()
    c = Consola(a.puerto, a.etiqueta)
    c.orden("GET")
    time.sleep(0.5)
    try:
        if a.guion and not c.orden(f"!guion {a.guion}"):
            return
        print("Escribí órdenes (HELP en LEEME_FIRMWARE.md). Ctrl+C o !salir para terminar.")
        while True:
            if not c.orden(input("> ")):
                break
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        c.cerrar()


if __name__ == "__main__":
    main()
