#!/usr/bin/env python3
"""
robot_gui.py — Interfaz de operación del trabajo integrador (Robótica II, FCAL-UNER).

Pestaña BRAZO (modo virtual): configura controlador, ganancias, carga, perturbación y duración del ciclo;
  ejecuta la simulación en Octave (brazo/octave/correr_desde_gui.m) sin bloquear la ventana; muestra
  referencia vs. posición, error del TCP y pares con sus límites; guarda cada ensayo en interfaz/ensayos/brazo/.
Pestaña AGV (modo físico): conecta por WiFi (UDP, firmware agv-0.2.0 o posterior) o por USB con el firmware
  agv_osoyoo; latido cada 100 ms; ARM/DIS/STOP/CLR; movimiento manual mantenido, GOTO con modo de control,
  parámetros con rangos; telemetría en vivo; registro de ensayos (CSV + JSON) con medición externa. Piso de
  baldosas de 30 cm (origen A = último ZERO; en pantalla el auto arranca mirando hacia +y, la interfaz convierte
  al marco del robot, x adelante): un clic elige el destino del GOTO. El auto se dibuja en tiempo real desde la
  telemetría (20 Hz) y, en modo virtual, la simulación se reproduce a velocidad real.
  El modo virtual del AGV usa la simulación de Octave (agv/octave/correr_agv_desde_gui.m).

Uso:  py -3.13 interfaz/robot_gui.py
Requiere: PySide6, pyqtgraph, numpy, pyserial (py -3.13 -m pip install PySide6 pyqtgraph pyserial)
"""
import collections
import csv
import datetime as dt
import json
import os
import shutil
import socket
import sys
import threading
import time

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import QProcess, Qt, QTimer, Signal
import pyqtgraph as pg

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # la pestaña del brazo funciona igual
    serial = None

RAIZ = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
DIR_OCT = os.path.join(RAIZ, "brazo", "octave")
DIR_ENS = os.path.join(RAIZ, "interfaz", "ensayos")
P = json.load(open(os.path.join(RAIZ, "parametros", "parametros.json"), encoding="utf-8"))
COLORES = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]
pg.setConfigOptions(antialias=True, background="w", foreground="k")


def buscar_octave():
    candidatos = [r"C:\Program Files\GNU Octave\Octave-10.3.0\mingw64\bin\octave-cli.exe",
                  shutil.which("octave-cli") or "", shutil.which("octave-cli.exe") or ""]
    for c in candidatos:
        if c and os.path.exists(c):
            return c
    return None


def sello():
    return dt.datetime.now().strftime("%Y%m%d-%H%M%S")


# ----------------------------------------------------------------------------- campos con nombre, unidad y rango
class Campo:
    """Campo numérico: etiqueta 'nombre [unidad]', rango validado por el propio control y ayuda emergente."""

    def __init__(self, form, nombre, unidad, mn, mx, valor, dec=3, paso=None, ayuda=""):
        self.w = QtWidgets.QDoubleSpinBox()
        self.w.setRange(mn, mx)
        self.w.setDecimals(dec)
        self.w.setSingleStep(paso if paso else (mx - mn) / 100)
        self.w.setValue(valor)
        self.w.setKeyboardTracking(False)
        txt = f"Rango {mn:g} a {mx:g} {unidad}. {ayuda}".strip()
        self.w.setToolTip(txt)
        etiqueta = QtWidgets.QLabel(f"{nombre} [{unidad}]" if unidad else nombre)
        etiqueta.setToolTip(txt)
        form.addRow(etiqueta, self.w)

    @property
    def v(self):
        return float(self.w.value())

    def poner(self, valor):
        self.w.setValue(float(valor))


def grupo(titulo):
    g = QtWidgets.QGroupBox(titulo)
    f = QtWidgets.QFormLayout(g)
    f.setLabelAlignment(Qt.AlignRight)
    return g, f


# ============================================================================= BRAZO
class PestanaBrazo(QtWidgets.QWidget):
    CONTROLES = [("Par calculado + integral (mejora)", "cti"), ("Par calculado", "ct"), ("PD + gravedad", "pdg")]

    def __init__(self):
        super().__init__()
        c = P["brazo"]["control"]
        izq = QtWidgets.QVBoxLayout()

        g, f = grupo("Ensayo")
        self.id = QtWidgets.QLineEdit("brazo_01")
        self.id.setValidator(QtGui.QRegularExpressionValidator(QtCore.QRegularExpression(r"[A-Za-z0-9_\-]{1,40}")))
        f.addRow("Identificación", self.id)
        self.notas = QtWidgets.QLineEdit()
        f.addRow("Notas", self.notas)
        izq.addWidget(g)

        g, f = grupo("Controlador (Tema V)")
        self.ctrl = QtWidgets.QComboBox()
        for nom, _ in self.CONTROLES:
            self.ctrl.addItem(nom)
        f.addRow("Ley de control", self.ctrl)
        self.wn1 = Campo(f, "ωn ejes 1–3", "rad/s", 5, 60, c["wn"][0], 1, 1, "Kp = ωn², Kd = 2ζωn (escaladas por M_ii en PD+g).")
        self.wn2 = Campo(f, "ωn ejes 4–6", "rad/s", 5, 100, c["wn"][3], 1, 1, "La muñeca necesita más rigidez por su baja inercia.")
        self.zeta = Campo(f, "ζ", "—", 0.5, 2.0, c["zeta"], 2, 0.05, "Amortiguamiento del error.")
        self.fric = Campo(f, "Fricción conocida por el modelo", "fracción", 0, 1, c["friccion_conocida"], 2, 0.05,
                          "Error de modelo deliberado del par calculado.")
        izq.addWidget(g)

        g, f = grupo("Planta y carga")
        self.m_real = Campo(f, "Masa real de la pieza", "kg", 0, 0.4, P["pieza"]["masa"], 3, 0.01, "La que lleva la pinza.")
        self.m_mod = Campo(f, "Masa supuesta por el modelo", "kg", 0, 0.4, P["pieza"]["masa"], 3, 0.01, "La que usa el controlador.")
        self.esc = Campo(f, "Escala de masas reales de eslabones", "×", 0.7, 1.3, 1.0, 2, 0.05, "Incertidumbre de parámetros.")
        izq.addWidget(g)

        g, f = grupo("Perturbación de par")
        self.pert = QtWidgets.QCheckBox("Aplicar pulso")
        f.addRow(self.pert)
        self.p_j = Campo(f, "Junta", "—", 1, 6, 2, 0, 1)
        self.p_t0 = Campo(f, "Inicio", "s", 0, 30, 5.0, 2, 0.1)
        self.p_d = Campo(f, "Duración", "s", 0.01, 2, 0.2, 2, 0.05)
        self.p_a = Campo(f, "Amplitud", "N·m", -3, 3, 1.0, 2, 0.1)
        izq.addWidget(g)

        g, f = grupo("Trayectoria (Tema IV)")
        self.margen = Campo(f, "Margen sobre la duración mínima", "×", 1.0, 2.0, P["brazo"]["tarea"]["margen_tiempo"], 2, 0.05,
                            "1,10 usa la referencia ya calculada; otro valor la regenera (~1 min).")
        izq.addWidget(g)

        fila = QtWidgets.QHBoxLayout()
        self.b_sim = QtWidgets.QPushButton("▶ Simular")
        self.b_sim.setStyleSheet("font-weight:bold; padding:6px")
        self.b_can = QtWidgets.QPushButton("■ Detener")
        self.b_can.setEnabled(False)
        self.b_abr = QtWidgets.QPushButton("Cargar ensayo…")
        self.b_dir = QtWidgets.QPushButton("Abrir carpeta")
        for b in (self.b_sim, self.b_can, self.b_abr, self.b_dir):
            fila.addWidget(b)
        izq.addLayout(fila)
        self.acumular = QtWidgets.QCheckBox("Superponer con los ensayos anteriores (comparación)")
        izq.addWidget(self.acumular)
        self.barra = QtWidgets.QProgressBar()
        izq.addWidget(self.barra)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        izq.addWidget(self.log, 1)
        panel_izq = QtWidgets.QWidget()
        panel_izq.setLayout(izq)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(panel_izq)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(390)

        der = QtWidgets.QVBoxLayout()
        sel = QtWidgets.QHBoxLayout()
        sel.addWidget(QtWidgets.QLabel("Junta graficada:"))
        self.junta = QtWidgets.QComboBox()
        self.junta.addItems([f"J{i}" for i in range(1, 7)])
        self.junta.setCurrentIndex(1)
        sel.addWidget(self.junta)
        sel.addStretch(1)
        der.addLayout(sel)
        self.p_q = pg.PlotWidget(title="Posición articular: referencia (punteada) y simulada")
        self.p_q.setLabel("left", "q", "°")
        self.p_e = pg.PlotWidget(title="Error de posición del TCP (requisito en toma/colocación: 1 mm)")
        self.p_e.setLabel("left", "error", "mm")
        self.p_t = pg.PlotWidget(title="Par aplicado en la junta y límite del accionamiento")
        self.p_t.setLabel("left", "τ", "N·m")
        self.p_t.setLabel("bottom", "t", "s")
        for p in (self.p_q, self.p_e, self.p_t):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.addLegend(offset=(-10, 10))
            der.addWidget(p)
        self.p_e.setXLink(self.p_q)
        self.p_t.setXLink(self.p_q)
        self.resumen = QtWidgets.QTableWidget(0, 8)
        self.resumen.setHorizontalHeaderLabels(["Ensayo", "Ley", "e máx [mm]", "e RMS [mm]", "e pinza [mm]",
                                                "áng. pinza [°]", "τ pico J2 [N·m]", "sat. [%]"])
        self.resumen.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.resumen.setMaximumHeight(140)
        der.addWidget(self.resumen)

        lay = QtWidgets.QHBoxLayout(self)
        lay.addWidget(scroll)
        cont = QtWidgets.QWidget()
        cont.setLayout(der)
        lay.addWidget(cont, 1)

        self.proc = None
        self.ensayos = []          # [(nombre, ley, datos, metricas)]
        self.b_sim.clicked.connect(self.simular)
        self.b_can.clicked.connect(self.detener)
        self.b_abr.clicked.connect(self.cargar_dialogo)
        self.b_dir.clicked.connect(lambda: os.startfile(os.path.join(DIR_ENS, "brazo")) if os.path.isdir(os.path.join(DIR_ENS, "brazo")) else None)
        self.junta.currentIndexChanged.connect(self.graficar)

    # ---- simulación
    def config(self):
        ley = self.CONTROLES[self.ctrl.currentIndex()][1]
        return dict(id=self.id.text() or "brazo", notas=self.notas.text(), fecha=sello(), controlador=ley,
                    wn_123=self.wn1.v, wn_456=self.wn2.v, zeta=self.zeta.v, friccion_conocida=self.fric.v,
                    m_real=self.m_real.v, m_modelo=self.m_mod.v, escala_masa_planta=self.esc.v,
                    pert_activa=bool(self.pert.isChecked()), pert_junta=int(self.p_j.v), pert_t0=self.p_t0.v,
                    pert_dur=self.p_d.v, pert_amp=self.p_a.v, margen_tiempo=self.margen.v)

    def simular(self):
        octave = buscar_octave()
        if not octave:
            QtWidgets.QMessageBox.critical(self, "Octave", "No se encontró octave-cli.")
            return
        cfg = self.config()
        self.dir_actual = os.path.join(DIR_ENS, "brazo", f"{cfg['fecha']}_{cfg['id']}")
        os.makedirs(self.dir_actual, exist_ok=True)
        ruta = os.path.join(self.dir_actual, "config.json")
        json.dump(cfg, open(ruta, "w", encoding="utf-8"), indent=2, ensure_ascii=False)
        self.log.appendPlainText(f"— Ensayo {cfg['id']} ({cfg['controlador']}) → {self.dir_actual}")
        self.proc = QProcess(self)
        self.proc.setWorkingDirectory(DIR_OCT)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._salida)
        self.proc.finished.connect(self._fin)
        r = ruta.replace("\\", "/")
        self.proc.start(octave, ["--no-gui", "--eval", f"more off; correr_desde_gui('{r}')"])
        self.barra.setValue(0)
        self.b_sim.setEnabled(False)
        self.b_can.setEnabled(True)

    def detener(self):
        if self.proc:
            self.proc.kill()
            self.log.appendPlainText("Simulación detenida por el usuario.")

    def _salida(self):
        texto = bytes(self.proc.readAllStandardOutput()).decode("utf-8", "replace")
        for l in texto.splitlines():
            if l.startswith("PROGRESO"):
                self.barra.setValue(int(l.split()[1]))
            elif l.strip():
                self.log.appendPlainText(l)
        with open(os.path.join(self.dir_actual, "octave.log"), "a", encoding="utf-8") as fh:
            fh.write(texto)

    def _fin(self, codigo, _estado):
        self.b_sim.setEnabled(True)
        self.b_can.setEnabled(False)
        if codigo == 0 and os.path.exists(os.path.join(self.dir_actual, "resultado.csv")):
            self.barra.setValue(100)
            self.cargar(self.dir_actual)
        else:
            self.log.appendPlainText(f"La simulación terminó con código {codigo}. Ver octave.log.")

    # ---- resultados
    def cargar_dialogo(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Carpeta de ensayo", os.path.join(DIR_ENS, "brazo"))
        if d:
            self.cargar(d)

    def cargar(self, carpeta):
        try:
            datos = np.loadtxt(os.path.join(carpeta, "resultado.csv"), delimiter=",")
            met = json.load(open(os.path.join(carpeta, "metricas.json"), encoding="utf-8"))
            cfg = json.load(open(os.path.join(carpeta, "config.json"), encoding="utf-8"))
        except (OSError, ValueError) as e:
            self.log.appendPlainText(f"No se pudo cargar {carpeta}: {e}")
            return
        if not self.acumular.isChecked():
            self.ensayos.clear()
        self.ensayos.append((cfg["id"], cfg["controlador"], datos, met))
        self.ensayos = self.ensayos[-4:]
        self.graficar()
        self.tabla()

    def graficar(self):
        for p in (self.p_q, self.p_e, self.p_t):
            p.clear()
        j = self.junta.currentIndex()
        for k, (nom, ley, d, met) in enumerate(self.ensayos):
            col = COLORES[k % len(COLORES)]
            t = d[:, 0]
            if k == 0:
                self.p_q.plot(t, np.degrees(d[:, 1 + j]), pen=pg.mkPen("k", width=1, style=Qt.DashLine), name="referencia")
            self.p_q.plot(t, np.degrees(d[:, 7 + j]), pen=pg.mkPen(col, width=2), name=f"{nom} ({ley})")
            self.p_e.plot(t, d[:, 25], pen=pg.mkPen(col, width=2), name=f"{nom} ({ley})")
            self.p_t.plot(t, d[:, 19 + j], pen=pg.mkPen(col, width=2), name=f"{nom} ({ley})")
            if k == 0:
                lim = met["tau_limite_Nm"][j]
                for s in (1, -1):
                    self.p_t.addItem(pg.InfiniteLine(pos=s * lim, angle=0, pen=pg.mkPen("r", style=Qt.DashLine)))
                self.p_e.addItem(pg.InfiniteLine(pos=1.0, angle=0, pen=pg.mkPen("k", style=Qt.DotLine)))

    def tabla(self):
        self.resumen.setRowCount(len(self.ensayos))
        for r, (nom, ley, d, m) in enumerate(self.ensayos):
            vals = [nom, ley, m["e_max_mm"], m["e_rms_mm"], m["e_pinza_mm"], m["ang_pinza_deg"],
                    m["tau_pico_Nm"][1], m["saturacion_pct"]]
            for c, v in enumerate(vals):
                it = QtWidgets.QTableWidgetItem(v if isinstance(v, str) else f"{v:.2f}")
                self.resumen.setItem(r, c, it)


# ============================================================================= AGV
CAMPOS_TEL = (["t_ms", "modo", "falla", "x_mm", "y_mm", "phi_mrad", "xr_mm", "yr_mm", "phir_mrad",
               "vx_mm_s", "vy_mm_s", "wz_mrad_s"] + [f"wr{i}_mrad_s" for i in range(4)] +
              [f"wm{i}_mrad_s" for i in range(4)] + [f"u{i}" for i in range(4)] + [f"c{i}" for i in range(4)] +
              ["vbat_mV", "ejec_us", "peor_periodo_us", "err_hall", "tx_perdidas"])
MODOS = ["DESARMADO", "QUIETO", "PWM", "PASO", "VEL", "IR_A", "FALLA"]
FALLAS = {1: "comunicación", 2: "batería", 4: "rueda trabada", 8: "sobrevelocidad", 16: "período", 32: "parada", 64: "reinicio"}
RUEDAS = ["FL", "FR", "RL", "RR"]


# Marco de la PANTALLA del piso: el auto arranca en A mirando hacia +y (arriba), como se lo ve desde atrás en el
# laboratorio; x a la derecha. El firmware, la simulación y el informe usan el marco de la celda: x adelante,
# y a la izquierda. Conversión: x_robot = y_pantalla, y_robot = −x_pantalla; φ_pantalla = φ_robot + 90°.
def a_pantalla(x, y):
    return -np.asarray(y), np.asarray(x)


def a_robot(xs, ys):
    return ys, -xs


def con_suma(cuerpo):
    x = 0
    for ch in cuerpo:
        x ^= ord(ch)
    return f"${cuerpo}*{x:02X}\n"


def verificar(linea):
    if not linea.startswith("$") or "*" not in linea:
        return None
    cuerpo, s = linea[1:].rsplit("*", 1)
    x = 0
    for ch in cuerpo:
        x ^= ord(ch)
    try:
        return cuerpo if int(s[:2], 16) == x else None
    except ValueError:
        return None


class _SocketUDP:
    """Enlace WiFi con el firmware agv-0.2.0 o posterior (ESP8266 en UDP transparente). Escucha en el puerto
    4211 de la PC y aprende la dirección del robot del primer datagrama: el robot manda $HOLA,WIFI,<ip>,agv
    cada 1 s mientras nadie le habla por WiFi. Misma interfaz que serial.Serial (write/read/close)."""
    PUERTO_PC, PUERTO_ROBOT = 4211, 4210
    ARCHIVO_IP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ultimo_robot_ip.txt")

    def __init__(self, robot_ip=None):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.s.bind(("0.0.0.0", self.PUERTO_PC))
        self.s.settimeout(0.05)
        self.robot = (robot_ip, self.PUERTO_ROBOT) if robot_ip else None
        # El robot deja de mandar su aviso $HOLA en cuanto recibe una orden por WiFi (por ejemplo de otra
        # corrida). Mientras no se lo conoce, las órdenes van a la última IP conocida y por difusión: el robot
        # contesta siempre a la PC configurada y así se aprende su dirección.
        self.candidatos = [("255.255.255.255", self.PUERTO_ROBOT)]
        try:
            ip = open(self.ARCHIVO_IP, encoding="utf-8").read().strip()
            if ip:
                self.candidatos.insert(0, (ip, self.PUERTO_ROBOT))
        except OSError:
            pass

    def write(self, datos):
        for destino in ([self.robot] if self.robot else self.candidatos):
            try:
                self.s.sendto(datos, destino)
            except OSError:
                pass

    def read(self, _n):
        try:
            d, dir_ = self.s.recvfrom(2048)
        except socket.timeout:
            return b""
        except OSError:                 # Windows: ICMP "puerto inalcanzable" de un envío anterior
            return b""
        if self.robot is None:
            self.robot = dir_
            try:
                open(self.ARCHIVO_IP, "w", encoding="utf-8").write(dir_[0])
            except OSError:
                pass
        return d if dir_[0] == self.robot[0] else b""

    def close(self):
        self.s.close()


class Enlace(QtCore.QObject):
    """Puerto serie (USB) o WiFi (UDP) en un hilo propio; emite cada línea válida (sin '$' ni suma)."""
    linea = Signal(str)
    estado = Signal(str)

    def __init__(self):
        super().__init__()
        self.ser = None
        self.wifi = False
        self.vivo = False
        self.lock = threading.Lock()

    @property
    def abierto(self):
        return self.ser is not None

    def abrir(self, puerto):
        """puerto: 'COMn' (USB) o 'udp' / 'udp:<ip del robot>' (WiFi)."""
        if puerto.startswith("udp"):
            self.ser = _SocketUDP(puerto.split(":", 1)[1] if ":" in puerto else None)
            self.wifi = True
        else:
            self.ser = serial.Serial(puerto, 115200, timeout=0.05)
            self.wifi = False
        self.vivo = True
        threading.Thread(target=self._leer, daemon=True).start()

    def cerrar(self):
        self.vivo = False
        if self.ser:
            try:
                self.ser.close()
            except (OSError, AttributeError):
                pass
        self.ser = None

    def enviar(self, cuerpo):
        if not self.ser:
            return False
        try:
            with self.lock:
                self.ser.write(con_suma(cuerpo).encode("ascii"))
            return True
        except OSError as e:            # serial.SerialException deriva de OSError
            self.estado.emit(f"Error de escritura: {e}")
            return False

    def _leer(self):
        buf = b""
        avisado = False
        while self.vivo:
            try:
                buf += self.ser.read(512)
            except (OSError, AttributeError, TypeError):
                if self.vivo:
                    self.estado.emit("Enlace desconectado")
                break
            if self.wifi and not avisado and getattr(self.ser, "robot", None):
                avisado = True
                self.estado.emit(f"Robot encontrado por WiFi en {self.ser.robot[0]}")
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                c = verificar(raw.decode("ascii", "replace").strip())
                if c:
                    self.linea.emit(c)


class PestanaAGV(QtWidgets.QWidget):
    # recorridos del modo virtual: lista de misiones [x, y, phi (rad), T (s; 0 = mínima)]
    ENSAYOS_VIRTUALES = [("GOTO de los campos «Ir a pose»", None),
                         ("T1 avance longitudinal 1,0 m", [[1.0, 0, 0, 0]]),
                         ("T2 desplazamiento lateral 0,6 m", [[0, 0.6, 0, 0]]),
                         ("T3 giro de 90° sobre el centro", [[0, 0, 1.5707963, 0]]),
                         ("T4 trayecto A → B", [[1.2, 0, 0, 0], [1.2, 0.6, 0, 0]]),
                         ("T4 ida y vuelta A → B → A", [[1.2, 0, 0, 0], [1.2, 0.6, 0, 0], [1.2, 0, 0, 0], [0, 0, 0, 0]])]

    def __init__(self):
        super().__init__()
        self.virtual = False
        self.proc = None
        self.enlace = Enlace()
        self.enlace.linea.connect(self.recibir)
        self.enlace.estado.connect(lambda s: self.log.appendPlainText(s))
        self.buf = collections.deque(maxlen=1500)
        self.par = {}
        self.t_ping = 0.0
        self.rtt = None
        self.version = ""
        self.reg = None        # registro de ensayo en curso
        izq = QtWidgets.QVBoxLayout()

        g, f = grupo("Conexión")
        self.modo = QtWidgets.QComboBox()
        self.modo.addItems(["Físico (robot por WiFi o USB)", "Virtual (simulación en Octave)"])
        f.addRow("Modo", self.modo)
        fila = QtWidgets.QHBoxLayout()
        self.puerto = QtWidgets.QComboBox()
        self.b_ref = QtWidgets.QPushButton("↻")
        self.b_con = QtWidgets.QPushButton("Conectar")
        fila.addWidget(self.puerto, 1)
        fila.addWidget(self.b_ref)
        fila.addWidget(self.b_con)
        f.addRow("Puerto", fila)
        izq.addWidget(g)

        g = QtWidgets.QGroupBox("Seguridad")
        h = QtWidgets.QHBoxLayout(g)
        self.b_stop = QtWidgets.QPushButton("STOP (Esc)")
        self.b_stop.setStyleSheet("background:#c62828; color:white; font-weight:bold; padding:10px")
        h.addWidget(self.b_stop, 2)
        for nombre, cmd in (("ARM", "ARM"), ("DIS", "DIS"), ("CLR", "CLR")):
            b = QtWidgets.QPushButton(nombre)
            b.clicked.connect(lambda _=False, c=cmd: self.enviar(c))
            h.addWidget(b)
        izq.addWidget(g)
        self.ch_bat = QtWidgets.QCheckBox("Vigilar batería (lectura de A5)")
        self.ch_bat.setChecked(True)
        self.ch_bat.setToolTip("Desmarcar solo si la lectura de batería es inválida (divisor de A5 desconectado) y se\n"
                               "controla la batería con el tester. Se aplica en RAM: al reiniciar el robot vuelve lo guardado.")
        self.ch_bat.toggled.connect(self._bateria)
        izq.addWidget(self.ch_bat)

        g, f = grupo("Movimiento manual (mantener presionado)")
        self.j_v = Campo(f, "Velocidad lineal", "m/s", 0.02, 0.30, 0.10, 2, 0.01, "Se recorta a vmax en el firmware.")
        self.j_w = Campo(f, "Velocidad angular", "rad/s", 0.1, 2.0, 0.5, 2, 0.1)
        grilla = QtWidgets.QGridLayout()
        self.jog = {}
        for (txt, vx, vy, wz, r, c) in (("↖", 1, 1, 0, 0, 0), ("↑", 1, 0, 0, 0, 1), ("↗", 1, -1, 0, 0, 2),
                                        ("←", 0, 1, 0, 1, 0), ("⟲", 0, 0, 1, 1, 1), ("→", 0, -1, 0, 1, 2),
                                        ("↙", -1, 1, 0, 2, 0), ("↓", -1, 0, 0, 2, 1), ("↘", -1, -1, 0, 2, 2),
                                        ("⟳", 0, 0, -1, 3, 1)):
            b = QtWidgets.QPushButton(txt)
            b.setFixedHeight(32)
            b.pressed.connect(lambda a=(vx, vy, wz): self.jog_ini(a))
            b.released.connect(self.jog_fin)
            grilla.addWidget(b, r, c)
        f.addRow(grilla)
        izq.addWidget(g)

        g, f = grupo("Ir a pose (GOTO, ejes del piso: y adelante en A)")
        self.g_x = Campo(f, "x (derecha)", "m", -3, 3, -0.6, 3, 0.05,
                         "Marco del piso de la interfaz: en A el auto mira hacia +y. B = (−0,6; 1,2).")
        self.g_y = Campo(f, "y (adelante)", "m", -3, 3, 1.2, 3, 0.05)
        self.g_p = Campo(f, "giro φ", "°", -180, 180, 0, 1, 5, "Giro respecto de la orientación en A (+ = antihorario).")
        self.g_T = Campo(f, "Duración pedida", "s", 0, 60, 0, 1, 0.5, "0 = la mínima por límites; el firmware la alarga si hace falta.")
        self.g_m = QtWidgets.QComboBox()
        self.g_m.addItems(["2 · PI de ruedas + lazo de pose", "1 · PI de ruedas (pose abierta)", "0 · Abierto (solo prealimentación)"])
        f.addRow("Control", self.g_m)
        fila = QtWidgets.QHBoxLayout()
        b = QtWidgets.QPushButton("Ir")
        b.clicked.connect(self.ir_a)
        b2 = QtWidgets.QPushButton("Pose = 0 (ZERO)")
        b2.clicked.connect(lambda: self.enviar("ZERO"))
        fila.addWidget(b)
        fila.addWidget(b2)
        f.addRow(fila)
        izq.addWidget(g)

        g, f = grupo("Simulación en Octave (modo virtual)")
        self.v_ens = QtWidgets.QComboBox()
        for nom, _ in self.ENSAYOS_VIRTUALES:
            self.v_ens.addItem(nom)
        f.addRow("Recorrido", self.v_ens)
        self.b_vsim = QtWidgets.QPushButton("▶ Simular")
        self.b_vsim.setStyleSheet("font-weight:bold; padding:5px")
        self.b_vsim.clicked.connect(self.simular_virtual)
        self.v_res = QtWidgets.QLabel("Usa el modo de control de «Ir a pose», la carga del registro y los parámetros de abajo.")
        self.v_res.setWordWrap(True)
        f.addRow(self.b_vsim)
        f.addRow(self.v_res)
        self.g_virtual = g
        g.setEnabled(False)
        izq.addWidget(g)

        g, f = grupo("Parámetros (rigen desde el período siguiente al ACK)")
        self.kp = Campo(f, "Kp rueda", "PWM/(rad/s)", 0, 50, 8, 2, 0.5)
        self.ki = Campo(f, "Ki rueda", "PWM/rad", 0, 500, 80, 1, 5)
        self.kx = Campo(f, "K pose x", "1/s", 0, 10, 1.6, 2, 0.1)
        self.ky = Campo(f, "K pose y", "1/s", 0, 10, 1.6, 2, 0.1)
        self.kf = Campo(f, "K pose φ", "1/s", 0, 10, 2.4, 2, 0.1)
        self.vmax = Campo(f, "vmax", "m/s", 0.01, 0.5, P["agv"]["control"]["vmax"], 2, 0.01)
        self.amax = Campo(f, "amax", "m/s²", 0.05, 2.0, 0.30, 2, 0.05)
        self.wmax = Campo(f, "ωmax cuerpo", "rad/s", 0.1, 4.0, 1.0, 2, 0.1)
        self.almax = Campo(f, "αmax cuerpo", "rad/s²", 0.1, 10, 2.0, 2, 0.1)
        self.wrue = Campo(f, "ω máx de rueda", "rad/s", 1, 15, 10, 1, 0.5)
        self.ex = Campo(f, "ηx (real/odometría)", "—", 0.5, 1.5, 1.0, 3, 0.005)
        self.ey = Campo(f, "ηy", "—", 0.5, 1.5, 1.0, 3, 0.005)
        self.ew = Campo(f, "ηω", "—", 0.5, 1.5, 1.0, 3, 0.005)
        self.tel = Campo(f, "Telemetría", "Hz", 1, 50, 20, 0, 1)
        fila = QtWidgets.QHBoxLayout()
        for txt, fn in (("Aplicar", self.aplicar), ("Leer del robot", lambda: self.enviar("GET")),
                        ("Guardar en EEPROM", lambda: self.enviar("SAVE"))):
            b = QtWidgets.QPushButton(txt)
            b.clicked.connect(fn)
            fila.addWidget(b)
        f.addRow(fila)
        izq.addWidget(g)

        g, f = grupo("Registro de ensayo")
        self.e_id = QtWidgets.QLineEdit("agv_01")
        f.addRow("Identificación", self.e_id)
        self.e_carga = Campo(f, "Carga adicional", "kg", 0, 2, 0, 3, 0.05)
        self.e_piso = QtWidgets.QLineEdit("cerámico laboratorio")
        f.addRow("Piso", self.e_piso)
        self.e_op = QtWidgets.QLineEdit()
        f.addRow("Operador", self.e_op)
        self.e_notas = QtWidgets.QLineEdit()
        f.addRow("Notas", self.e_notas)
        self.b_reg = QtWidgets.QPushButton("● Iniciar registro")
        self.b_reg.clicked.connect(self.registro)
        f.addRow(self.b_reg)
        self.m_x = Campo(f, "x medido (externo)", "m", -5, 5, 0, 3, 0.005)
        self.m_y = Campo(f, "y medido", "m", -5, 5, 0, 3, 0.005)
        self.m_p = Campo(f, "φ medido", "°", -180, 180, 0, 1, 0.5)
        b = QtWidgets.QPushButton("Guardar medición externa en el último ensayo")
        b.clicked.connect(self.medicion)
        f.addRow(b)
        izq.addWidget(g)
        izq.addStretch(1)

        panel = QtWidgets.QWidget()
        panel.setLayout(izq)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(panel)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(400)

        der = QtWidgets.QVBoxLayout()
        self.estado = QtWidgets.QLabel("Desconectado")
        self.estado.setStyleSheet("font-size:13px; padding:4px; color:#111; background:#eceff1")
        der.addWidget(self.estado)
        # ---- piso de baldosas 30 x 30 cm en el marco de la celda (origen A = pose del último ZERO, x hacia
        # adelante del robot, y a su izquierda). Un clic elige el destino del GOTO (cualquier punto, no solo
        # las esquinas); se puede ajustar a 5 cm o a esquinas / centros de baldosa.
        barra = QtWidgets.QHBoxLayout()
        barra.addWidget(QtWidgets.QLabel("Clic en el piso = destino, ajuste:"))
        self.snap = QtWidgets.QComboBox()
        self.snap.addItems(["libre (1 cm)", "cada 5 cm", "esquina de baldosa", "centro de baldosa"])
        barra.addWidget(self.snap)
        self.ch_clic = QtWidgets.QCheckBox("enviar GOTO al hacer clic")
        self.ch_clic.setToolTip("Si no está marcado, el clic solo fija el destino y se envía con «Ir al destino».")
        barra.addWidget(self.ch_clic)
        self.l_dest = QtWidgets.QLabel()
        barra.addWidget(self.l_dest, 1)
        b = QtWidgets.QPushButton("Ir al destino")
        b.setStyleSheet("font-weight:bold")
        b.clicked.connect(self.ir_a)
        barra.addWidget(b)
        b = QtWidgets.QPushButton("Borrar trazo")
        b.clicked.connect(self.borrar_trazo)
        barra.addWidget(b)
        der.addLayout(barra)
        self.p_xy = pg.PlotWidget(title="Piso (baldosas de 30 cm): odometría, referencia, robot y destino")
        self.p_xy.setBackground("#f6f1e7")
        self.p_xy.setAspectLocked(True)
        self.p_xy.setLabel("bottom", "x (derecha del auto en A)", "m")
        self.p_xy.setLabel("left", "y (adelante del auto en A)", "m")
        self.p_xy.addLegend(offset=(-10, 10))
        self._dibujar_baldosas()
        self.c_xyr = self.p_xy.plot(pen=pg.mkPen("#777", width=1.5, style=Qt.DashLine), name="referencia")
        self.c_xy = self.p_xy.plot(pen=pg.mkPen("#1f77b4", width=2.5), name="odometría")
        self.c_xyv = self.p_xy.plot(pen=pg.mkPen("#2ca02c", width=2.5), name="real (solo simulación)")
        self.c_rob = self.p_xy.plot(pen=pg.mkPen("#0d47a1", width=2), connect="finite", name="robot")
        self.c_dst = self.p_xy.plot(pen=pg.mkPen("#e65100", width=1.5, style=Qt.DashLine), connect="finite",
                                    name="destino")
        self.c_dstp = self.p_xy.plot(symbol="+", symbolSize=16, symbolPen=pg.mkPen("#e65100", width=2), pen=None)
        self.p_xy.setRange(xRange=(-1.05, 0.75), yRange=(-0.35, 1.6), padding=0)
        self.p_xy.scene().sigMouseClicked.connect(self._clic_piso)
        for c in (self.g_x, self.g_y, self.g_p):
            c.w.valueChanged.connect(self._dibujar_destino)
        self._dibujar_destino()
        self.c_rob.setData(*self._contorno_robot(0, 0, 0))
        self.anim = None                                    # reproducción de la simulación virtual
        self.t_anim = QTimer(self, interval=40, timeout=self._animar)
        der.addWidget(self.p_xy, 3)
        self.p_w = pg.PlotWidget(title="Velocidad de ruedas: medida (línea) y referencia (punteada)")
        self.p_w.setLabel("left", "ω", "rad/s")
        self.p_w.addLegend(offset=(-10, 10))
        self.c_wm = [self.p_w.plot(pen=pg.mkPen(COLORES[i], width=2), name=RUEDAS[i]) for i in range(4)]
        self.c_wr = [self.p_w.plot(pen=pg.mkPen(COLORES[i], width=1, style=Qt.DashLine)) for i in range(4)]
        self.p_u = pg.PlotWidget(title="Comando PWM por rueda")
        self.p_u.setLabel("left", "u", "PWM")
        self.p_u.setLabel("bottom", "t", "s")
        self.c_u = [self.p_u.plot(pen=pg.mkPen(COLORES[i], width=2)) for i in range(4)]
        self.p_u.setXLink(self.p_w)
        for p in (self.p_w, self.p_u):
            p.showGrid(x=True, y=True, alpha=0.3)
        der.addWidget(self.p_w, 1)
        der.addWidget(self.p_u, 1)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(400)
        self.log.setMaximumHeight(120)
        der.addWidget(self.log)
        cont = QtWidgets.QWidget()
        cont.setLayout(der)
        lay = QtWidgets.QHBoxLayout(self)
        lay.addWidget(scroll)
        lay.addWidget(cont, 1)

        self.b_ref.clicked.connect(self.listar)
        self.b_con.clicked.connect(self.conectar)
        self.b_stop.clicked.connect(lambda: self.enviar("STOP"))
        QtGui.QShortcut(QtGui.QKeySequence(Qt.Key_Escape), self, activated=lambda: self.enviar("STOP"))
        self.modo.currentIndexChanged.connect(self._modo)
        self.t_lat = QTimer(self, interval=100, timeout=self._latido)
        self.t_jog = QTimer(self, interval=100, timeout=self._jog_tick)
        self.t_plot = QTimer(self, interval=50, timeout=self._graficar)   # el auto se redibuja a 20 Hz
        self.t_plot.start()
        self.jog_dir = None
        self.listar()

    # ---- conexión
    def _modo(self, i):
        """Físico: órdenes al Arduino por USB. Virtual: misma lógica de control simulada en Octave."""
        if i == 1 and self.enlace.abierto:
            QtWidgets.QMessageBox.warning(self, "Modo", "Desconectá el robot antes de pasar al modo virtual.")
            self.modo.setCurrentIndex(0)
            return
        self.virtual = (i == 1)
        for w in (self.puerto, self.b_ref, self.b_con):
            w.setEnabled(not self.virtual)
        self.g_virtual.setEnabled(self.virtual)
        self.estado.setText("MODO VIRTUAL · simulación en Octave (agv/octave/simular_agv.m)" if self.virtual
                            else "Desconectado")
        self.estado.setStyleSheet("font-size:13px; padding:4px; color:#111; background:%s" % ("#e8f5e9" if self.virtual else "#eceff1"))
        self.anim = None
        for c in [self.c_xy, self.c_xyr, self.c_xyv] + self.c_wm + self.c_wr + self.c_u:
            c.setData([], [])
        self.c_rob.setData(*self._contorno_robot(0, 0, 0))

    # ---- modo virtual
    def simular_virtual(self):
        octave = buscar_octave()
        if not octave:
            QtWidgets.QMessageBox.critical(self, "Octave", "No se encontró octave-cli.")
            return
        nom, mis = self.ENSAYOS_VIRTUALES[self.v_ens.currentIndex()]
        if mis is None:
            xr, yr = a_robot(self.g_x.v, self.g_y.v)
            mis = [[float(xr), float(yr), float(np.radians(self.g_p.v)), self.g_T.v]]
        cfg = dict(id=self.e_id.text() or "agv_virtual", fecha=sello(), recorrido=nom, misiones=mis,
                   modo=2 - self.g_m.currentIndex(), carga=self.e_carga.v,
                   eta_cal=[self.ex.v, self.ey.v, self.ew.v], kp=self.kp.v, ki=self.ki.v,
                   kx=self.kx.v, ky=self.ky.v, kphi=self.kf.v, vmax=self.vmax.v, amax=self.amax.v,
                   wmax=self.wmax.v, alfamax=self.almax.v, wrueda_max=self.wrue.v,
                   piso=self.e_piso.text(), operador=self.e_op.text(), notas=self.e_notas.text())
        self.v_dir = os.path.join(DIR_ENS, "agv_virtual", f"{cfg['fecha']}_{cfg['id']}")
        os.makedirs(self.v_dir, exist_ok=True)
        ruta = os.path.join(self.v_dir, "config.json")
        json.dump(cfg, open(ruta, "w", encoding="utf-8"), indent=2, ensure_ascii=False)
        self.log.appendPlainText(f"— Simulación virtual «{nom}», modo {cfg['modo']} → {self.v_dir}")
        self.proc = QProcess(self)
        self.proc.setWorkingDirectory(os.path.join(RAIZ, "agv", "octave"))
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        self.proc.readyReadStandardOutput.connect(
            lambda: self.log.appendPlainText(bytes(self.proc.readAllStandardOutput()).decode("utf-8", "replace").strip()))
        self.proc.finished.connect(self._v_fin)
        r = ruta.replace("\\", "/")
        self.proc.start(octave, ["--no-gui", "--eval", f"more off; correr_agv_desde_gui('{r}')"])
        self.b_vsim.setEnabled(False)
        self.v_res.setText("Simulando…")

    def _v_fin(self, codigo, _estado):
        self.b_vsim.setEnabled(True)
        if codigo != 0 or not os.path.exists(os.path.join(self.v_dir, "resultado.csv")):
            self.v_res.setText(f"La simulación terminó con código {codigo}.")
            return
        self.cargar_virtual(self.v_dir)

    def cargar_virtual(self, carpeta):
        d = np.loadtxt(os.path.join(carpeta, "resultado.csv"), delimiter=",")
        m = json.load(open(os.path.join(carpeta, "metricas.json"), encoding="utf-8"))
        t = d[:, 0]
        self.c_xyr.setData(*a_pantalla(d[:, 7], d[:, 8]))
        self.anim = dict(d=d, t0=time.time())          # el auto recorre la simulación en tiempo real (_animar)
        self.t_anim.start()
        for i in range(4):
            self.c_wr[i].setData(t, d[:, 10 + i])
            self.c_wm[i].setData(t, d[:, 14 + i])
            self.c_u[i].setData(t, d[:, 18 + i])
        txt = (f"Error final REAL {m['error_pos_real_mm']:.1f} mm / {m['error_ang_real_deg']:.2f}° · "
               f"deriva de odometría {m['deriva_odometria_mm']:.1f} mm · PWM máx {m['pwm_max']:.0f} · "
               f"duración {m['duracion_s']:.1f} s")
        self.v_res.setText(txt)
        self.estado.setText("MODO VIRTUAL · " + txt)

    def listar(self):
        self.puerto.clear()
        self.puerto.addItem("WiFi (UDP 4211, el robot avisa su IP)", "udp")
        if serial:
            for p in serial.tools.list_ports.comports():
                self.puerto.addItem(f"{p.device} — {p.description}", p.device)

    def conectar(self):
        if self.enlace.abierto:
            self.enviar("DIS")
            self.t_lat.stop()
            self.enlace.cerrar()
            self.b_con.setText("Conectar")
            self.estado.setText("Desconectado")
            return
        dev = self.puerto.currentData()
        if not dev:
            return
        if not serial and not dev.startswith("udp"):
            QtWidgets.QMessageBox.critical(self, "pyserial", "Instalar pyserial: py -3.13 -m pip install pyserial")
            return
        try:
            self.enlace.abrir(dev)
        except OSError as e:
            QtWidgets.QMessageBox.critical(self, "Puerto", str(e))
            return
        self.b_con.setText("Desconectar")
        self.hola = False
        if self.enlace.wifi:
            # el latido arranca ya: los PING se descartan hasta que llega el primer aviso $HOLA del robot
            self.log.appendPlainText("Esperando el aviso del robot por WiFi (UDP 4211)… Si no aparece: robot "
                                     "prendido, misma red y el firewall de Windows dejando pasar Python.")
            self.t_lat.start()
        else:
            self.log.appendPlainText(f"Conectado a {dev}. La Mega se reinicia al abrir el puerto (~2 s).")
            QTimer.singleShot(2200, lambda: (self.t_lat.start(), self.enviar("GET")))

    def enviar(self, cuerpo):
        if not self.enlace.abierto:
            return
        if self.enlace.enviar(cuerpo):
            if cuerpo != "PING" and not cuerpo.startswith("VEL"):
                self.log.appendPlainText(f"> {cuerpo}")
            if self.reg and cuerpo != "PING":
                self.reg["w_evt"].writerow([round(time.time() - self.reg["t0"], 4), "PC->AGV", cuerpo])

    def _latido(self):
        self.t_ping = time.time()
        self.enviar("PING")

    # ---- recepción
    def recibir(self, c):
        if c.startswith("TEL,"):
            d = c.split(",")[1:]
            if len(d) != len(CAMPOS_TEL):
                return
            v = dict(zip(CAMPOS_TEL, map(float, d)))
            self.buf.append(v)
            if self.reg:
                self.reg["w_tel"].writerow([round(time.time() - self.reg["t0"], 4)] + d)
                self.reg["ultima"] = v
            return
        if c.startswith("PONG"):
            self.rtt = (time.time() - self.t_ping) * 1000
            p = c.split(",")
            self.version = p[1] if len(p) > 1 else ""
            return
        if c.startswith("HOLA"):               # aviso WiFi del robot (cada 1 s hasta recibir una orden)
            if not getattr(self, "hola", False):
                self.hola = True
                self.log.appendPlainText(f"< {c}")
                self.enviar("GET")
            return
        if self.reg:
            self.reg["w_evt"].writerow([round(time.time() - self.reg["t0"], 4), "AGV->PC", c])
        if c.startswith("PAR,"):
            self._leer_par(c.split(",")[1:])
        if c.startswith("NAK"):
            self.log.appendPlainText(f"⚠ < {c}   (el robot rechazó la orden: ¿está armado y sin falla? CLR y ARM)")
            return
        if c.startswith("EVT,FALLA,") and c != "EVT,FALLA,0":
            bits = int(c.split(",")[2]) if c.split(",")[2].isdigit() else 0
            txt = ", ".join(n for k, n in FALLAS.items() if bits & k) or c
            extra = " (si la batería está bien, la lectura de A5 es inválida: desmarcar «Vigilar batería»)" if bits & 2 else ""
            self.log.appendPlainText(f"⚠ < {c}   FALLA: {txt}{extra}. Para seguir: CLR y ARM.")
            return
        self.log.appendPlainText(f"< {c}")

    def _leer_par(self, p):
        self.par[p[0] + (("_" + p[1]) if p[0] == "MDL" else "")] = p[1:]
        try:
            if p[0] == "GAIN":
                self.kp.poner(p[1]); self.ki.poner(p[2])
            elif p[0] == "KPOSE":
                self.kx.poner(p[1]); self.ky.poner(p[2]); self.kf.poner(p[3])
            elif p[0] == "LIM":
                self.vmax.poner(p[1]); self.amax.poner(p[2]); self.wmax.poner(p[3]); self.almax.poner(p[4]); self.wrue.poner(p[5])
            elif p[0] == "CAL":
                self.ex.poner(p[1]); self.ey.poner(p[2]); self.ew.poner(p[3])
            elif p[0] == "TEL":
                self.tel.poner(p[1])
            elif p[0] == "SAFE":
                self.ch_bat.blockSignals(True)
                self.ch_bat.setChecked(p[4] == "1")
                self.ch_bat.blockSignals(False)
        except (IndexError, ValueError):
            pass

    # ---- órdenes
    def aplicar(self):
        self.enviar(f"GAIN,{self.kp.v:.3f},{self.ki.v:.3f}")
        self.enviar(f"KPOSE,{self.kx.v:.3f},{self.ky.v:.3f},{self.kf.v:.3f}")
        self.enviar(f"LIM,{self.vmax.v:.3f},{self.amax.v:.3f},{self.wmax.v:.3f},{self.almax.v:.3f},{self.wrue.v:.2f}")
        self.enviar(f"CAL,{self.ex.v:.4f},{self.ey.v:.4f},{self.ew.v:.4f}")
        self.enviar(f"TEL,{int(self.tel.v)}")

    # ---- piso de baldosas
    BALDOSA = 0.30                       # m, lado de las baldosas del laboratorio

    def _dibujar_baldosas(self):
        """Juntas cada 30 cm (el origen A está sobre un cruce de juntas, como en los ensayos), ejes x/y desde A,
        el punto B del trayecto de ensayo y marcas del eje cada baldosa."""
        B = self.BALDOSA
        xs, ys = np.arange(-10, 11) * B, np.arange(-6, 17) * B
        px, py = [], []
        for x in xs:
            px += [x, x]; py += [ys[0], ys[-1]]
        for y in ys:
            px += [xs[0], xs[-1]]; py += [y, y]
        juntas = pg.PlotDataItem(px, py, connect="pairs", pen=pg.mkPen("#c9b89a", width=1.5))
        juntas.setZValue(-10)
        self.p_xy.addItem(juntas)
        for (dx, dy, color, txt, ang) in ((B, 0, "#c62828", "x", 180), (0, B, "#2e7d32", "y", -90)):
            self.p_xy.plot([0, dx], [0, dy], pen=pg.mkPen(color, width=3))
            self.p_xy.addItem(pg.ArrowItem(pos=(dx, dy), angle=ang, headLen=14, brush=color, pen=color))
            t = pg.TextItem(txt, color=color, anchor=(0, 1) if txt == "x" else (1.2, 0.5))
            t.setFont(QtGui.QFont("Segoe UI", 12, QtGui.QFont.Bold))
            t.setPos(dx + 0.02 if txt == "x" else dx, dy)
            self.p_xy.addItem(t)
        for (x, y, txt) in ((0, 0, "A"), (-0.6, 1.2, "B")):           # B = (1,2; 0,6) del marco del robot
            t = pg.TextItem(txt, color="#333", anchor=(1.2, -0.1))
            t.setFont(QtGui.QFont("Segoe UI", 11, QtGui.QFont.Bold))
            t.setPos(x, y)
            self.p_xy.addItem(t)
            self.p_xy.plot([x], [y], symbol="o", symbolSize=7, symbolBrush="#333", pen=None)
        marcas = [(v, f"{v:.1f}".replace(".", ",")) for v in np.arange(-10, 17) * B]
        for eje in ("bottom", "left"):
            self.p_xy.getAxis(eje).enableAutoSIPrefix(False)     # las marcas son fijas en m
            self.p_xy.getAxis(eje).setTicks([marcas, []])

    def _contorno(self, x, y, phi):
        """Silueta esquemática del robot (chasis, 4 ruedas en (±lx, ±ly) y flecha de avance) en la pose dada."""
        a = P["agv"]
        lx, ly, r, b = a["semidistancia_ejes_lx"], a["semitrocha_ly"], a["radio_rueda"], a["ancho_rueda"] / 2
        piezas = [[(-0.10, -ly + b), (0.10, -ly + b), (0.10, ly - b), (-0.10, ly - b), (-0.10, -ly + b)],
                  [(0.03, -0.04), (0.08, 0), (0.03, 0.04)]]
        for sx in (1, -1):
            for sy in (1, -1):
                cx, cy = sx * lx, sy * ly
                piezas.append([(cx - r, cy - b), (cx + r, cy - b), (cx + r, cy + b), (cx - r, cy + b), (cx - r, cy - b)])
        c, s = np.cos(phi), np.sin(phi)
        px, py = [], []
        for p in piezas:
            for (u, v) in p:
                px.append(x + c * u - s * v); py.append(y + s * u + c * v)
            px.append(np.nan); py.append(np.nan)
        return np.array(px), np.array(py)

    def _contorno_robot(self, x, y, phi):
        """Silueta para una pose del marco del robot (x adelante, y izquierda): se dibuja en el piso de la pantalla."""
        xs, ys = a_pantalla(x, y)
        return self._contorno(float(xs), float(ys), phi + np.pi / 2)

    def _animar(self):
        """Reproduce la simulación virtual en tiempo real: el auto avanza por la trayectoria real simulada."""
        if not self.anim:
            self.t_anim.stop()
            return
        d = self.anim["d"]
        k = int(np.searchsorted(d[:, 0], time.time() - self.anim["t0"]))
        k = min(max(k, 1), len(d))
        self.c_xyv.setData(*a_pantalla(d[:k, 1], d[:k, 2]))
        self.c_xy.setData(*a_pantalla(d[:k, 4], d[:k, 5]))
        self.c_rob.setData(*self._contorno_robot(d[k - 1, 1], d[k - 1, 2], d[k - 1, 3]))
        if k >= len(d):
            self.anim = None
            self.t_anim.stop()

    def _ajustar(self, x, y):
        B, i = self.BALDOSA, self.snap.currentIndex()
        if i == 0:
            return round(x, 2), round(y, 2)
        if i == 1:
            return round(x / 0.05) * 0.05, round(y / 0.05) * 0.05
        if i == 2:
            return round(x / B) * B, round(y / B) * B
        return (np.floor(x / B) + 0.5) * B, (np.floor(y / B) + 0.5) * B

    def _clic_piso(self, ev):
        if ev.button() != Qt.LeftButton:
            return
        vb = self.p_xy.getPlotItem().vb
        if not vb.sceneBoundingRect().contains(ev.scenePos()):
            return
        p = vb.mapSceneToView(ev.scenePos())
        x, y = self._ajustar(p.x(), p.y())
        self.g_x.poner(round(x, 3))
        self.g_y.poner(round(y, 3))
        if self.ch_clic.isChecked():
            self.ir_a()

    def _dibujar_destino(self, *_):
        x, y, f = self.g_x.v, self.g_y.v, np.radians(self.g_p.v)
        self.c_dst.setData(*self._contorno(x, y, f + np.pi / 2))    # en A el auto mira hacia +y
        self.c_dstp.setData([x], [y])
        B = self.BALDOSA
        self.l_dest.setText(f"destino ({x:.3f}; {y:.3f}) m, {self.g_p.v:.0f}°  =  "
                            f"{x / B:.2f} × {y / B:.2f} baldosas".replace(".", ","))

    def borrar_trazo(self):
        self.anim = None
        self.buf.clear()
        for c in [self.c_xy, self.c_xyr, self.c_xyv] + self.c_wm + self.c_wr + self.c_u:
            c.setData([], [])

    def ir_a(self):
        if self.virtual:
            self.log.appendPlainText("Modo virtual: el destino se simula con «▶ Simular» (recorrido «GOTO de los campos»).")
            return
        if not self._listo_para_mover():
            return
        modo = 2 - self.g_m.currentIndex()
        xr, yr = a_robot(self.g_x.v, self.g_y.v)                     # pantalla -> marco del robot
        self.enviar(f"GOTO,{xr:.4f},{yr:.4f},{np.radians(self.g_p.v):.4f},{self.g_T.v:.2f},{modo}")

    def _bateria(self, activa):
        """SAFE con los valores vigentes del robot, cambiando solo la vigilancia de batería (no se guarda)."""
        s = self.par.get("SAFE", ["500", "600", "1500", "1", "6400"])
        self.enviar(f"SAFE,{s[0]},{s[1]},{s[2]},{1 if activa else 0},{s[4]}")
        if not activa:
            self.log.appendPlainText("Protección de batería desactivada (solo en RAM): controlar la batería con el tester.")

    def _listo_para_mover(self):
        """Avisa si el robot no puede moverse (desarmado o en falla). Devuelve True si hay que enviar igual."""
        if not self.enlace.abierto:
            self.log.appendPlainText("No hay robot conectado: elegir el puerto (WiFi o COM) y «Conectar».")
            return False
        if self.buf:
            u = self.buf[-1]
            m = int(u["modo"])
            if m == 0:
                self.log.appendPlainText("El robot está DESARMADO: tocá ARM antes de moverlo.")
            elif m == 6:
                f = int(u["falla"])
                txt = ", ".join(n for k, n in FALLAS.items() if f & k) or "?"
                self.log.appendPlainText(f"El robot está en FALLA ({txt}): tocá CLR y después ARM."
                                         + (" Si la batería está bien, desmarcá «Vigilar batería»." if f & 2 else ""))
        return True

    def jog_ini(self, a):
        self._listo_para_mover()
        self.jog_dir = a
        self._jog_tick()
        self.t_jog.start()

    def jog_fin(self):
        self.t_jog.stop()
        self.jog_dir = None
        self.enviar("VEL,0,0,0")

    def _jog_tick(self):
        if self.jog_dir:
            vx, vy, wz = self.jog_dir
            n = (vx * vx + vy * vy) ** 0.5 or 1
            self.enviar(f"VEL,{self.j_v.v * vx / n:.3f},{self.j_v.v * vy / n:.3f},{self.j_w.v * wz:.3f}")

    # ---- registro de ensayos
    def registro(self):
        if self.reg:
            self._cerrar_registro()
            return
        nombre = f"{sello()}_{self.e_id.text() or 'agv'}"
        d = os.path.join(DIR_ENS, "agv", nombre)
        os.makedirs(d, exist_ok=True)
        f_tel = open(os.path.join(d, "telemetria.csv"), "w", newline="", encoding="utf-8")
        f_evt = open(os.path.join(d, "eventos.csv"), "w", newline="", encoding="utf-8")
        w_tel = csv.writer(f_tel)
        w_tel.writerow(["t_pc_s"] + CAMPOS_TEL)
        w_evt = csv.writer(f_evt)
        w_evt.writerow(["t_pc_s", "sentido", "linea"])
        self.reg = dict(dir=d, f_tel=f_tel, f_evt=f_evt, w_tel=w_tel, w_evt=w_evt, t0=time.time(), ultima=None,
                        meta=dict(id=self.e_id.text(), inicio=sello(), modo=self.modo.currentText(),
                                  puerto=self.puerto.currentData(), carga_kg=self.e_carga.v, piso=self.e_piso.text(),
                                  operador=self.e_op.text(), notas=self.e_notas.text(), firmware=self.version,
                                  parametros_robot=dict(self.par)))
        self.b_reg.setText("■ Detener registro")
        self.b_reg.setStyleSheet("background:#ffcdd2")
        self.log.appendPlainText(f"Registrando en {d}")
        self.enviar("GET")                 # los parámetros vigentes quedan en eventos.csv y en ensayo.json

    def _cerrar_registro(self):
        r = self.reg
        self.reg = None
        r["f_tel"].close()
        r["f_evt"].close()
        m = r["meta"]
        m["fin"] = sello()
        m["parametros_robot"] = dict(self.par)      # últimos valores informados por el firmware (PAR)
        m["firmware"] = self.version
        if r["ultima"]:
            u = r["ultima"]
            m["odometria_final"] = dict(x_m=u["x_mm"] / 1000, y_m=u["y_mm"] / 1000, phi_rad=u["phi_mrad"] / 1000)
        json.dump(m, open(os.path.join(r["dir"], "ensayo.json"), "w", encoding="utf-8"), indent=2, ensure_ascii=False)
        self.ultimo_dir = r["dir"]
        self.b_reg.setText("● Iniciar registro")
        self.b_reg.setStyleSheet("")
        self.log.appendPlainText(f"Ensayo guardado en {r['dir']}")

    def medicion(self):
        d = getattr(self, "ultimo_dir", None)
        if not d:
            QtWidgets.QMessageBox.information(self, "Medición", "Primero registrar y detener un ensayo.")
            return
        ruta = os.path.join(d, "ensayo.json")
        m = json.load(open(ruta, encoding="utf-8"))
        ext = dict(x_m=self.m_x.v, y_m=self.m_y.v, phi_rad=float(np.radians(self.m_p.v)))
        m["medicion_externa"] = ext
        if "odometria_final" in m:
            o = m["odometria_final"]
            dphi = np.arctan2(np.sin(ext["phi_rad"] - o["phi_rad"]), np.cos(ext["phi_rad"] - o["phi_rad"]))
            m["error_odometria"] = dict(posicion_m=float(np.hypot(ext["x_m"] - o["x_m"], ext["y_m"] - o["y_m"])),
                                        angulo_rad=float(dphi))
        json.dump(m, open(ruta, "w", encoding="utf-8"), indent=2, ensure_ascii=False)
        self.log.appendPlainText(f"Medición externa guardada en {ruta}")

    # ---- gráficos y estado
    def _graficar(self):
        if self.virtual or not self.buf:      # en modo virtual las curvas las fija cargar_virtual()
            return
        b = list(self.buf)
        u = b[-1]
        falla = int(u["falla"])
        txt_f = ", ".join(n for k, n in FALLAS.items() if falla & k) or "ninguna"
        rtt = f"{self.rtt:.0f} ms" if self.rtt is not None else "—"
        self.estado.setText(
            f"<b>{MODOS[int(u['modo'])] if 0 <= u['modo'] < 7 else u['modo']}</b> · falla: {txt_f} · "
            f"pose en el piso ({-u['y_mm']/1000:.3f}; {u['x_mm']/1000:.3f}) m, giro {np.degrees(u['phi_mrad']/1000):.1f}° · "
            f"Vbat {u['vbat_mV']/1000:.2f} V · lazo {u['ejec_us']:.0f} µs (peor período {u['peor_periodo_us']:.0f} µs) · "
            f"latencia {rtt} · err. Hall {u['err_hall']:.0f} · firmware {self.version}")
        t = np.array([v["t_ms"] for v in b]) / 1000
        self.c_rob.setData(*self._contorno_robot(u["x_mm"] / 1000, u["y_mm"] / 1000, u["phi_mrad"] / 1000))
        self.c_xy.setData(*a_pantalla([v["x_mm"] / 1000 for v in b], [v["y_mm"] / 1000 for v in b]))
        self.c_xyr.setData(*a_pantalla([v["xr_mm"] / 1000 for v in b], [v["yr_mm"] / 1000 for v in b]))
        for i in range(4):
            self.c_wm[i].setData(t, [v[f"wm{i}_mrad_s"] / 1000 for v in b])
            self.c_wr[i].setData(t, [v[f"wr{i}_mrad_s"] / 1000 for v in b])
            self.c_u[i].setData(t, [v[f"u{i}"] for v in b])

    def cerrar(self):
        if self.reg:
            self._cerrar_registro()
        if self.enlace.abierto:
            self.enviar("DIS")
            time.sleep(0.1)
            self.enlace.cerrar()


# ============================================================================= ventana principal
class Ventana(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robótica II — Trabajo integrador: brazo 6R y AGV mecanum")
        self.resize(1500, 950)
        tabs = QtWidgets.QTabWidget()
        self.brazo = PestanaBrazo()
        self.agv = PestanaAGV()
        tabs.addTab(self.brazo, "Brazo 6R (virtual)")
        tabs.addTab(self.agv, "AGV OSOYOO")
        self.setCentralWidget(tabs)
        self.statusBar().showMessage(f"Proyecto: {RAIZ}   ·   Octave: {buscar_octave() or 'no encontrado'}")

    def closeEvent(self, e):
        self.agv.cerrar()
        if self.agv.proc and self.agv.proc.state() != QProcess.NotRunning:
            self.agv.proc.kill()
        if self.brazo.proc and self.brazo.proc.state() != QProcess.NotRunning:
            self.brazo.proc.kill()
        e.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    v = Ventana()
    v.show()
    if "--prueba" in sys.argv:            # arranque automático para verificación: cierra a los 3 s
        QTimer.singleShot(3000, v.close)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
