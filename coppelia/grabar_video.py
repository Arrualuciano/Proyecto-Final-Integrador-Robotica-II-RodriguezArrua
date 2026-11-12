#!/usr/bin/env python3
"""grabar_video.py — Graba en MP4 la reproducción de la escena abierta en CoppeliaSim (celda_integrador.ttt).

Usa el modo paso a paso del cliente ZMQ y una cámara temporal: vista general mientras el AGV va de A a B y
acercamiento al brazo durante la toma y la colocación (transición suave de 1 s). 25 cuadros/s, 1280 × 720,
con un rótulo que indica la fase y el origen de los datos. Lo que se ve es el resultado de Octave
(reproduccion.csv), no un movimiento generado por CoppeliaSim.
Requiere: py -3.13 -m pip install imageio imageio-ffmpeg
Uso (con CoppeliaSim abierto y la escena cargada): py -3.13 coppelia/grabar_video.py
"""
import csv
import math
import os

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, ".."))
SALIDA = os.path.join(RAIZ, "videos", "simulacion_celda_coppelia.mp4")
ANCHO, ALTO, FPS = 1280, 720, 25

filas = [list(map(float, f)) for f in list(csv.reader(open(os.path.join(AQUI, "reproduccion.csv"))))[1:]]
t_csv = np.array([f[0] for f in filas])
fase = np.array([f[-1] for f in filas])
T_BRAZO = t_csv[np.argmax(fase == 2)]          # comienzo del ciclo del brazo
T_FIN = t_csv[-1]


def mirar(obj, desde):
    """Matriz 3x4 (lista de 12) de una cámara en `desde` que mira a `obj` (eje z de la cámara hacia adelante)."""
    obj, desde = np.asarray(obj, float), np.asarray(desde, float)
    z = (obj - desde) / np.linalg.norm(obj - desde)
    y = np.array([0, 0, 1.0]) - z[2] * z
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    R = np.column_stack([x, y, z])
    return [*R[0], desde[0], *R[1], desde[1], *R[2], desde[2]]


VISTA_AGV = (np.array([0.75, 0.40, 0.10]), np.array([0.75, 0.40, 0.10]) + 1.45 * np.array([1.0, -1.25, 1.0]))
VISTA_BRAZO = (np.array([1.33, 0.78, 0.20]), np.array([1.33, 0.78, 0.20]) + 0.80 * np.array([1.05, -1.05, 0.85]))


def vista(t):
    """Interpola entre la vista general y la del brazo alrededor del comienzo del ciclo del brazo."""
    s = min(max((t - (T_BRAZO - 0.5)) / 1.0, 0.0), 1.0)
    s = s * s * (3 - 2 * s)                     # suavizado
    obj = (1 - s) * VISTA_AGV[0] + s * VISTA_BRAZO[0]
    desde = (1 - s) * VISTA_AGV[1] + s * VISTA_BRAZO[1]
    return mirar(obj, desde)


def rotulo(t):
    if t < 1.0:
        return "Celda: AGV en la estación A con la pieza"
    if t < T_BRAZO:
        return "AGV A → B · pose real simulada (PI de ruedas + lazo de pose, odometría calibrada)"
    return "Brazo 6R · q real del lazo de par calculado + integral (juntas en modo cinemático)"


try:
    fuente = ImageFont.truetype("segoeui.ttf", 22)
    fuente_chica = ImageFont.truetype("segoeui.ttf", 18)
except OSError:
    fuente = fuente_chica = ImageFont.load_default()

client = RemoteAPIClient()
sim = client.require("sim")
sim.stopSimulation()
while sim.getSimulationState() != sim.simulation_stopped:
    pass
cam = sim.createVisionSensor(1, [ANCHO, ALTO, 0, 0], [0.05, 10.0, math.radians(50), 0.1, 0, 0, 0.92, 0.92, 0.95, 0, 0])
sim.setObjectInt32Param(cam, sim.visionintparam_perspective_operation, 1)
sim.setObjectFloatParam(cam, sim.visionfloatparam_perspective_angle, math.radians(50))
sim.setObjectInt32Param(cam, sim.visionintparam_resolution_x, ANCHO)
sim.setObjectInt32Param(cam, sim.visionintparam_resolution_y, ALTO)

os.makedirs(os.path.dirname(SALIDA), exist_ok=True)
escritor = imageio.get_writer(SALIDA, fps=FPS, codec="libx264", quality=8, macro_block_size=8)
client.setStepping(True)
sim.startSimulation()
cuadros = 0
paso = sim.getSimulationTimeStep()
cada = max(1, round(1 / (FPS * paso)))           # pasos de simulación por cuadro
k = 0
while True:
    t = sim.getSimulationTime()
    if k % cada == 0:
        sim.setObjectMatrix(cam, list(map(float, vista(t))), sim.handle_world)
        sim.handleVisionSensor(cam)
        img, res = sim.getVisionSensorImg(cam)
        cuadro = Image.fromarray(np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)[::-1])
        d = ImageDraw.Draw(cuadro)
        d.rectangle([0, ALTO - 44, ANCHO, ALTO], fill=(20, 33, 61))
        d.rectangle([0, 0, ANCHO, 40], fill=(245, 243, 238))
        d.text((16, ALTO - 38), rotulo(t), font=fuente, fill=(245, 243, 238))
        d.text((ANCHO - 210, 12), f"t = {t:5.1f} s", font=fuente_chica, fill=(20, 33, 61))
        d.text((16, 12), "Robótica II · Rodriguez – Arrua · CoppeliaSim 4.10 (resultado de Octave)", font=fuente_chica,
               fill=(20, 33, 61))
        escritor.append_data(np.asarray(cuadro))
        cuadros += 1
    if t >= T_FIN:
        break
    client.step()
    k += 1
sim.stopSimulation()
client.setStepping(False)
sim.removeObjects([cam])
escritor.close()
print(f"{cuadros} cuadros ({cuadros / FPS:.1f} s) en {SALIDA}")
