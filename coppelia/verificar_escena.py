#!/usr/bin/env python3
"""verificar_escena.py — Comprueba la escena abierta en CoppeliaSim (celda_integrador.ttt):
  1) correspondencia de juntas: en instantes elegidos, q leída de J1..J6 = q de reproduccion.csv
     (la escena muestra el resultado del lazo de control, no otra cosa);
  2) correspondencia geométrica: el TCP calculado con la MCD de Python a partir de las juntas leídas
     coincide con la posición de la pieza mientras está en la pinza;
  3) capturas con una cámara temporal (coppelia/capturas/*.png) para revisar la escena.
Usa el modo paso a paso del cliente ZMQ (client.setStepping / client.step): resultados deterministas.
Uso: py -3.13 coppelia/verificar_escena.py
"""
import csv
import math
import os

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from escena_config import T_MUNDO_BASE, mcd

AQUI = os.path.dirname(os.path.abspath(__file__))
os.makedirs(os.path.join(AQUI, "capturas"), exist_ok=True)
filas = [list(map(float, f)) for f in list(csv.reader(open(os.path.join(AQUI, "reproduccion.csv"))))[1:]]

client = RemoteAPIClient()
sim = client.require("sim")
sim.stopSimulation()
while sim.getSimulationState() != sim.simulation_stopped:
    pass
alias = {sim.getObjectAlias(h): h for h in sim.getObjectsInTree(sim.handle_scene)}
juntas = [alias[f"J{i}"] for i in range(1, 7)]
modos = [sim.getJointMode(j)[0] for j in juntas]
print("Modo de las juntas:", ["cinemático" if m == sim.jointmode_kinematic else m for m in modos])
print("Paso de simulación:", sim.getSimulationTimeStep(), "s")

# cámara temporal: mira la celda desde arriba y de costado
cam = sim.createVisionSensor(1, [960, 600, 0, 0], [0.05, 10.0, math.radians(60), 0.1, 0, 0, 0.92, 0.92, 0.95, 0, 0])
obj = np.array([0.75, 0.45, 0.15]); pos = obj + 1.35 * np.array([1.2, -1.15, 1.05])
z = (obj - pos) / np.linalg.norm(obj - pos)
y = np.array([0, 0, 1.0]) - z[2] * z; y /= np.linalg.norm(y)
x = np.cross(y, z)
R = np.column_stack([x, y, z])
m = [*R[0], pos[0], *R[1], pos[1], *R[2], pos[2]]
sim.setObjectMatrix(cam, list(map(float, m)), sim.handle_world)
sim.setObjectInt32Param(cam, sim.visionintparam_perspective_operation, 1)
sim.setObjectFloatParam(cam, sim.visionfloatparam_perspective_angle, math.radians(45))
sim.setObjectInt32Param(cam, sim.visionintparam_resolution_x, 960)
sim.setObjectInt32Param(cam, sim.visionintparam_resolution_y, 600)


def captura(nombre):
    sim.handleVisionSensor(cam)
    img, res = sim.getVisionSensorImg(cam)
    a = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)[::-1]
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.imsave(os.path.join(AQUI, "capturas", nombre), a)


client.setStepping(True)
sim.startSimulation()
controles = [(0.5, "01_inicio.png"), (8.0, "02_agv_en_camino.png"), (15.4, "03_agv_en_B.png"),
             (19.5, "04_brazo_con_pieza.png"), (26.6, "05_pieza_colocada.png")]
for tc, nombre in controles:
    while sim.getSimulationTime() < tc - 1e-6:
        client.step()
    t = sim.getSimulationTime()
    k = min(int(round(t / 0.02)), len(filas) - 1)
    q_esc = np.array([sim.getJointPosition(j) for j in juntas])
    q_csv = np.array(filas[k][1:7])
    p_pz = np.array(sim.getObjectPosition(alias["Pieza"], sim.handle_world))
    p_agv = sim.getObjectPosition(alias["AGV"], sim.handle_world)
    Tt, _ = mcd(q_esc)
    tcp = (T_MUNDO_BASE @ Tt)[:3, 3]
    print(f"t = {t:5.2f} s | max|q_escena − q_csv| = {np.max(np.abs(q_esc - q_csv)):.1e} rad | "
          f"AGV ({p_agv[0]:.3f}, {p_agv[1]:.3f}) m | pieza ({p_pz[0]:.3f}, {p_pz[1]:.3f}, {p_pz[2]:.3f}) | "
          f"|TCP − pieza| = {1000 * np.linalg.norm(tcp - p_pz):.1f} mm")
    captura(nombre)
sim.stopSimulation()
client.setStepping(False)
sim.removeObjects([cam])
print("Capturas en", os.path.join(AQUI, "capturas"))
