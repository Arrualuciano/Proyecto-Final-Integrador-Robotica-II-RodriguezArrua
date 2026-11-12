#!/usr/bin/env python3
"""construir_escena.py — Arma la escena de CoppeliaSim 4.10 por la API remota ZMQ y la guarda como
coppelia/celda_integrador.ttt. Requiere CoppeliaSim abierto (el servidor ZMQ arranca solo).

Contenido:
  * Brazo 6R: STL exportados por Fusion (brazo/stl, mm, en la pose cero y en {0}), cadena
    L0 → J1 → L1 → … → J6 → L6; juntas de revolución en MODO CINEMÁTICO ubicadas en las tramas DH
    (eje z_{i-1}) y con los límites de parametros.json; base apoyada en un pedestal.
  * AGV OSOYOO: representación con primitivas (placas 200 × 155 mm, ruedas Ø80 × 36 mm en (±lx, ±ly)),
    agrupada bajo el marco 'AGV'.
  * Celda: estaciones A y B marcadas en el piso, mesa de trabajo con bandeja, pieza (cubo 40 mm).
  * Script de simulación (reproduccion.lua) que reproduce coppelia/reproduccion.csv.
Todas las formas son estáticas y no respondientes: no interviene el motor de física.
Uso: con CoppeliaSim abierto, py -3.13 coppelia/construir_escena.py [--verificar]
"""
import math
import os
import sys
import time

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from escena_config import BASE_POS, H_BANDEJA, LADO_PIEZA, P, RAIZ, T_MUNDO_BASE, cuaternion, mcd, rot_z

AQUI = os.path.dirname(os.path.abspath(__file__))
STL = os.path.join(RAIZ, "brazo", "stl")
ESCENA = os.path.join(AQUI, "celda_integrador.ttt")

client = RemoteAPIClient()
sim = client.require("sim")
v = sim.getInt32Param(sim.intparam_program_full_version)
print(f"CoppeliaSim {v // 1000000}.{(v // 10000) % 100}.{(v // 100) % 100}")
if v < 4060000:
    sys.exit("Se necesita CoppeliaSim 4.6 o posterior (API de scripts como objetos).")

sim.stopSimulation()
while sim.getSimulationState() != sim.simulation_stopped:
    time.sleep(0.1)
sim.closeScene()                                       # escena nueva (piso, luces y cámara por defecto)


def pose(T):
    return [*map(float, T[:3, 3]), *map(float, cuaternion(T[:3, :3]))]


def T_de(pos, R=None):
    T = np.eye(4)
    T[:3, 3] = pos
    if R is not None:
        T[:3, :3] = R
    return T


def estatica(h, color, alias):
    sim.setObjectAlias(h, alias)
    sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, list(map(float, color)))
    sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
    return h


def primitiva(tipo, tam, T, color, alias, padre=None):
    h = sim.createPrimitiveShape(tipo, list(map(float, tam)), 0)
    sim.setObjectPose(h, pose(T), sim.handle_world)
    estatica(h, color, alias)
    if padre is not None:
        sim.setObjectParent(h, padre, True)
    return h


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


CUBO, CIL = sim.primitiveshape_cuboid, sim.primitiveshape_cylinder
GRIS, NARANJA, OSCURO, AZUL, VERDE = [0.6, 0.6, 0.62], [0.95, 0.55, 0.1], [0.15, 0.15, 0.17], [0.2, 0.45, 0.85], [0.2, 0.7, 0.3]

# ------------------------------------------------------------------ celda
primitiva(CIL, [0.25, 0.25, 0.002], T_de([0, 0, 0.001]), VERDE, "Estacion_A")
primitiva(CIL, [0.25, 0.25, 0.002], T_de([1.2, 0.6, 0.001]), AZUL, "Estacion_B")
primitiva(CIL, [0.18, 0.18, H_BANDEJA], T_de([BASE_POS[0], BASE_POS[1], H_BANDEJA / 2]), [0.4, 0.4, 0.42], "Pedestal")
mesa = T_MUNDO_BASE @ T_de([0, 0.30, 0])
primitiva(CUBO, [0.30, 0.30, H_BANDEJA], T_de([mesa[0, 3], mesa[1, 3], H_BANDEJA / 2]), [0.75, 0.68, 0.55], "Mesa")
primitiva(CUBO, [0.10, 0.10, 0.004], T_de([mesa[0, 3], mesa[1, 3], H_BANDEJA + 0.002]), [0.3, 0.3, 0.8], "Bandeja_C")

# ------------------------------------------------------------------ brazo
raiz = sim.createDummy(0.03)
sim.setObjectAlias(raiz, "Brazo6R")
sim.setObjectPose(raiz, pose(T_MUNDO_BASE), sim.handle_world)
nombres = [e["nombre"] for e in P["brazo"]["cad"]["eslabones"]]
colores = [GRIS, NARANJA, GRIS, NARANJA, GRIS, NARANJA, GRIS]
estructura, motores = {}, {}
for nom, col in zip(nombres, colores):
    for archivo in sorted(os.listdir(STL)):
        if not archivo.startswith(nom + "__") or not archivo.endswith(".stl"):
            continue
        h = sim.importShape(0, os.path.join(STL, archivo), 0, 0.0001, 0.001)   # STL en mm -> m
        sim.setObjectParent(h, raiz, False)                                    # vértices en {0}
        cuerpo = archivo[len(nom) + 2:-4]
        es_estr = cuerpo.startswith("estructura")        # L2 viene en dos mitades: estructura_a y _b
        estatica(h, col if es_estr else OSCURO, nom if cuerpo == "estructura" or (es_estr and nom not in estructura)
                 else f"{nom}_{cuerpo}")
        if es_estr and nom not in estructura:
            estructura[nom] = h                          # primera pieza: eslabón de la cadena
        else:
            motores.setdefault(nom, []).append(h)        # demás piezas: solidarias a la primera
_, tramas = mcd(np.zeros(6))
lim = P["brazo"]["limites"]
juntas = []
for i in range(6):
    j = sim.createJoint(sim.joint_revolute_subtype, sim.jointmode_kinematic, 0, [0.07, 0.014])
    sim.setObjectAlias(j, f"J{i + 1}")
    sim.setObjectPose(j, pose(T_MUNDO_BASE @ tramas[i]), sim.handle_world)       # eje = z_{i-1} de DH
    qmin, qmax = math.radians(lim["q_min_deg"][i]), math.radians(lim["q_max_deg"][i])
    sim.setJointInterval(j, False, [qmin, qmax - qmin])
    sim.setJointPosition(j, 0.0)
    juntas.append(j)
# cadena cinemática: L0 -> J1 -> L1 -> ... -> J6 -> L6 (manteniendo la pose en el mundo)
sim.setObjectParent(estructura[nombres[0]], raiz, True)
for i in range(6):
    sim.setObjectParent(juntas[i], estructura[nombres[i]], True)
    sim.setObjectParent(estructura[nombres[i + 1]], juntas[i], True)
for nom, hs in motores.items():
    for h in hs:
        sim.setObjectParent(h, estructura[nom], True)

# ------------------------------------------------------------------ AGV
g = P["agv"]
lx, ly, r = g["semidistancia_ejes_lx"], g["semitrocha_ly"], g["radio_rueda"]
agv = sim.createDummy(0.03)
sim.setObjectAlias(agv, "AGV")
sim.setObjectPose(agv, pose(np.eye(4)), sim.handle_world)
primitiva(CUBO, [0.200, 0.155, 0.003], T_de([0, 0, 0.055]), OSCURO, "AGV_chasis_inferior", agv)
primitiva(CUBO, [0.200, 0.155, 0.003], T_de([0, 0, H_BANDEJA - 0.0015]), [0.1, 0.1, 0.1], "AGV_bandeja", agv)
for k, (sx, sy, nom) in enumerate([(1, 1, "FL"), (1, -1, "FR"), (-1, 1, "RL"), (-1, -1, "RR")]):
    primitiva(CIL, [2 * r, 2 * r, g["ancho_rueda"]], T_de([sx * lx, sy * ly, r], rot_x(math.pi / 2)), [0.12, 0.12, 0.12], f"AGV_rueda_{nom}", agv)
    primitiva(CIL, [0.025, 0.025, 0.06], T_de([sx * lx, sy * (ly - 0.05), r], rot_x(math.pi / 2)), [0.7, 0.7, 0.72], f"AGV_motor_{nom}", agv)
for sx, sy in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
    primitiva(CIL, [0.006, 0.006, H_BANDEJA - 0.057], T_de([sx * 0.085, sy * 0.065, (H_BANDEJA + 0.057) / 2]), [0.8, 0.7, 0.3], "AGV_columna", agv)
primitiva(CUBO, [0.10, 0.053, 0.012], T_de([-0.03, 0, H_BANDEJA + 0.006]), [0.1, 0.35, 0.6], "AGV_Mega2560", agv)

# ------------------------------------------------------------------ pieza
primitiva(CUBO, [LADO_PIEZA] * 3, T_de([0, 0, H_BANDEJA + LADO_PIEZA / 2]), [0.85, 0.15, 0.15], "Pieza")

# ------------------------------------------------------------------ script de reproducción
rep = sim.createDummy(0.02)
sim.setObjectAlias(rep, "Reproduccion")
codigo = open(os.path.join(AQUI, "reproduccion.lua"), encoding="utf-8").read()
scr = sim.createScript(sim.scripttype_simulation, codigo, 0, "lua")
sim.setObjectParent(scr, rep, True)
try:
    sim.setFloatParam(sim.floatparam_simulation_time_step, 0.02)
except Exception as e:  # noqa: BLE001
    print("Aviso: no se pudo fijar el paso de 20 ms:", e)

sim.saveScene(ESCENA)
print("Escena guardada en", ESCENA)
# recargarla: así la escena abierta tiene ruta y el script encuentra reproduccion.csv a su lado
sim.closeScene()
sim.loadScene(ESCENA)
print("Verificación: py -3.13 coppelia/verificar_escena.py (con la escena abierta en CoppeliaSim)")
