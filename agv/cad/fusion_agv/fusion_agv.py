# -*- coding: utf-8 -*-
"""
fusion_agv.py — Genera la representación CAD del AGV OSOYOO M2.0 en Autodesk Fusion a partir de
parametros/parametros.json (bloque 'agv'), con la geometría RELEVADA en el laboratorio.

Qué hace:
  1. Lee radio y ancho de rueda, l_x y l_y medidos (11/11/2026), placa del chasis y altura de la bandeja.
  2. Crea un documento nuevo con:
     - componente 'Chasis' (fijo): placa inferior, bandeja superior, 4 columnas, 4 motorreductores GM25-370,
       Arduino Mega 2560 con el shield ESP8266 y portapilas 2 x 18650;
     - 4 componentes 'Rueda_FL/FR/RL/RR' (Ø80 x 36 mm) en (±l_x, ±l_y), cada una con una articulación de
       revolución sobre su eje (as-built joint), en el orden de ruedas del modelo (1 FL, 2 FR, 3 RL, 4 RR).
     Marco del AGV = el del modelo cinemático y del firmware: origen en el centro geométrico al nivel del piso,
     x hacia adelante, y a la izquierda, z hacia arriba.
  3. Exporta agv/cad/agv_osoyoo.step, un STL por cuerpo en agv/cad/stl/ y agv/cad/propiedades_fusion_agv.csv.

Ubicaciones: ruedas, motores, placas y columnas como en la escena de CoppeliaSim (coppelia/construir_escena.py);
Mega y portapilas en posición APROXIMADA (relevadas a ojo, sin medir). Es una representación para ubicar
componentes y verificar la geometría, no un CAD de fabricación del kit.

Cómo ejecutarlo: Fusion > UTILIDADES > COMPLEMENTOS > Scripts y complementos (Shift+S) > '+' junto a
'Mis scripts' > elegir la carpeta 'fusion_agv' > seleccionarlo y 'Ejecutar'.
Unidades: el JSON está en metros; la API de Fusion trabaja en centímetros.
"""
import adsk.core
import adsk.fusion
import traceback
import json
import os
import re
import csv

M2CM = 100.0
H_BANDEJA = 0.090          # m, altura de la placa superior (igual que coppelia/escena_config.py) [E]
Z_PLACA_INF = 0.055        # m, placa inferior                                                       [E]
E_PLACA = 0.003            # m, espesor de las placas                                                [E]


def P3(p):
    return adsk.core.Point3D.create(p[0] * M2CM, p[1] * M2CM, p[2] * M2CM)


def V3(v):
    return adsk.core.Vector3D.create(v[0], v[1], v[2])


def caja(tbm, centro, tam):
    obb = adsk.core.OrientedBoundingBox3D.create(P3(centro), V3([1, 0, 0]), V3([0, 1, 0]),
                                                 tam[0] * M2CM, tam[1] * M2CM, tam[2] * M2CM)
    return tbm.createBox(obb)


def cilindro(tbm, p1, p2, r):
    return tbm.createCylinderOrCone(P3(p1), r * M2CM, P3(p2), r * M2CM)


def buscar_material(app, claves):
    todos = []
    for lib in app.materialLibraries:
        try:
            todos.extend(list(lib.materials))
        except Exception:
            continue
    for clave in claves:
        for m in todos:
            if clave.lower() in re.split(r'[\s,()\-/]+', m.name.lower()):
                return m
    return None


def componente(root, nombre, cuerpos, materiales):
    """cuerpos: lista de (nombre, BRepBody temporal); materiales: dict prefijo -> material."""
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = nombre
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    for n, b in cuerpos:
        comp.bRepBodies.add(b, bf).name = n
    bf.finishEdit()
    for i in range(comp.bRepBodies.count):
        body = comp.bRepBodies.item(i)
        for pref, mat in materiales.items():
            if body.name.startswith(pref) and mat is not None:
                try:
                    body.material = mat
                except Exception:
                    pass
                break
    return occ


def cara_cilindrica_mayor(occ):
    mejor = None
    for i in range(occ.bRepBodies.count):
        for f in occ.bRepBodies.item(i).faces:
            if f.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                if mejor is None or f.area > mejor.area:
                    mejor = f
    return mejor


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        aqui = os.path.dirname(os.path.realpath(__file__))
        proyecto = os.path.abspath(os.path.join(aqui, '..', '..', '..'))
        with open(os.path.join(proyecto, 'parametros', 'parametros.json'), encoding='utf-8') as fh:
            g = json.load(fh)['agv']
        r, ancho = g['radio_rueda'], g['ancho_rueda']
        lx, ly = g['semidistancia_ejes_lx'], g['semitrocha_ly']
        px, py = g['placa_chasis']

        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        root = design.rootComponent
        tbm = adsk.fusion.TemporaryBRepManager.get()

        metal = buscar_material(app, ['Aluminum', 'Aluminio', 'Steel', 'Acero'])
        acero = buscar_material(app, ['Steel', 'Acero'])
        goma = buscar_material(app, ['Rubber', 'Caucho', 'Goma'])
        plastico = buscar_material(app, ['ABS', 'Plastic', 'Plástico'])
        log = []

        # ---- chasis (fijo)
        c = []
        c.append(('placa_inferior', caja(tbm, [0, 0, Z_PLACA_INF], [px, py, E_PLACA])))
        c.append(('placa_bandeja', caja(tbm, [0, 0, H_BANDEJA - E_PLACA / 2], [px, py, E_PLACA])))
        for sx in (1, -1):
            for sy in (1, -1):
                c.append(('columna', cilindro(tbm, [sx * 0.085, sy * 0.065, Z_PLACA_INF + E_PLACA / 2],
                                              [sx * 0.085, sy * 0.065, H_BANDEJA - E_PLACA], 0.003)))
        for nom, sx, sy in (('FL', 1, 1), ('FR', 1, -1), ('RL', -1, 1), ('RR', -1, -1)):
            # motorreductor GM25-370: cuerpo Ø25 mm a lo largo de y, eje de salida hacia la rueda
            c.append(('motor_' + nom, cilindro(tbm, [sx * lx, sy * (ly - 0.080), r], [sx * lx, sy * (ly - 0.020), r], 0.0125)))
            c.append(('eje_' + nom, cilindro(tbm, [sx * lx, sy * (ly - 0.020), r], [sx * lx, sy * (ly - ancho / 2), r], 0.002)))
        c.append(('placa_Mega2560', caja(tbm, [-0.030, 0, H_BANDEJA + 0.008], [0.1016, 0.0533, 0.016])))
        c.append(('portapilas_2x18650', caja(tbm, [0.030, 0, Z_PLACA_INF + E_PLACA / 2 + 0.011], [0.077, 0.041, 0.022])))
        occ_ch = componente(root, 'Chasis', c, {'placa_M': plastico, 'portapilas': plastico, 'motor': acero,
                                                 'eje': acero, 'placa': metal, 'columna': metal})
        occ_ch.isGrounded = True
        log.append('Chasis: %d cuerpos' % occ_ch.component.bRepBodies.count)

        # ---- ruedas con articulación de revolución
        ruedas = []
        for nom, sx, sy in (('FL', 1, 1), ('FR', 1, -1), ('RL', -1, 1), ('RR', -1, -1)):
            yc = sy * ly
            cuerpo = cilindro(tbm, [sx * lx, yc - ancho / 2, r], [sx * lx, yc + ancho / 2, r], r)
            occ_r = componente(root, 'Rueda_' + nom, [('rueda_mecanum_' + nom, cuerpo)], {'rueda': goma})
            ruedas.append(occ_r)
            cara = cara_cilindrica_mayor(occ_r)
            geo = adsk.fusion.JointGeometry.createByNonPlanarFace(cara, adsk.fusion.JointKeyPointTypes.MiddleKeyPoint)
            ji = root.asBuiltJoints.createInput(occ_r, occ_ch, geo)
            ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
            j = root.asBuiltJoints.add(ji)
            j.name = 'Giro_' + nom
        log.append('Ruedas en (±%.0f, ±%.0f) mm con articulación de giro' % (1000 * lx, 1000 * ly))

        # ---- exportación
        dir_cad = os.path.join(proyecto, 'agv', 'cad')
        dir_stl = os.path.join(dir_cad, 'stl')
        os.makedirs(dir_stl, exist_ok=True)
        em = design.exportManager
        try:
            em.execute(em.createSTEPExportOptions(os.path.join(dir_cad, 'agv_osoyoo.step'), root))
            log.append('STEP: agv/cad/agv_osoyoo.step')
        except Exception as e:
            log.append('No se pudo exportar el STEP: %s' % e)
        filas = []
        for occ in [occ_ch] + ruedas:
            for i in range(occ.bRepBodies.count):
                body = occ.bRepBodies.item(i)
                archivo = os.path.join(dir_stl, '%s__%s_%d.stl' % (occ.component.name, body.name, i))
                try:
                    op = em.createSTLExportOptions(body, archivo)
                    op.isBinaryFormat = True
                    op.sendToPrintUtility = False
                    em.execute(op)
                except Exception as e:
                    log.append('No se pudo exportar %s: %s' % (archivo, e))
                pp = body.physicalProperties
                cm = pp.centerOfMass
                filas.append([occ.component.name, body.name, body.material.name if body.material else '',
                              '%.5f' % pp.mass, '%.3f' % cm.x, '%.3f' % cm.y, '%.3f' % cm.z])
        with open(os.path.join(dir_cad, 'propiedades_fusion_agv.csv'), 'w', newline='', encoding='utf-8') as fh:
            w = csv.writer(fh)
            w.writerow(['# Masas con el material asignado por Fusion (SÓLIDO): solo orientativas; la masa del kit '
                        'no se pesó (1,5 kg estimado). Centro de masa en cm, marco del AGV.'])
            w.writerow(['componente', 'cuerpo', 'material', 'masa_kg', 'cm_x', 'cm_y', 'cm_z'])
            w.writerows(filas)
        log.append('STL y propiedades en agv/cad/')
        with open(os.path.join(aqui, 'ultimo_log.txt'), 'w', encoding='utf-8') as fh:
            fh.write('\n'.join(log))
        ui.messageBox('AGV OSOYOO generado.\n\n' + '\n'.join(log))
    except Exception:
        if ui:
            ui.messageBox('Error:\n{}'.format(traceback.format_exc()))
