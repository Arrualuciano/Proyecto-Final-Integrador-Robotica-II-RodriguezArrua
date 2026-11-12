# -*- coding: utf-8 -*-
"""
fusion_brazo.py — Genera el brazo antropomorfico 6R en Autodesk Fusion a partir de
parametros/parametros.json (tabla unica del proyecto).

Que hace:
  1. Lee la tabla DH y la geometria simplificada ('brazo.cad') del JSON.
  2. Calcula los puntos clave de la pose cero (O, S, E, W, F, T) con la cinematica directa
     y verifica que coincidan con la convencion del CAD (brazo vertical, antebrazo hacia +x).
  3. Crea un documento nuevo con un componente por eslabon (L0..L6) y la pieza.
     Cada eslabon tiene un cuerpo 'estructura' (union de primitivas) y cuerpos 'motor_*'.
     Si el eslabon tiene 'division' en el JSON (L2, que no entra en la cama de impresion), se le
     hacen los agujeros de tornillos y pasadores y se divide en 'estructura_a' y 'estructura_b'.
  4. Crea las 6 articulaciones de revolucion (as-built joints) J1..J6 sobre los ejes DH.
  5. Exporta un STL por eslabon (estructura y motores por separado) a brazo/stl/
     y las propiedades fisicas que calcula Fusion a brazo/cad/propiedades_fusion.csv.

Como ejecutarlo: Fusion > pestana UTILIDADES > COMPLEMENTOS > Scripts y complementos (Shift+S)
  > '+' junto a 'Mis scripts' > elegir la carpeta 'fusion_brazo' > seleccionar y 'Ejecutar'.

Unidades: el JSON esta en metros; la API de Fusion trabaja internamente en centimetros.
"""
import adsk.core
import adsk.fusion
import traceback
import json
import math
import os
import re
import csv

M2CM = 100.0  # m -> cm (unidad interna de la API)


# ----------------------------------------------------------------------------- utilidades
def P3(p):
    return adsk.core.Point3D.create(p[0] * M2CM, p[1] * M2CM, p[2] * M2CM)


def V3(v):
    return adsk.core.Vector3D.create(v[0], v[1], v[2])


def suma(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def dh_A(theta, d, a, alpha):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return [[ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0]]


def puntos_clave(P):
    """Puntos clave de la pose cero calculados con la cinematica directa (q = 0)."""
    dh = P['brazo']['dh']
    T = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    origenes, ejes_z = [[0.0, 0.0, 0.0]], [[0.0, 0.0, 1.0]]
    for fila in dh:
        T = matmul(T, dh_A(math.radians(fila['theta_offset_deg']), fila['d'], fila['a'],
                           math.radians(fila['alpha_deg'])))
        origenes.append([T[0][3], T[1][3], T[2][3]])
        ejes_z.append([T[0][2], T[1][2], T[2][2]])
    ht = P['brazo']['herramienta']['tcp_z6']
    z6 = ejes_z[6]
    tcp = suma(origenes[6], [ht * z6[0], ht * z6[1], ht * z6[2]])
    K = {'O': origenes[0], 'S': origenes[1], 'E': origenes[2], 'W': origenes[4],
         'F': origenes[6], 'T': tcp}
    # Convencion que asume la geometria del JSON
    d1, a2, d4, d6 = dh[0]['d'], dh[1]['a'], dh[3]['d'], dh[5]['d']
    esperado = {'S': [0, 0, d1], 'E': [0, 0, d1 + a2], 'W': [d4, 0, d1 + a2],
                'F': [d4 + d6, 0, d1 + a2], 'T': [d4 + d6 + ht, 0, d1 + a2]}
    err = max(math.dist(K[k], esperado[k]) for k in esperado)
    return K, err, ejes_z


def buscar_material(app, claves):
    """Primer material cuyo nombre contenga la clave como PALABRA (en orden de prioridad).
    Se compara por palabras para que 'PLA' no coincida con 'Plastic'."""
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


# ----------------------------------------------------------------------------- geometria
def crear_primitiva(tbm, prim, K):
    ref = K[prim['ref']]
    if prim['tipo'] == 'cil':
        p1, p2 = suma(ref, prim['p1']), suma(ref, prim['p2'])
        r = prim['r'] * M2CM
        return tbm.createCylinderOrCone(P3(p1), r, P3(p2), r)
    # 'caja' y 'motor': prisma alineado con los ejes de la base
    c, t = suma(ref, prim['centro']), prim['tamano']
    obb = adsk.core.OrientedBoundingBox3D.create(P3(c), V3([1, 0, 0]), V3([0, 1, 0]),
                                                 t[0] * M2CM, t[1] * M2CM, t[2] * M2CM)
    return tbm.createBox(obb)


def caja_entre(tbm, xmin, xmax, ymin, ymax, zmin, zmax):
    """Prisma alineado con los ejes entre los limites dados [m]."""
    c = [(xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2]
    obb = adsk.core.OrientedBoundingBox3D.create(P3(c), V3([1, 0, 0]), V3([0, 1, 0]),
                                                 (xmax - xmin) * M2CM, (ymax - ymin) * M2CM, (zmax - zmin) * M2CM)
    return tbm.createBox(obb)


def dividir(tbm, estructura, div, K, log):
    """Agujeros pasantes (a lo largo de y0) y division en dos mitades con solape a media altura:
    mitad 'a' = z < s_inicio, mas el solape con y < 0; mitad 'b' = el resto (ver parametros.json)."""
    ref = K[div['ref']]
    B = adsk.fusion.BooleanTypes
    for h in div['agujeros']:
        c = suma(ref, h['centro'])
        r = h['d'] / 2 * M2CM
        cil = tbm.createCylinderOrCone(P3(suma(c, [0, -0.2, 0])), r, P3(suma(c, [0, 0.2, 0])), r)
        tbm.booleanOperation(estructura, cil, B.DifferenceBooleanType)
    G = 1.0
    z0, z1 = ref[2] + div['s_inicio'], ref[2] + div['s_fin']
    region_a = caja_entre(tbm, -G, G, -G, G, -G, z0)
    tbm.booleanOperation(region_a, caja_entre(tbm, -G, G, -G, ref[1], z0, z1), B.UnionBooleanType)
    mitad_a = tbm.copy(estructura)
    tbm.booleanOperation(mitad_a, tbm.copy(region_a), B.IntersectionBooleanType)
    mitad_b = tbm.copy(estructura)
    tbm.booleanOperation(mitad_b, tbm.copy(region_a), B.DifferenceBooleanType)
    log.append('  division en dos mitades: solape %.0f..%.0f mm desde %s, %d agujeros'
               % (1000 * div['s_inicio'], 1000 * div['s_fin'], div['ref'], len(div['agujeros'])))
    return [('estructura_a', mitad_a), ('estructura_b', mitad_b)]


def es_estructura(body):
    return body.name.startswith('estructura')


def crear_eslabon(root, tbm, esl, K, mat_estr, mat_motor, log):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = esl['nombre']

    estructura = None
    motores = []
    for prim in esl['primitivas']:
        b = crear_primitiva(tbm, prim, K)
        if prim['tipo'] == 'motor':
            motores.append((prim['id'], b))
        elif estructura is None:
            estructura = b
        else:
            tbm.booleanOperation(estructura, b, adsk.fusion.BooleanTypes.UnionBooleanType)

    cuerpos = []
    if estructura is not None:
        if esl.get('division'):
            cuerpos = dividir(tbm, estructura, esl['division'], K, log)
        else:
            cuerpos = [('estructura', estructura)]
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    for nombre, b in cuerpos:
        comp.bRepBodies.add(b, bf).name = nombre
    for mid, b in motores:
        comp.bRepBodies.add(b, bf).name = 'motor_' + mid
    bf.finishEdit()

    for i in range(comp.bRepBodies.count):
        body = comp.bRepBodies.item(i)
        mat = mat_estr if es_estructura(body) else mat_motor
        if mat is not None:
            try:
                body.material = mat
            except Exception:
                pass
    log.append('Eslabon %s: %d cuerpo(s)' % (esl['nombre'], comp.bRepBodies.count))
    return occ


def cara_del_eje(occ, punto, direccion, radio):
    """Cara cilindrica de un cuerpo 'estructura*' cuyo eje coincide con el eje articular."""
    caras = []
    for i in range(occ.bRepBodies.count):
        if es_estructura(occ.bRepBodies.item(i)):
            caras.extend(list(occ.bRepBodies.item(i).faces))
    u = V3(direccion)
    u.normalize()
    p = P3(punto)
    candidata = None
    for f in caras:
        g = f.geometry
        if g.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
            continue
        cyl = adsk.core.Cylinder.cast(g)
        if abs(cyl.radius - radio * M2CM) > 1e-4:
            continue
        eje = cyl.axis.copy()
        eje.normalize()
        if abs(abs(eje.dotProduct(u)) - 1.0) > 1e-6:
            continue
        w = p.vectorTo(cyl.origin)
        if w.crossProduct(u).length > 1e-4:
            continue
        if candidata is None or f.area > candidata.area:
            candidata = f
    return candidata


def crear_articulacion(root, occ_hijo, occ_padre, esl, K, log):
    ej = esl['eje_cad']
    cara = cara_del_eje(occ_hijo, K[ej['ref']], ej['dir'], ej['r_cara'])
    if cara is None:
        log.append('ATENCION: no se encontro la cara del eje para J%d' % esl['articulacion'])
        return None
    geo = adsk.fusion.JointGeometry.createByNonPlanarFace(
        cara, adsk.fusion.JointKeyPointTypes.MiddleKeyPoint)
    ji = root.asBuiltJoints.createInput(occ_hijo, occ_padre, geo)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = root.asBuiltJoints.add(ji)
    j.name = 'J%d' % esl['articulacion']
    log.append('Articulacion %s creada (eje %s por %s)' % (j.name, ej['dir'], ej['ref']))
    return j


# ----------------------------------------------------------------------------- exportacion
def exportar(design, occs, dir_stl, ruta_csv, log):
    os.makedirs(dir_stl, exist_ok=True)
    em = design.exportManager
    filas = []
    for occ in occs:
        nombre = occ.component.name
        # borrar archivos viejos del eslabon (p. ej. el L2 entero de una version anterior):
        # Octave suma todos los '<eslabon>__estructura*.stl'
        for viejo in os.listdir(dir_stl):
            if viejo.startswith(nombre + '__') or re.fullmatch(re.escape(nombre) + r'(_[a-z])?\.3mf', viejo):
                os.remove(os.path.join(dir_stl, viejo))
        for i in range(occ.bRepBodies.count):
            body = occ.bRepBodies.item(i)
            archivo = os.path.join(dir_stl, '%s__%s.stl' % (nombre, body.name))
            try:
                op = em.createSTLExportOptions(body, archivo)
                op.isBinaryFormat = True
                op.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
                op.sendToPrintUtility = False
                em.execute(op)
            except Exception as e:
                log.append('No se pudo exportar %s: %s' % (archivo, e))
            pp = body.physicalProperties
            com = pp.centerOfMass
            _, ixx, iyy, izz, ixy, iyz, ixz = pp.getXYZMomentsOfInertia()
            _, i1, i2, i3 = pp.getPrincipalMomentsOfInertia()
            filas.append([nombre, body.name, body.material.name if body.material else '',
                          '%.6f' % pp.mass, '%.4f' % pp.volume, '%.6f' % (pp.density * 1e6),
                          '%.4f' % com.x, '%.4f' % com.y, '%.4f' % com.z,
                          '%.6f' % ixx, '%.6f' % iyy, '%.6f' % izz,
                          '%.6f' % ixy, '%.6f' % iyz, '%.6f' % ixz,
                          '%.6f' % i1, '%.6f' % i2, '%.6f' % i3])
        # Un 3MF por pieza impresa (cuerpos 'estructura*'): L2 sale en dos archivos, L2_brazo_a y _b
        for i in range(occ.bRepBodies.count):
            body = occ.bRepBodies.item(i)
            if not es_estructura(body):
                continue
            sufijo = body.name[len('estructura'):]
            try:
                op = em.createC3MFExportOptions(body, os.path.join(dir_stl, nombre + sufijo + '.3mf'))
                em.execute(op)
            except Exception:
                pass
    with open(ruta_csv, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['# Propiedades calculadas por Fusion con el material asignado (densidad SOLIDA). '
                    'Unidades: masa kg, volumen cm3, densidad kg/m3, centro de masa cm (pose cero, marco 0), '
                    'inercias kg*cm2 (Ixx..Ixz segun getXYZMomentsOfInertia; I1..I3 principales en el CM). '
                    'Octave reescala con la densidad efectiva.'])
        w.writerow(['eslabon', 'cuerpo', 'material', 'masa', 'volumen', 'densidad',
                    'cm_x', 'cm_y', 'cm_z', 'Ixx', 'Iyy', 'Izz', 'Ixy', 'Iyz', 'Ixz', 'I1', 'I2', 'I3'])
        w.writerows(filas)
    log.append('Exportados %d cuerpos a %s' % (len(filas), dir_stl))


# ----------------------------------------------------------------------------- principal
def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        aqui = os.path.dirname(os.path.realpath(__file__))
        proyecto = os.path.abspath(os.path.join(aqui, '..', '..', '..'))
        ruta_json = os.path.join(proyecto, 'parametros', 'parametros.json')
        with open(ruta_json, encoding='utf-8') as fh:
            P = json.load(fh)

        K, err, _ = puntos_clave(P)
        if err > 1e-9:
            ui.messageBox('La tabla DH no produce la pose cero que asume la geometria del CAD '
                          '(error %.3g m). Revisar parametros.json.' % err)
            return

        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        root = design.rootComponent
        # Nota: el nombre del componente raiz no se puede cambiar (es el del documento), y la API
        # informa ese error recien en la llamada siguiente. Por eso no se intenta renombrarlo.
        tbm = adsk.fusion.TemporaryBRepManager.get()

        mat_estr = buscar_material(app, ['PLA', 'ABS'])
        mat_motor = buscar_material(app, ['Steel', 'Acero'])
        log = ['Material estructura: %s | motores: %s' % (
            mat_estr.name if mat_estr else 'por defecto', mat_motor.name if mat_motor else 'por defecto')]

        occs = {}
        for esl in P['brazo']['cad']['eslabones']:
            occs[esl['nombre']] = crear_eslabon(root, tbm, esl, K, mat_estr, mat_motor, log)
        occs['L0_base'].isGrounded = True

        for esl in P['brazo']['cad']['eslabones']:
            if esl.get('padre'):
                crear_articulacion(root, occs[esl['nombre']], occs[esl['padre']], esl, K, log)

        # Pieza transportada, ubicada en el TCP y unida rigidamente a la pinza
        pz = P['pieza']
        occ_p = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ_p.component.name = 'Pieza'
        lado = pz['lado'] * M2CM
        cubo = tbm.createBox(adsk.core.OrientedBoundingBox3D.create(
            P3(K['T']), V3([1, 0, 0]), V3([0, 1, 0]), lado, lado, lado))
        bf = occ_p.component.features.baseFeatures.add()
        bf.startEdit()
        occ_p.component.bRepBodies.add(cubo, bf).name = 'estructura'
        bf.finishEdit()
        # Grupo rigido pinza + pieza: la pieza acompana al eslabon 6 cuando se mueve el brazo
        grupo = adsk.core.ObjectCollection.create()
        grupo.add(occs['L6_brida_pinza'])
        grupo.add(occ_p)
        root.rigidGroups.add(grupo, False).name = 'Agarre_pieza'

        lista = [occs[e['nombre']] for e in P['brazo']['cad']['eslabones']] + [occ_p]
        exportar(design, lista, os.path.join(proyecto, 'brazo', 'stl'),
                 os.path.join(proyecto, 'brazo', 'cad', 'propiedades_fusion.csv'), log)

        with open(os.path.join(aqui, 'ultimo_log.txt'), 'w', encoding='utf-8') as fh:
            fh.write('\n'.join(log))
        ui.messageBox('Brazo generado.\n\n' + '\n'.join(log) +
                      '\n\nTCP en pose cero: (%.0f, %.0f, %.0f) mm' % tuple(1000 * v for v in K['T']) +
                      '\nGuardar el documento con Archivo > Guardar.')
    except Exception:
        if ui:
            ui.messageBox('Error:\n{}'.format(traceback.format_exc()))
