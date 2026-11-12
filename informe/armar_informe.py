#!/usr/bin/env python3
"""armar_informe.py — Une las secciones del informe (Markdown) en un documento único y genera el PDF.

  * Portada con los datos que pide el enunciado §10 (completar ALUMNOS abajo).
  * Las 11 secciones del enunciado, en orden, con índice.
  * Numeración automática: "![texto](ruta)" → Figura k; una línea "Tabla: texto" antes de una tabla → Tabla k.
  * Encabezados de los borradores normalizados: "5.A" → nivel 2, "5.A.1" → nivel 3; se quita "(borrador...)".
  * Anexo B: registro de decisiones tomado de 00_Aplicacion_y_plan.md §7.
Salida: informe/informe_integrador.html y informe/informe_integrador.pdf (Microsoft Edge sin ventana).
Uso: py -3.13 informe/armar_informe.py
"""
import datetime as dt
import html
import os
import re
import subprocess

import markdown

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, ".."))

# ------------------------------------------------------------------ datos de la portada
TITULO = "Celda de abastecimiento de piezas con un AGV omnidireccional y un brazo antropomórfico 6R"
ALUMNOS = ["Nicolás Rodriguez", "Luciano Arrua"]
DOCENTE = "Ing. Nahuel Medina · Patricio Silva"
MATERIA = "Robótica II"
CARRERA = "Ingeniería en Mecatrónica"
FACULTAD = "Facultad de Ciencias de la Alimentación — Universidad Nacional de Entre Ríos"
FECHA = "Noviembre de 2026"

SECCIONES = [
    ("1. Resumen", ["01_resumen.md"]),
    ("2. Aplicación y requisitos", ["02_aplicacion.md"]),
    ("3. Antecedentes", ["03_antecedentes.md"]),
    ("4. Diseño", ["04_diseno_brazo.md", "04a_verificacion_estructural_L2.md", "04c_fabricacion.md", "04d_agv_kit.md"]),
    ("5. Modelos", ["05a_cinematica_brazo.md", "05b_dinamica_brazo.md", "05c_agv_modelo.md"]),
    ("6. Trayectorias", ["06a_trayectorias_brazo.md", "06b_agv_trayectorias.md"]),
    ("7. Control e interfaz", ["07a_control_brazo.md", "07b_agv_control.md", "07c_arquitectura_interfaz.md"]),
    ("8. Validación", ["08_validacion.md"]),
    ("9. Discusión", ["09_discusion.md"]),
    ("10. Conclusiones", ["10_conclusiones.md"]),
    ("11. Bibliografía", ["11_bibliografia.md"]),
    ("Anexo A. Instrucciones de ejecución", ["12_anexo_ejecucion.md"]),
]

CSS = """
@page { size: A4; margin: 20mm 18mm 20mm 18mm; }
body { font-family: 'Segoe UI', Calibri, Arial, sans-serif; font-size: 10.3pt; line-height: 1.42; color: #1d1d1f; }
h1 { font-size: 17pt; color: #1f3d6b; border-bottom: 2px solid #1f3d6b; padding-bottom: 3px; margin-top: 0;
     page-break-before: always; }
h2 { font-size: 13.5pt; color: #1f3d6b; margin-top: 18px; }
h3 { font-size: 11.5pt; color: #2b4f86; margin-top: 14px; }
h4 { font-size: 10.5pt; color: #2b4f86; }
h2, h3, h4 { page-break-after: avoid; break-after: avoid; }   /* sin títulos huérfanos al pie de página */
table { border-collapse: collapse; margin: 6px 0 12px 0; font-size: 8.8pt; width: 100%; page-break-inside: auto; }
tr { page-break-inside: avoid; }
th, td { border: 1px solid #b9c3d3; padding: 3px 5px; vertical-align: top; }
th { background: #e8eef7; }
caption { caption-side: top; text-align: left; font-weight: 600; font-size: 9.2pt; padding-bottom: 3px; color: #333; }
figure { margin: 10px 0 14px 0; text-align: center; page-break-inside: avoid; }
figure img { max-width: 100%; max-height: 120mm; }
figcaption { font-size: 9pt; color: #333; margin-top: 4px; }
code { font-family: Consolas, monospace; font-size: 8.8pt; background: #f2f4f7; padding: 0 2px; }
pre { background: #f2f4f7; padding: 6px 8px; font-size: 8.6pt; white-space: pre-wrap; page-break-inside: avoid; }
blockquote { border-left: 3px solid #b9c3d3; margin: 6px 0; padding: 2px 10px; color: #444; background: #f8f9fb; }
.portada { text-align: center; padding-top: 30mm; }
.portada .inst { font-size: 12pt; }
.portada .titulo { font-size: 21pt; font-weight: 700; color: #1f3d6b; margin: 28mm 0 8mm 0; }
.portada .sub { font-size: 12.5pt; color: #444; }
.portada .datos { margin-top: 30mm; font-size: 11.5pt; line-height: 1.8; }
.indice { page-break-before: always; }
.indice ul { list-style: none; padding-left: 0; }
.indice li { margin: 2px 0; }
.indice li.n2 { padding-left: 16px; font-size: 9.6pt; }
"""


def normalizar(md):
    """Ajusta encabezados de los borradores y convierte las marcas de figura y tabla."""
    out = []
    for linea in md.splitlines():
        m = re.match(r"^(#+)\s+(\d+\.[A-Z](?:\.\d+)*)\b(.*)$", linea)
        if m:
            nivel = 2 + m.group(2).count(".") - 1
            texto = re.sub(r"\s*\(borrador[^)]*\)", "", m.group(3))
            linea = "#" * nivel + " " + m.group(2) + texto
        elif re.match(r"^#{1,3}\s+(Reproducción|Referencias)\s*$", linea):
            linea = "#### " + linea.lstrip("#").strip()
        out.append(linea)
    return "\n".join(out)


def separar_listas(md):
    """Python-Markdown no reconoce una lista pegada a un párrafo: inserta una línea en blanco antes."""
    out, previa, en_codigo = [], "", False
    for linea in md.splitlines():
        if linea.lstrip().startswith("```"):
            en_codigo = not en_codigo
        es_item = re.match(r"^\s*([-*]|\d+\.)\s", linea) is not None
        previa_item = re.match(r"^\s*([-*]|\d+\.)\s", previa) is not None
        if (not en_codigo and es_item and previa.strip() and not previa_item and not previa.startswith(" ")
                and not previa.startswith("|") and not previa.startswith(">")):
            out.append("")
        elif not en_codigo and linea.startswith("![") and previa.strip():
            out.append("")                       # figura: siempre como párrafo propio, fuera de listas
        out.append(linea)
        previa = linea
    return "\n".join(out)


def html_seccion(md, contadores):
    # marcadores de tabla: "Tabla: texto" en la línea anterior a la tabla
    def tabla(m):
        contadores["tabla"] += 1
        return f'\n<p class="tcap">Tabla {contadores["tabla"]}. {m.group(1).strip()}</p>\n'
    md = re.sub(r"^Tabla:\s*(.+)$", tabla, md, flags=re.M)
    md = separar_listas(md)
    cuerpo = markdown.markdown(md, extensions=["tables", "fenced_code", "mdx_truly_sane_lists"],
                               extension_configs={"mdx_truly_sane_lists": {"nested_indent": 2, "truly_sane": True}})

    def figura(m):
        contadores["figura"] += 1
        alt, src = html.unescape(m.group(1)), m.group(2)
        return (f'<figure><img src="{src}" alt="{html.escape(alt)}">'
                f'<figcaption><b>Figura {contadores["figura"]}.</b> {html.escape(alt)}</figcaption></figure>')
    cuerpo = re.sub(r'<p><img alt="([^"]*)" src="([^"]+)" ?/?></p>', figura, cuerpo)
    # el rótulo de tabla pasa a ser <caption> de la tabla siguiente
    cuerpo = re.sub(r'<p class="tcap">(.*?)</p>\s*<table>', r'<table><caption>\1</caption>', cuerpo, flags=re.S)
    return cuerpo


def registro_decisiones():
    t = open(os.path.join(RAIZ, "00_Aplicacion_y_plan.md"), encoding="utf-8").read()
    bloque = t.split("## 7. Registro de decisiones", 1)[1].split("\n---", 1)[0]
    return ("Registro breve de decisiones y cambios (enunciado §8), en orden de registro. Las versiones de los "
            "archivos se identifican por fecha en cada carpeta de resultados.\n\nTabla: Registro de decisiones\n"
            + bloque)


def main():
    cont = {"figura": 0, "tabla": 0}
    partes, indice = [], []
    for titulo, archivos in SECCIONES + [("Anexo B. Registro de decisiones", None)]:
        ancla = re.sub(r"[^a-z0-9]+", "-", titulo.lower()).strip("-")
        indice.append(f'<li><a href="#{ancla}">{html.escape(titulo)}</a></li>')
        if archivos is None:
            md = registro_decisiones()
        else:
            md = "\n\n".join(normalizar(open(os.path.join(AQUI, a), encoding="utf-8").read()) for a in archivos)
        for m in re.finditer(r"^##\s+(\d+\.[A-Z])\s+(.*)$", md, flags=re.M):
            indice.append(f'<li class="n2">{m.group(1)} {html.escape(m.group(2))}</li>')
        partes.append(f'<h1 id="{ancla}">{html.escape(titulo)}</h1>\n' + html_seccion(md, cont))

    portada = f"""<div class="portada">
<div class="inst">{html.escape(FACULTAD)}<br>{html.escape(CARRERA)} — {html.escape(MATERIA)}</div>
<div class="titulo">{html.escape(TITULO)}</div>
<div class="sub">Trabajo integrador: brazo antropomórfico imprimible (diseño, cálculo y simulación)<br>
y AGV OSOYOO con ruedas mecanum (modelo, control y ensayo)</div>
<div class="datos">Alumnos: {' · '.join(html.escape(a) for a in ALUMNOS)}<br>
Docentes: {html.escape(DOCENTE)}<br>{html.escape(FECHA)}</div></div>"""
    doc = f"""<!doctype html><html lang="es"><head><meta charset="utf-8">
<title>{html.escape(TITULO)}</title><style>{CSS}</style></head><body>
{portada}
<div class="indice"><h2>Índice</h2><ul>{''.join(indice)}</ul>
<p style="font-size:9pt;color:#555">Versión generada el {dt.datetime.now():%d/%m/%Y %H:%M} con informe/armar_informe.py.
{cont['figura']} figuras, {cont['tabla']} tablas.</p></div>
{''.join(partes)}
</body></html>"""
    ruta_html = os.path.join(AQUI, "informe_integrador.html")
    open(ruta_html, "w", encoding="utf-8").write(doc)
    print(f"HTML: {ruta_html} ({cont['figura']} figuras, {cont['tabla']} tablas)")

    edge = next((p for p in [r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
                             r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"] if os.path.exists(p)), None)
    if edge is None:
        print("No se encontró Microsoft Edge: abrir el HTML e imprimir a PDF a mano.")
        return
    ruta_pdf = os.path.join(AQUI, "informe_integrador.pdf")
    subprocess.run([edge, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
                    f"--print-to-pdf={ruta_pdf}", "file:///" + ruta_html.replace("\\", "/")],
                   check=False, timeout=180, capture_output=True)
    print("PDF:", ruta_pdf if os.path.exists(ruta_pdf) else "NO generado")


if __name__ == "__main__":
    main()
