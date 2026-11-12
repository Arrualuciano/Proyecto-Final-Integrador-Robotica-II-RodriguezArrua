# CAD del brazo 6R en Fusion

El CAD **no se dibuja a mano**: lo genera `fusion_brazo/fusion_brazo.py` a partir de
`parametros/parametros.json`. Así, Fusion, Octave y CoppeliaSim usan las mismas medidas (enunciado §7.1).
Si cambia una medida, se edita el JSON y se vuelve a ejecutar el script.

## Ejecutar el script

0. (Una sola vez) Fusion → **Preferencias** → **General** → **Orientación de modelado predeterminada:
   Z hacia arriba**. El modelo usa z₀ vertical, como la cátedra y CoppeliaSim. Con la opción de fábrica
   («Y hacia arriba») el brazo se ve acostado, aunque las coordenadas y los archivos exportados son los mismos.
1. Abrir Fusion.
2. Pestaña **UTILIDADES** → **COMPLEMENTOS** → **Scripts y complementos** (atajo `Shift+S`).
3. En la pestaña **Scripts**, botón **+** (junto a «Mis scripts») → elegir la carpeta
   `brazo/cad/fusion_brazo` de este proyecto.
4. Seleccionar `fusion_brazo` → **Ejecutar**. Tarda unos segundos y abre un documento nuevo.
5. Guardar el documento (Archivo → Guardar). Versión vigente: documento **`robot`** en la nube de Fusion
   (07/11/2026, con L2 dividido).

## Qué genera

| Salida | Dónde | Para qué |
|---|---|---|
| Componentes `L0_base` … `L6_brida_pinza` y `Pieza` | Documento de Fusion | Un componente por eslabón; cuerpo `estructura` (piezas impresas) y cuerpos `motor_*` (envolventes de los accionamientos) |
| Articulaciones `J1` … `J6` (revolución) y grupo rígido `Agarre_pieza` (pinza + pieza) | Documento de Fusion | Mover el brazo a mano para revisar el alcance y las interferencias |
| `<eslabón>__estructura.stl`, `<eslabón>__motor_*.stl` | `brazo/stl/` | Malla para CoppeliaSim (en la pose cero, marco 0) y cálculo de masas e inercias en Octave. L2 sale en dos mitades: `L2_brazo__estructura_a.stl` y `_b.stl` (Octave suma todos los `__estructura*.stl`) |
| `<eslabón>.3mf` (L2: `L2_brazo_a.3mf`, `L2_brazo_b.3mf`) | `brazo/stl/` | Archivos de impresión, uno por pieza impresa |
| `propiedades_fusion.csv` | `brazo/cad/` | Masas, centros de masa e inercias que calcula Fusion. Sirven para contrastar con el cálculo de Octave |
| `ultimo_log.txt` | `brazo/cad/fusion_brazo/` | Registro de la última ejecución |

## Convenciones

- **STL en milímetros**, en la trama {0} y en la pose cero (verificado: la base va de z = 0 a 80 mm y la
  pieza está centrada en x = 330 mm).
- `propiedades_fusion.csv`: Fusion no trae PLA en la biblioteca y usó **ABS (1060 kg/m³, sólido)**. Las
  masas de la estructura se reescalan en Octave a la densidad efectiva del JSON. Las masas de los motores del
  CSV (acero macizo) **no se usan**: valen las del JSON. Los productos de inercia (Ixy, Iyz, Ixz) son
  elementos del tensor (con signo menos, −∫yz dm) respecto del origen de {0}. Comprobado con el motor M2:
  Iyz = −m·y·z = −209,4 kg·cm².
- La pose del CAD es la **pose cero de la tabla DH**: brazo vertical, antebrazo y herramienta horizontales
  hacia +x₀. TCP en (330, 0, 380) mm. Lo comprueba `brazo/octave/verificar_pose_cero.m`.
- El origen del documento coincide con la trama {0}: eje 1 vertical, apoyo de la base en z = 0.
- El sentido positivo de giro de las articulaciones de Fusion puede no coincidir con el de DH, porque Fusion
  toma el eje de la cara cilíndrica. Para el cálculo y la simulación vale el sentido DH; en Fusion las
  articulaciones se usan solo para revisar el movimiento.
- La geometría es **simplificada** (cilindros y prismas). Encima de esta base se agregan los detalles de
  fabricación: alojamientos de motores y rodamientos, tornillería, canales de cables y paredes huecas.
  Cada detalle que cambie la masa se refleja en el JSON o se vuelve a exportar.

## L2 en dos mitades

L2 (70 × 90 × 267 mm) no entra en una cama de 220 × 220 mm. Por eso el eslabón `L2_brazo` del JSON tiene
un bloque `division`, y con él el script:
- hace 2 agujeros Ø4,4 para tornillos M4 y 2 Ø3,8 para pasadores Ø4 (se escarian), pasantes en y₀;
- divide la estructura en `estructura_a` (lado del hombro) y `estructura_b` (lado del codo), con un
  solape a media altura entre s = 65 y 115 mm desde el hombro.

Al exportar, el script borra los archivos viejos del eslabón (el `L2_brazo__estructura.stl` entero), para
que Octave no sume dos veces la misma masa. Justificación, orientación de impresión y verificación:
`informe/04a_verificacion_estructural_L2.md`. Para comprobar las mitades exportadas:
`py -3.13 brazo/python/division_L2.py` (volumen, que entren en la cama, superficie con soporte).

Comprobado con la exportación de Fusion del 07/11:
- mitades de 268,5 y 340,7 cm³, estancas y sin superposición;
- L2a de 70 × 150 × 44 mm, sin soportes;
- L2b de 64 × 167 × 90 mm, con ≈ 58 cm² de soporte;
- en los 6 eslabones, Octave (malla) y Fusion difieren < 0,1 % en volumen y < 0,01 mm en centro de masa.

(Si no hubiera Fusion a mano, `division_L2.py` también aplica la división sobre un `L2_brazo__estructura.stl`
entero.)

## Revisiones que conviene hacer en Fusion

- **Inspeccionar → Interferencia** sobre todos los componentes, en la pose cero y en poses extremas: mover J2,
  J3 y J5 hasta los límites de `parametros.json`.
- **Inspeccionar → Medir** el TCP y comparar con la salida de `mcd_brazo.m` para una pose conocida.
