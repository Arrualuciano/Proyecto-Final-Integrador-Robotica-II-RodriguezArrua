## 4.C Fabricación, componentes y montaje del brazo

### 4.C.1 Propuesta de impresión

- La orientación y los parámetros de cada pieza están en la tabla de la sección 4.B.8. Todos se rigen por un
  mismo criterio: la tensión principal en el plano de la capa y los agujeros de ejes verticales.
- Parámetros comunes, que también están en `parametros.json`, en `brazo.material.impresion`: PLA, boquilla
  0,4 mm, capas de 0,2 mm, 4 perímetros, 6 capas sólidas y relleno giroide del 30 %.
- **Material total.** Con la densidad efectiva de 800 kg/m³, la estructura impresa pesa unos 3,2 kg. La
  base L0, de 1,13 kg, se puede vaciar sin problema porque solo trabaja a compresión y torsión. Con soportes
  y desperdicio, hacen falta unos **3,5 kg de filamento** (estimado).
- **Probeta de calibración.** Antes de las piezas finales se imprime una probeta con:
  - un agujero Ø4,4 (tornillo), uno Ø3,8 (pasador, para escariar) y uno de Ø22 (asiento de rodamiento 608);
  - un tramo de pared de 4 perímetros;
  - probetas de tracción planas y paradas.

  Sirve para ajustar la compensación de agujeros del laminador y verificar la densidad efectiva (pesándola)
  y las resistencias del material (sección 4.B.7).

### 4.C.2 Tolerancias de encastres y uniones

Una impresora FDM típica da ±0,1–0,2 mm, y los agujeros salen ≈ 0,1–0,2 mm más chicos que en el modelo
porque el filamento se aplasta hacia adentro del contorno.

| Unión | Medida en el CAD | Ajuste buscado | Justificación |
|---|---|---|---|
| Agujero de paso para M4 | Ø4,4 | Juego de 0,2–0,4 mm | Montaje sin forzar; el corte no pasa por los tornillos (los pasadores ubican) |
| Pasador Ø4 h8 | Agujero impreso Ø3,8, escariado a Ø4 H7 | Ajuste sin juego | Elimina el deslizamiento de la unión de L2: el juego daría 2,2 mm en el TCP (4.B.5) |
| Asiento de rodamiento 608 (Ø22) | Ø22,0 + compensación de la probeta | Leve interferencia (0–0,05 mm) | Que el rodamiento no gire en el asiento; se monta a presión con prensa |
| Eje en el rodamiento (Ø8) | Eje de acero rectificado | h6 en el rodamiento | Lo fija el componente comercial |
| Agujeros horizontales | Forma de lágrima con punta a 45° | — | Imprimir sin soportes y conservar el diámetro útil |
| Solape de L2 | Caras planas de contacto | Planitud de la capa superior | La cara de contacto de L2b se imprime arriba para que salga lisa |

### 4.C.3 Lista de componentes

Estimación de orden de magnitud con precios de referencia de tiendas en línea, en USD. **No es una
cotización**: hay que actualizarla con proveedores locales.

Tabla: Lista de componentes del brazo

| Componente | Cant. | Tipo | Precio unit. est. [USD] | Subtotal [USD] |
|---|---|---|---|---|
| Filamento PLA, 1 kg | 4 | impreso | 20 | 80 |
| Motor NEMA 17 de 48 mm (ejes 1–3) | 3 | comercial | 12 | 36 |
| Motor NEMA 14 de 28 mm (ejes 4–5) | 2 | comercial | 12 | 24 |
| Micro motor paso a paso (eje 6) | 1 | comercial | 10 | 10 |
| Driver de lazo cerrado con encoder (por motor) | 6 | comercial | 20 | 120 |
| Micro servo de 15 g (pinza) | 1 | comercial | 3 | 3 |
| Rodamientos 608 y de brida de diámetro grande para los reductores | ~12 | comercial | 1,5 | 18 |
| Correas y poleas GT2 (ejes 3 y 5) | 2 juegos | comercial | 6 | 12 |
| Tornillería M3/M4, tuercas, arandelas DIN 9021, 2 pasadores Ø4 × 40 | 1 juego | comercial | 15 | 15 |
| Controlador (Arduino Mega o ESP32) | 1 | comercial | 15 | 15 |
| Fuente de 24 V y 5 A, cables, borneras y parada de emergencia | 1 juego | comercial | 35 | 35 |
| **Total estimado** | | | | **≈ 370** |

- Las piezas impresas son L0, L1, L2a, L2b, L3, L4, L5 y L6, más los dedos y los engranajes de los
  reductores.
- El costo está dominado por los drivers de lazo cerrado, que exige la hipótesis de control por par. Con
  drivers de paso comunes (≈ USD 3 c/u), el total baja a unos USD 270, pero el control queda limitado a
  posición.

### 4.C.4 Secuencia de montaje

1. **Reductores.** Montar los reductores impresos con sus motores y rodamientos y probarlos en banco:
   juego, rozamiento y sentido.
2. **Base (L0).** Fijar la base al pedestal. Colocar M1 y el reductor del eje 1 con el rodamiento de la
   torreta.
3. **Hombro (L1).** Montar L1 sobre la salida del eje 1. Colocar M2 y su reductor en la oreja derecha del
   hombro.
4. **Brazo, mitad del hombro (L2a).** Insertar el cubo de L2a en la horquilla del hombro, con el eje
   pasante y los rodamientos. Acoplarlo a la salida del reductor 2.
5. **Brazo, mitad del codo (L2b).** Presentarla sobre el solape, colocar los **2 pasadores** golpeando
   suave y después los 2 tornillos M4 con arandelas anchas. Apretar a ≈ 0,8 N·m: más apriete aplasta el PLA.
6. **Motor del codo (M3).** Montarlo sobre L2a, con la correa GT2 hasta la polea del codo. Tensar la correa.
7. **Codo (L3).** Montar L3 en la horquilla del codo con su eje y rodamientos. Colocar M4 detrás del codo.
8. **Antebrazo (L4).** Montarlo sobre la salida del eje 4. Colocar M5 y su correa.
9. **Muñeca (L5 y M6).** Montar la muñeca en la horquilla del antebrazo y colocar M6.
10. **Brida y pinza (L6).** Montar la brida, los dedos y el servo de la pinza.
11. **Cableado.** Pasar los cables por el exterior de los eslabones, con espiral y sujeción en cada
    articulación. Dejar un rulo de servicio para el giro de cada eje: ±170° en el eje 1 y ±180° en los ejes
    4 y 6.
12. **Puesta a cero.** Llevar el brazo a la pose cero (vertical, antebrazo horizontal) con una escuadra,
    fijar el cero de cada encoder y comprobar los sentidos contra la tabla DH.

**Accesibilidad e interferencias.**
- Los tornillos de la unión de L2 se ajustan desde las dos caras laterales, que quedan libres en todo el
  rango del eje 2.
- Las holguras con el plano de apoyo a lo largo del ciclo están en la sección 6.A.2.
- **Pendiente:** la revisión de interferencias en poses extremas con *Inspeccionar → Interferencia* de
  Fusion, y el detalle de alojamientos de rodamientos y canales de cables. La geometría actual es
  simplificada.
