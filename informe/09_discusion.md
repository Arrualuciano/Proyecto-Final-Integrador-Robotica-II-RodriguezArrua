### 9.1 Interpretación

- **El brazo está dominado por la gravedad, no por la inercia.**
  - En el eje 2, el par dinámico pico supera al estático en solo un 7 % (6.A.3). Por eso acortar el ciclo casi
    no aumenta la exigencia de los motores.
  - Por la misma razón, el error del control con modelo imperfecto es sobre todo un **sesgo gravitatorio**:
    la masa no modelada multiplicada por g y por el brazo de palanca, dividida por la rigidez del lazo.
  - Ese sesgo explica por qué la acción integral resuelve E2 y E3 sin aumentar el par pico.
- **La rigidez estructural es comparable con el error de control.**
  - L2 se deforma 0,30 mm en la colocación (4.B.6), del orden del error del control (0,15–0,46 mm).
  - Los encoders de los motores no ven esa deformación. El presupuesto de error de R‑B‑4 (1 mm) queda
    ajustado cuando se suman las demás contribuciones no modeladas.
- **En el AGV, lo que limita es el estimador.**
  - El lazo de pose sigue casi perfectamente a la odometría (en el piso, < 1° y < 5 mm según los encoders).
    El error final real es el deslizamiento de los rodillos, que los encoders no ven: +3 % en avance, −20 % en
    lateral y −0,5 % en giro (medidos), más el acople lateral → avance y el giro por arranques desparejos con
    carga (sección 8.B).
  - La calibración corrige solo la parte que se repite. **La mejora de fondo es medir la pose sin las ruedas**:
    una IMU (MPU‑6050) para el ángulo en el lazo de pose y una marca visual en B para la aproximación final.

### 9.2 Tipos de error

- **Numérico.** Se acotó repitiendo el cálculo con un paso menor:
  - en el control del brazo es < 2 % del error total;
  - en la dinámica directa, la energía se conserva dentro de 5·10⁻⁷ J.

  No condiciona ninguna conclusión.
- **De modelo.** Es el dominante en los ensayos con incertidumbre:
  - brazo: carga, masas y fricción;
  - AGV: deslizamiento, zona muerta en el piso y dispersión entre motores.
- **De control.** Es el error transitorio que dejan las ganancias con el modelo exacto. Domina en el PD+g
  nominal (7 mm).

### 9.3 Límites del modelo

- **Brazo.**
  - Actuadores ideales de par: requiere drivers de lazo cerrado.
  - Transmisiones rígidas, sin juego.
  - Mediciones exactas: sin cuantización ni ruido.
  - Fricción con valores estimados.
  - Geometría simplificada: sin alojamientos ni rodamientos.
  - Material con propiedades de bibliografía.
- **AGV.**
  - Geometría, K, zona muerta, dispersión entre motores y deslizamiento están **medidos** en el kit (5.C.4
    y 8.B). Masa, inercias y constante de tiempo siguen estimadas.
  - La fricción es solo de Coulomb, sin fricción estática mayor, así que el modelo no reproduce que las
    ruedas se peguen a baja velocidad.
  - El deslizamiento se modela como un factor constante por dirección. En el piso depende de la maniobra, la
    carga y la superficie, y además acopla el lateral con el avance.
  - K del accionamiento es proporcional a V_bat, que baja durante la sesión.

### 9.4 Lo previsto frente a lo medido en el piso

Antes de los ensayos se había previsto que el robot real:
- necesitaría más PWM para arrancar en el piso que con las ruedas elevadas. **Se confirmó:** 15–30 PWM más;
- tendría en lazo abierto una dispersión mayor que la simulada. **Fue peor:** con la fricción estática el
  lazo abierto ni siquiera completa las maniobras (T2 no se movió, T3 giró 26° de 90°);
- perdería más precisión en el lateral que en el avance. **Se confirmó:** 22 cm contra 4 cm.

Aparecieron además tres efectos que el modelo no tenía:
1. **Acople lateral → avance:** el robot avanza 6–18 cm al moverse 0,6 m de costado.
2. **Deslizamiento lateral variable:** 0,80 en T2 y 0,95–1,0 en el tramo lateral de A→B. Un factor fijo no lo
   calibra.
3. **Giro por arranques desparejos con carga:** 8–12°, invisible para los encoders.

**Lo que sí se verificó del modelo:**
- los signos, el patrón X de rodillos y l_x + l_y: el giro de 90° en modo 2 es exacto y η_ω = 0,995;
- el radio efectivo, que da +3 % en avance.

**Superficie.** Sobre otra superficie del laboratorio, el operador observó que los mismos movimientos
salían sin desvío visible. Es coherente con que el error venga del deslizamiento sobre el piso cerámico y
no del control, pero no se midió.

### 9.5 Mejoras propuestas

1. **Medir la pose del AGV sin pasar por las ruedas.** Es la mejora que más reduce el error real:
   - una IMU (por ejemplo MPU‑6050) en el lazo de pose corrige el giro por deslizamiento, que los encoders
     no ven (8–12° con carga);
   - una marca visual en B, o una cámara, corrige la posición final.

   Con eso, R‑AGV‑3 deja de depender del piso.
2. **Arranque sincronizado de las ruedas.** En el tramo lateral, frenar en proporción a las otras ruedas
   mientras una no despegó, como hace el firmware BioLog cuando una rueda satura. Reduciría el giro al
   arrancar con carga.
3. **Prealimentación compensada por tensión.** Escalar K con la V_bat medida (divisor en A5, ya previsto en
   el firmware) para que el lazo abierto no dependa de la carga de la batería.
4. **Calibración dependiente de la carga.** Usar η en función de la masa transportada (T5) o corregir la pose
   final con una referencia externa (marca visual en B) en la aproximación a la mesa.
5. **Enseñar el punto de colocación.** En el brazo construido, medir el error repetible de la colocación
   (deformación más geometría real) y corregirlo con un desplazamiento fijo.
6. **Diseño detallado del CAD.** Alojamientos de rodamientos, reductores, canales de cables y revisión de
   interferencias en poses extremas.
7. **Ensayos de material.** Probetas planas y paradas del filamento usado, para reemplazar los valores de
   bibliografía.
