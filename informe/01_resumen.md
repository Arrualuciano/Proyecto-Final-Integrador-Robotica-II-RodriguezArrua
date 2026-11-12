**Problema.** Se diseñó una celda de abastecimiento de piezas con dos robots que comparten la misma pieza:
- un AGV omnidireccional OSOYOO con ruedas mecanum lleva un cubo de 40 mm y 0,10 kg desde una estación de
  carga A hasta la estación B, en un recorrido con avance y desplazamiento lateral;
- un brazo antropomórfico 6R, de diseño propio e imprimible en 3D, toma la pieza de la bandeja del AGV y la
  coloca girada 90° en el alojamiento de una bandeja de trabajo.

Se fijaron requisitos cuantitativos para los dos robots. Los más exigentes son:
- brazo: error de colocación ≤ 1 mm y ≤ 1°, ciclo ≤ 12 s, carga máxima 0,25 kg;
- AGV: error de llegada a B ≤ 20 mm y ≤ 3°, traslado ≤ 15 s.

**Desarrollo.** Una sola tabla de parámetros (`parametros.json`) alimenta todo lo que sigue.
- **CAD del brazo:** lo genera un script paramétrico de Fusion. Produce los STL/3MF y las propiedades de masa.
- **Cinemática del brazo:** tabla DH estándar; cinemática inversa analítica con 8 soluciones y selección por
  continuidad; Jacobiano geométrico con sus singularidades en forma cerrada.
- **Dinámica del brazo:** Newton–Euler recursivo, contrastado con Lagrange. Con ella se dimensionaron los
  accionamientos y la reducción del eje 2 pasó de 20 a 30.
- **Trayectorias del brazo:** quínticas articulares y tramos cartesianos con interpolación de orientación
  por eje–ángulo.
- **Control del brazo:** se compararon tres leyes sobre la planta simulada en Octave: PD con compensación
  de gravedad, par calculado y par calculado con acción integral.
- **Verificación estructural:** se verificó la pieza impresa crítica (el eslabón L2) teniendo en cuenta la
  dirección de las capas.
- **CoppeliaSim:** la escena reproduce el resultado del lazo de control con juntas cinemáticas.
- **AGV:** se desarrollaron el modelo cinemático y dinámico (con el motor CC), el método de identificación y
  un firmware para la Arduino Mega con dos lazos (PI de ruedas a 100 Hz y lazo de pose a 50 Hz), watchdog
  y registro. La simulación en Octave ejecuta el mismo algoritmo que el firmware.
- **Interfaz:** una sola interfaz en Qt opera el brazo virtual y el AGV, en modo virtual o físico.

**Resultados principales.**
- **Control del brazo.** El par calculado con integral cumple todos los requisitos en los tres ensayos, el
  nominal, el de carga máxima con perturbación y el de masas +15 %: colocación ≤ 0,19 mm y ≤ 0,04°,
  seguimiento ≤ 2,7 mm. El PD con gravedad y el par calculado solo no los cumplen cuando el modelo tiene
  error.
- **Ciclo del brazo:** dura 10,1 s.
- **Estructura.** L2 resiste con un factor de seguridad ≥ 5,4 incluso con el motor 2 saturado contra un
  obstáculo. No entra en una cama de 220 × 220 mm, así que se dividió en dos mitades con pasadores. La
  rigidez, y no la resistencia, es lo que limita.
- **AGV en el kit real** (ensayos del 11/11/2026, órdenes por WiFi, medición externa sobre las baldosas).
  - **Identificación:** 1482 cuentas por vuelta (reducción ~34:1, no 75:1 como dice la ficha), K ≈ 0,125
    rad/s/PWM y zonas muertas de 66–104 PWM en el piso. Con eso se ajustó el modelo.
  - **Lazo abierto:** no completa las maniobras. En el lateral no se movió, el giro de 90° quedó en 26° y
    A→B quedó a 52 cm.
  - **Lazo cerrado con odometría calibrada:** A→B en 15 s a 15 cm de B, repetible y con la orientación
    exacta. El giro de 90° es exacto y el avance de 1 m deja 3–5 cm.
  - **Lateral:** es el movimiento menos preciso. Los rodillos deslizan (−20 %), el robot se corre hacia
    adelante y, con 0,5 kg, termina girado 8–12° sin que los encoders lo vean.
  - **Requisito R‑AGV‑3 (20 mm, 3°):** se cumple en la simulación (4,9 mm) pero no en el piso. Para cumplirlo
    hace falta medir la pose sin pasar por las ruedas (IMU o marcas).
- **CoppeliaSim:** coincide con el cálculo hasta 10⁻¹³ rad.
