### 3.1 Brazos antropomórficos: referencia industrial y diseños imprimibles

- **Referencia industrial.** El brazo se dimensionó como una versión simplificada y a escala del KUKA KR 6 R700,
  el robot 3 del TP1 de la cátedra. Su cadena es 6R con muñeca esférica: d₁ = 400, a₁ = 25, a₂ = 315,
  a₃ = 35, d₄ = 365 y d₆ = 80 mm.
  - Se tomaron a₁ = a₃ = 0. Así los ejes de la muñeca se cortan en un punto, el problema inverso se desacopla
    en posición y orientación y se resuelve en forma cerrada [Barrientos et al., 2007, §4.3].
  - Las longitudes se redujeron a 0,18 / 0,20 / 0,20 / 0,05 m, compatibles con la impresión 3D y con la tarea.
- **Brazos imprimibles de código abierto.** Se revisaron dos antecedentes:
  - **BCN3D Moveo**: desarrollo educativo de BCN3D con la Generalitat de Catalunya. Usa motores paso a paso
    con reducción por correa y electrónica basada en Arduino.
  - **Thor** (AngelLM): 6 grados de libertad, 625 mm de alto y 750 g de carga declarada. Usa engranajes
    impresos, poleas y correas GT2 y motores paso a paso.
  - Los dos confirman que la arquitectura elegida es viable con PLA y motores NEMA 17/14 con reducción.
  - Muestran además las dos transmisiones más usadas en este tipo de brazos: correas con poleas, y reductores
    planetarios o cicloidales impresos.
  - De ellos se tomó el criterio de ubicar los motores cerca de la base para bajar la inercia de los
    eslabones distales.
- **Alternativa considerada:** usar servos de modelismo en lugar de pasos a paso. Se descartó porque no dan
  el par del eje 2 (≈ 5 N·m) y porque no permiten un lazo de par, que es lo que suponen las leyes de control
  dinámico de la sección 7.

### 3.2 Material impreso y dirección de capas

El PLA impreso por deposición fundida (FDM) es **anisótropo**:
- la resistencia a lo largo de los filamentos es mayor que la de la adhesión entre capas;
- Eryildiz (2021) midió 55,5 MPa en probetas planas y 35,5 MPa en probetas paradas, un 36 % menos;
- Chacón et al. (2017) encontraron la misma jerarquía: la orientación parada es la más débil, y el espesor de
  capa y la velocidad de alimentación también influyen.

Estos datos fijan la orientación de impresión de la pieza crítica (sección 4.B).

### 3.3 Plataformas omnidireccionales con ruedas mecanum

- **Origen y principio.** La rueda mecanum (Ilon, 1975) tiene rodillos pasivos a 45° alrededor de su
  periferia. Cada rueda solo impone la velocidad en la dirección perpendicular al eje de sus rodillos; con
  cuatro ruedas en patrón X, la base obtiene tres grados de libertad en el plano.
- **Modelo.** El modelo cinemático (mapa entre velocidades de rueda y del cuerpo, con su pseudoinversa) y la
  odometría se tomaron de Lynch y Park (2017), §13.2 y §13.4.
  - El apunte de la cátedra (Tema VI) trata el accionamiento diferencial, así que el modelo mecanum se
    desarrolló a partir de esa fuente.
  - Esa misma fuente advierte que el deslizamiento de los rodillos degrada la odometría, sobre todo en el
    desplazamiento lateral.
- **Kit.** OSOYOO «M2.0 Metal Mecanum» para Arduino Mega 2560: chasis metálico, cuatro motorreductores
  GM25‑370 de 75:1 con encoder Hall y driver «Model Y» (2 × TB6612). La documentación se tomó del manual
  del fabricante y de la hoja de datos del motor que reunió un compañero de cursada
  (`referencias/analisis_proyecto_koba.md`).
  - El manual no informa la relación de reducción efectiva, las cuentas por vuelta ni la geometría entre
    ruedas: se relevan en el laboratorio.
  - Los encoders no vienen conectados de fábrica.

### 3.4 Control

- **Leyes de control del brazo.**
  - El **PD con compensación de gravedad** es la ley de referencia, con estabilidad asintótica en
    regulación.
  - El **par calculado** cancela la dinámica no lineal con el modelo inverso y deja el error como un sistema
    lineal desacoplado [Barrientos et al., 2007, §7.2; Spong et al., 2006, cap. 8; apunte Tema V].
  - Su debilidad conocida es la sensibilidad al error de modelo. Para corregirla se agregó una acción
    integral con antiwindup por integración condicionada (Tema V §5.4).
- **Dinámica inversa.** Se calcula con Newton–Euler recursivo. M, c y g salen del método de Walker y Orin
  (1982), que evalúa el mismo algoritmo con entradas particulares.
- **Control del AGV.** La arquitectura de dos niveles (velocidad de rueda con encoders y pose con odometría)
  es la habitual en plataformas omnidireccionales [Lynch y Park, 2017, §13.2].

### 3.5 Simulación gráfica

CoppeliaSim permite tres modos de articulación:
- cinemático: la posición se impone desde un script;
- dinámico: con controladores internos de posición o velocidad;
- de par.

Se eligió el **modo cinemático** (manual oficial, «Joint modes»), para que la escena muestre exactamente la
trayectoria que calculó el lazo de control en Octave sin que intervenga el motor físico del simulador. La
escena se construye y verifica por la API remota ZMQ (Coppelia Robotics, versión 4.10).
