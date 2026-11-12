Tabla: Cumplimiento de los requisitos

| Requisito | Valor pedido | Resultado | Estado |
|---|---|---|---|
| R‑B‑1 alcance horizontal | ≥ 0,38 m | a₂ + d₄ = 0,40 m con la herramienta hacia abajo (límite singular); la tarea trabaja a 0,30 m, lejos de la singularidad (\|cos q₃\| ≥ 0,94) | ✓ |
| R‑B‑2 configuración | 6R, muñeca esférica | a₁ = a₃ = 0; MCI analítica con 8 soluciones | ✓ |
| R‑B‑3 carga máxima | 0,25 kg | Admisible estática 0,379 kg (eje 4) | ✓ |
| R‑B‑4 error de colocación | ≤ 1 mm | 0,15–0,19 mm (control) + 0,30 mm (deformación de L2) | ✓ simulado |
| R‑B‑5 orientación | ≤ 1° | ≤ 0,04° | ✓ simulado |
| R‑B‑6 seguimiento | ≤ 5 mm | ≤ 2,7 mm, incluso con carga máxima y perturbación | ✓ simulado |
| R‑B‑7 ciclo | ≤ 12 s | 10,11 s | ✓ |
| R‑B‑8 límites articulares | 90–180 °/s, 180–360 °/s² | Uso máximo 91 % (q̇₁) | ✓ |
| R‑B‑9 fabricación | PLA, cama 220 × 220 | L2 dividido en dos mitades; todo entra | ✓ |
| R‑AGV‑1 recorrido | 1,2 m + 0,6 m lateral | Trayectoria T4 | ✓ |
| R‑AGV‑2 límites del cuerpo | 0,25 m/s, 0,30 m/s² | Impuestos por la trayectoria y el firmware | ✓ |
| R‑AGV‑3 llegada a B | ≤ 20 mm, ≤ 3° | Simulado: 4,9 mm (sin carga) y 16 mm (0,5 kg). **Físico: 149–152 mm y ≈0° sin carga; 114–200 mm y 8–12° con 0,5 kg** | ✓ simulado; **✗ físico** |
| R‑AGV‑4 tiempo A→B | ≤ 15 s | Simulado 14,3 s; físico 14,7–15,0 s sin carga y 17,6–18,3 s con carga (terminan por tiempo) | ✓ simulado; **límite** sin carga; ✗ con carga |
| R‑AGV‑5 período de ruedas | 10 ms | Medido en movimiento: ejecución ≤ 5,3 ms; peor período p99 11,9 ms, máx. 13,3 ms (velocidad de rueda limitada a 10 rad/s) | ✓ medido |
| R‑AGV‑6 seguridad | Parada, watchdog, límites, reinicio | Actuaron en el kit: traba de rueda, batería, watchdog de comunicación y período; reinicio por watchdog de hardware | ✓ medido |

**Preguntas de investigación** (detalle en 8.C):
1. La compensación dinámica sola mejora unas 5 veces el seguimiento nominal, pero con carga no modelada
   necesita acción integral para cumplir la colocación.
2. N₂ = 30 es necesario para la carga máxima. Su costo es una velocidad de motor mayor y más inercia
   reflejada, ambas aceptables.
3. El lateral es el movimiento menos preciso: 4 veces en simulación y 5 veces en el piso (22 cm contra 4 cm
   del avance). La calibración funciona en simulación, pero en el piso no se transfiere bien entre maniobras
   porque el deslizamiento lateral cambia de una a otra.
4. El PI de ruedas con encoders aporta casi toda la mejora real: el lazo abierto no completa las maniobras
   y el lazo cerrado llega a centímetros. Lo que queda es el error de la odometría misma.
5. La pieza crítica impresa acostada tiene entre 26 y 29 % más margen que parada.

**Alcance de los resultados.**
- Los resultados del brazo son de **simulación**, con un modelo verificado en cada etapa y coherente con el
  CAD y con el entorno gráfico. Su validez depende de las hipótesis de la sección 9.3.
- Los del AGV son **físicos** (sección 8.B), con un modelo ajustado a la identificación del kit:
  - la simulación predice bien lo que modela: avance, error lateral en y y giro;
  - no reproduce la fricción estática, el acople lateral → avance ni el giro por arranques desparejos con
    carga, que son los que impiden cumplir R‑AGV‑3 en el piso.
