## 6.B Trayectorias del AGV

> Firmware: `nucleo.cpp` (`nucleo_ir_a`, `T_min_quintico`). Copia en Octave: `agv/octave/simular_agv.m`.

**Construcción.** La orden `GOTO x y φ T modo` genera una referencia quíntica de reposo a reposo en las tres
coordenadas de {G}, (x, y, φ), con la misma ley s(u) = 10u³ − 15u⁴ + 6u⁵ que el brazo.

**Duración.** Se toma la mayor entre la pedida y la mínima que respeta los límites del cuerpo:

T ≥ máx( 15d/(8v_máx), √(10√3·d/(3a_máx)), 15|Δφ|/(8ω_máx), √(10√3·|Δφ|/(3α_máx)) ),

- d es la distancia recorrida en el plano;
- los límites son v_máx = 0,25 m/s, a_máx = 0,30 m/s², ω_máx = 1 rad/s y α_máx = 2 rad/s²
  (`parametros.json`, `agv.control`).

Si la duración pedida viola un límite, el firmware la alarga y la informa en la respuesta.

**Condiciones de inicio y fin.** Velocidad y aceleración nulas en los dos extremos. Con eso no hay saltos de
referencia en los cambios de tramo y la pieza en la bandeja no desliza: a ≪ μg.

Tabla: Trayectorias del AGV (las mismas en simulación y en el laboratorio)

| Ensayo | Referencia | Duración mínima | Límite que la fija |
|---|---|---|---|
| T1 avance | (0, 0, 0) → (1,0; 0; 0) | 7,5 s | v_máx |
| T2 lateral | (0, 0, 0) → (0; 0,6; 0) | 4,5 s | v_máx |
| T3 giro | (0, 0, 0) → (0; 0; π/2) | 2,95 s | ω_máx |
| T4 A→B | (0, 0, 0) → (1,2; 0; 0) → (1,2; 0,6; 0) | 9,0 + 4,5 s | v_máx (R‑AGV‑4: 13,7 s ≤ 15 s con el tiempo de asentamiento) |

**Saturación de ruedas.** Si la combinación de la referencia y la corrección del lazo de pose pide una
velocidad de rueda mayor que ω_rueda,máx = 10 rad/s, las cuatro ruedas se escalan por un mismo factor.
Así se conserva la dirección del movimiento (sección 5.C.1).
