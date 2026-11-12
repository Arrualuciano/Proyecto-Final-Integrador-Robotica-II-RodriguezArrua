## 5.C AGV OSOYOO: modelo cinemático, dinámico y del accionamiento

> Código: `agv/octave/` (`agv_parametros.m`, `verificar_cinematica_agv.m`, `simular_agv.m`,
> `ajustar_escalones.m`, `identificacion_simulada.m`, `identificar_lab.m`, `ensayos_agv.m`), firmware
> `agv/firmware/agv_osoyoo/` (mismo algoritmo de control) y figuras `agv/python/figuras_agv.py`.
> Referencias: Lynch y Park, *Modern Robotics* §13.2 (mecanum) y §13.4 (odometría); apuntes Tema V (PI,
> antiwindup) y Tema VI (odometría, saturación con escala común); hoja de datos GM25-370.
> El modelo es de simulación, pero desde el 11/11/2026 sus parámetros principales son **medidos en el kit**
> (marcados M): geometría, cuentas por vuelta, K y zona muerta de cada motor y deslizamiento. Quedan como
> estimados (E) la masa, las inercias y la constante de tiempo.

## 5.C.1 Convenciones y cinemática omnidireccional

- **Marcos.**
  - Marco global {G}: el piso de la celda, con origen en la estación A.
  - Marco del cuerpo {B}: en el centro geométrico, x hacia adelante, y a la izquierda, giro positivo antihorario.
  - Pose ξ = (x, y, φ).
- **Ruedas.** 1 = FL, 2 = FR, 3 = RL, 4 = RR. ω_i > 0 cuando la rueda rueda hacia adelante. Rodillos a 45°
  en patrón X; queda a verificar en el kit, y si resultara O, el firmware invierte el eje lateral (`GEO … sy`).
- **Mapa inverso.** ω = H v_b, con H = (1/r)[1 −1 −L; 1 1 L; 1 1 −L; 1 −1 L] y L = l_x + l_y.
  Valores: r = 0,040 m (manual, Ø 80 mm), l_x = 0,050 m (M) y l_y = 0,100 m (M), medidos con ±1 cm
  (2l_x = 10 cm, 2l_y = 20 cm). L = 0,150 m; el grupo de G. Buffa midió en el mismo kit 0,1525 m.
- **Mapa directo.** v_b = H⁺ω, con H⁺ = (HᵀH)⁻¹Hᵀ.
- **Verificaciones** (`verificar_cinematica_agv.m`):
  - rango(H) = 3 y H⁺H = I (error 2·10⁻¹⁶);
  - **compatibilidad de las órdenes de rueda:** el espacio nulo de Hᵀ es n = (−1, −1, 1, 1), con las
    delanteras "peleando" contra las traseras. Una orden ω es realizable sin deslizar solo si nᵀω = 0. Por
    ejemplo, ω = (5, 5, 5, 0) deja un residuo de ±1,25 rad/s que los rodillos absorben deslizando;
  - **casos:**
    - avance: las 4 ruedas iguales;
    - lateral +y: FL y RR hacia atrás, FR y RL hacia adelante;
    - diagonal: FL y RR quietas;
    - giro antihorario: las izquierdas hacia atrás.
- **Límites de rueda → conjunto admisible del cuerpo.** |v_x| + |v_y| + L|ω| ≤ r·ω_máx (un octaedro).
  - El motor daría ω = K(255 − u_dz) ≈ 22 rad/s en el piso a 7,3 V, pero el firmware limita la rueda a
    **ω_máx = 10 rad/s**: con PWM ≥ 200 la carga de interrupciones de los encoders estiraba el período del lazo
    a 18–31 ms (falla de período medida el 11/11).
  - Con 10 rad/s: r·ω_máx = 0,40 m/s en avance o lateral, 0,28 m/s en diagonal y 2,7 rad/s de giro.
  - Una orden fuera del conjunto se escala por un factor común, lo que conserva la dirección del movimiento.
- **Odometría.** Δ = H⁺Δφ con los incrementos de los encoders (x4, **1482 cuentas/vuelta medidas**: 5 vueltas a
  mano por rueda, FL 1481, FR 1467, RL 1489, RR 1493). La ficha (75:1, 11 PPR) daría 3300; el motor del kit es
  otra variante, de ~33,7:1. Se integra con el
  rumbo en el punto medio y se corrige con factores η_x, η_y, η_ω (real/odometría) obtenidos por calibración.
  **Fuentes de deriva:** deslizamiento de los rodillos (distinto por dirección), radio efectivo, l_x + l_y,
  cuantización y dispersión entre ruedas.

## 5.C.2 Modelo dinámico de la base y del accionamiento

- **Base.** Con la restricción de rodadura ω = Hv_b, la potencia Σ τ_i ω_i = Fᵀv_b lleva los pares de rueda al
  esfuerzo sobre el chasis: (F_x, F_y, M_z) = Hᵀτ. Newton–Euler plano en el cuerpo:

  M_b v̇_b = Hᵀτ + (m ω v_y, −m ω v_x, 0)ᵀ,  M_b = diag(m, m, I_z) + J_w HᵀH,  J_w = N²J_rotor + J_rueda.

  - Con m = 1,5 kg (E), I_z = 0,017 kg·m² (C, por distribución de masas), J_rotor = 2,5 g·cm² (E) y la
    reducción medida de 33,7:1: **J_w = 3,4·10⁻⁴ kg·m²**.
  - **La masa equivalente en traslación es 2,4 kg** (m + 4J_w/r²): el rotor reflejado suma 0,86 kg, más de la
    mitad de la masa del robot. Con la reducción de la ficha (75:1) habría sido 5,2 kg.
- **Reparto de esfuerzos.** El motor i aporta τ_i y la estructura transmite Hᵀτ. La componente de τ sobre n no
  produce movimiento: carga los rodillos y se pierde.
- **Accionamiento, nivel físico** (hoja GM25-370 del lado de la rueda).
  - Modelo: τ_i = K_t,i(V_i − K_e ω_i)/R − b ω_i − τ_C tanh(ω_i/ε), con V_i = u_i V_bat/255.
  - Valores ajustados a la identificación (11/11): R = 2,0 Ω (E), K_e = 0,22 V·s/rad y K_tη = 0,09 N·m/A,
    V_bat = 7,3 V y dispersión entre motores = K de cada rueda / media = (1,02; 1,01; 0,94; 1,04) (M). Los
    valores iniciales de la ficha (0,55 y 0,21, con 75:1) daban K = 0,050, 2,5 veces menos que lo medido.
  - Fricción de Coulomb: τ_C,reductor = 0,075 N·m y τ_C,rodadura = 0,028 N·m, que solo actúa en el piso.
    Reproducen la zona muerta media medida elevado (58 PWM) y en el piso (80 PWM).
- **Accionamiento, nivel identificable** (relación entre ambos niveles).
  - En régimen permanente: ω = K(u − u_dz), con K = (K_t V_bat/(255R))/(K_tK_e/R + b) = 0,124 rad/s/PWM,
    u_dz = 255 τ_C R/(K_t V_bat) y τ = J_w/(K_tK_e/R + b).
  - Con las ruedas elevadas, u_dz = 58 y τ = 0,033 s. En el piso, u_dz = 80 porque se suma la rodadura.
  - La zona muerta es la fricción vista como tensión. K depende casi solo de V_bat/K_e, es decir de la
    tensión de batería, que hay que registrar en cada ensayo.
- **Deslizamiento.** El movimiento sobre el piso es diag(η_d)v_b, con η_d = (1,029; 0,805; 0,995) (M, la
  calibración en el piso de la sección 8.B) y 3 % menos en el lateral por cada kg de carga (E). Los encoders
  no lo ven. **El modelo no incluye** dos efectos medidos en el piso: el acople lateral → avance (el robot
  avanza 6–18 cm al moverse 0,6 m de costado) y el giro por deslizamiento al arrancar el lateral con carga
  (8–12°).

## 5.C.3 Identificación (método probado sobre la planta simulada)

**Procedimiento.**
- Con el robot elevado se aplican 12 escalones de PWM, ±40…255, de 1,5 s cada uno; es el mismo guion del
  laboratorio.
- La velocidad de régimen se toma como la media del último 40 % de cada escalón.
- K y u_dz salen de una regresión por mínimos cuadrados y τ del 63 % de la respuesta.
- **Se ajusta con los escalones impares y se verifica con los pares**: datos distintos, como pide el enunciado.

Tabla: Identificación del accionamiento sobre la planta simulada (ajuste con escalones impares, verificación con pares)

| Rueda | K identificado / físico [rad/s/PWM] | u_dz identificado / físico | τ identificado [s] | Error de verificación [rad/s] |
|---|---|---|---|---|
| FL | 0,0499 / 0,0499 | 25,6 / 25,6 | 0,040 | 0,001 |
| FR | 0,0497 / 0,0497 | 26,9 / 26,9 | 0,050 | 0,001 |
| RL | 0,0501 / 0,0501 | 24,3 / 24,3 | 0,040 | 0,001 |
| RR | 0,0498 / 0,0497 | 26,4 / 26,4 | 0,045 | 0,001 |

- **Resultado.** El método recupera K y u_dz.
- **Constante de tiempo.** τ sale mayor que la mecánica (0,023 s) porque incluye el filtro de velocidad
  (α = 0,5) y el muestreo de 10 ms. Es la constante que efectivamente ve el lazo.
- **Con el kit real**, `identificar_lab.m <carpeta>` hace lo mismo sobre `telemetria.csv` y escribe las
  órdenes `MDL` para el firmware.

## 5.C.4 Identificación en el kit (11/11/2026)

- **Condiciones.** Robot elevado, batería de 7,4–7,5 V, firmware agv-0.1.2. El guion tiene escalones de
  ±40…160 PWM: a 255 la falla de período cortaba el ensayo.
- **Cambio de método.** Con el ajuste impares/verificación pares, a RL le quedaba un solo escalón con la rueda
  girando: su zona muerta es mayor y no gira por debajo de PWM 125. Se ajustó con **todos los escalones en
  régimen con la rueda girando** (|ω| > 0,5 rad/s), con `agv/pc/ajuste_identificacion.py`. La verificación es
  el residuo RMS del ajuste.
- **Zona muerta en el piso.** Se midió aparte, en los primeros movimientos sobre el piso: es el PWM con el que
  cada rueda todavía gira a ~1,5 rad/s al desacelerar. Es la que usa el firmware (orden `MDL`).

Tabla: Identificación del accionamiento en el kit real (registro agv/datos_lab/20261111-150604_s1d_identificacion_7V)

| Rueda | K [rad/s/PWM] | u_dz elevado [PWM] | u_dz en el piso [PWM] | Escalones usados | RMS del ajuste [rad/s] |
|---|---|---|---|---|---|
| FL | 0,1275 | 51 | 66 | 7 | 0,21 |
| FR | 0,1267 | 47 | 79 | 8 | 0,21 |
| RL | 0,1171 | 75 | 104 | 3 | 0,13 |
| RR | 0,1296 | 61 | 72 | 7 | 0,25 |

- **K es 2,5 veces la del modelo inicial** (0,050), coherente con una reducción de ~34:1 y no 75:1. A PWM 160
  la rueda gira a ~14 rad/s.
- **RL es el motor más débil:** 8 % menos K y la mayor zona muerta. Es el que limita el lazo abierto.
- **En el piso la zona muerta sube 15–30 PWM** por la rodadura, como predecía el modelo.
- **Lo que no se midió:** la constante de tiempo (el registro a 50 Hz no resuelve los ~30 ms) y la
  dependencia con la tensión de batería.
