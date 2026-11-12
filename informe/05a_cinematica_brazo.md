# 5.A Modelo cinemático del brazo 6R (borrador para el informe)

> Fuente de datos: `parametros/parametros.json`. Código: `brazo/octave/` (`mcd_brazo.m`, `mci_brazo.m`,
> `jacobiano_brazo.m`). Verificación reproducible: `verificar_cinematica.m`, cuyo registro queda en
> `brazo/resultados/verificacion_cinematica.txt`.
> Convenciones: SI y radianes; DH estándar ⁱ⁻¹Aᵢ = Rz(θᵢ)·Tz(dᵢ)·Tx(aᵢ)·Rx(αᵢ), con θᵢ = qᵢ + offsetᵢ
> [Barrientos et al., 2007, §4.1]. Giros positivos según la regla de la mano derecha sobre zᵢ₋₁.

## 5.A.1 Tramas y tabla de Denavit–Hartenberg

La trama {0} está en la intersección del eje 1 con el plano de apoyo de la base, con z₀ vertical hacia arriba
y x₀ hacia la estación de toma. En la **pose cero** (q = 0) el brazo queda vertical y el antebrazo y la
herramienta horizontales hacia +x₀. Es la pose en la que se dibuja el CAD.

Tabla: Parámetros de Denavit–Hartenberg (estándar) del brazo

| i | θᵢ | dᵢ [m] | aᵢ [m] | αᵢ | Eje zᵢ₋₁ en la pose cero |
|---|---|---|---|---|---|
| 1 | q₁ | 0,18 | 0 | −90° | +z₀ (giro de la base) |
| 2 | q₂ − 90° | 0 | 0,20 | 0 | +y₀ (hombro) |
| 3 | q₃ | 0 | 0 | −90° | +y₀ (codo) |
| 4 | q₄ | 0,20 | 0 | +90° | +x₀ (giro del antebrazo) |
| 5 | q₅ | 0 | 0 | −90° | +y₀ (cabeceo de la muñeca) |
| 6 | q₆ | 0,05 | 0 | 0 | +x₀ (giro de la brida) |
| TCP | — | 0,08 sobre z₆ | — | — | — |

- El offset de −90° en θ₂ es el desfasaje entre el cero del motor (brazo vertical) y el cero que resulta de
  asignar las tramas. Se carga como offset del eje.
- a₁ = a₃ = 0 simplifica el KUKA KR 6 R700, que tiene 25 y 35 mm. Gracias a eso los ejes 4, 5 y 6 se cortan
  en el centro de muñeca W y el brazo cumple la condición de **muñeca esférica**.
- Sentidos positivos en la pose cero: q₂ > 0 inclina el brazo hacia adelante y q₃ > 0 baja el antebrazo.

## 5.A.2 Cinemática directa

⁰T_TCP = ⁰A₁ ¹A₂ ²A₃ ³A₄ ⁴A₅ ⁵A₆ · Tz(0,08). La posición del centro de muñeca en el plano vertical del brazo es

ρ = a₂ sin q₂ + d₄ cos(q₂+q₃),   z_W = d₁ + a₂ cos q₂ − d₄ sin(q₂+q₃),   W = (ρ cos q₁, ρ sin q₁, z_W).

**Comprobación en configuraciones conocidas** (error < 10⁻¹⁶ m en todas):

| q [°] | Lectura física | TCP esperado [m] |
|---|---|---|
| 0 | pose cero | (0,33; 0; 0,38) |
| q₁ = 90 | girado hacia +y | (0; 0,33; 0,38) |
| q₅ = 90 | herramienta hacia abajo | (0,20; 0; 0,25) |
| q₂ = 90, q₃ = −90 | brazo estirado horizontal | (0,53; 0; 0,18) |
| q₃ = 90 | antebrazo hacia abajo | (0; 0; 0,05) |

## 5.A.3 Cinemática inversa analítica (desacople cinemático)

Dada la pose deseada (p, R) del TCP:

1. **Centro de muñeca**: p_W = p − (d₆ + h)·a, con a = tercera columna de R y h = 0,08 m.
2. **Eje 1**: q₁ = atan2(p_Wy, p_Wx) (hombro adelante, ρ = +r) o q₁ + π (hombro atrás, ρ = −r), con
   r = √(p_Wx² + p_Wy²) y s = p_Wz − d₁.
3. **Ejes 2 y 3**, por el teorema del coseno en el triángulo hombro–codo–muñeca:
   ρ² + s² = a₂² + d₄² − 2a₂d₄ sin q₃ ⇒ sin q₃ = (a₂² + d₄² − ρ² − s²)/(2a₂d₄);
   q₃ = atan2(sin q₃, ±√(1 − sin²q₃)): el signo + es **codo arriba**.
   q₂ = atan2(k₁ρ − k₂s, k₂ρ + k₁s), con k₁ = a₂ − d₄ sin q₃ y k₂ = d₄ cos q₃.
4. **Muñeca**: ³R₆ = (⁰R₃)ᵀ R. Por la estructura de ³R₆:
   ³R₆(:,3) = (−c₄s₅, −s₄s₅, c₅)ᵀ y ³R₆(3,1:2) = (s₅c₆, −s₅s₆), de donde
   q₅ = atan2(±√(r₁₃² + r₂₃²), r₃₃),  q₄ = atan2(−r₂₃/s₅, −r₁₃/s₅),  q₆ = atan2(−r₃₂/s₅, r₃₁/s₅).

Se obtienen 2 (hombro) × 2 (codo) × 2 (muñeca) = **8 soluciones**. Siempre se usa atan2 para resolver el cuadrante.

**Selección y continuidad.** Cada solución se lleva al equivalente módulo 2π más cercano a la configuración
actual y se descartan las que salen de los límites. Entre las válidas se elige la de menor distancia
ponderada a la configuración actual. Así no hay saltos de rama a lo largo de una trayectoria.

**Singularidades resueltas por convención.** Si r = 0, q₁ toma el valor actual. Si sin q₅ = 0, q₄ toma el valor
actual y q₆ se despeja de la suma (o la diferencia) q₄ ± q₆.

**Verificación** (2000 poses aleatorias dentro de los límites):

- error máximo ‖MCD(MCI(T)) − T‖ = 1,1·10⁻¹³;
- la configuración original aparece entre las 8 soluciones en el 100 % de los casos.

## 5.A.4 Jacobiano y singularidades

Jacobiano geométrico del TCP en {0}, con todas las articulaciones rotacionales:
Jᵢ = [ zᵢ₋₁ × (p_TCP − oᵢ₋₁) ; zᵢ₋₁ ]. Es la forma matricial de la propagación de velocidades base →
herramienta (Tema III).

- **Verificación**: la diferencia con la derivada numérica de la MCD (diferencias centrales, 200 poses) es 3,2·10⁻¹⁰.
- **Determinante**: det J = −a₂ d₄ · ρ · cos q₃ · sin q₅. Ajusta en 200 poses con un desvío de 10⁻¹⁴.

Hay tres singularidades:

| Factor nulo | Singularidad | Interpretación física |
|---|---|---|
| ρ = 0 | de hombro (interior) | El centro de muñeca está sobre el eje 1; girar la base no lo mueve |
| cos q₃ = 0 | de codo (frontera) | Brazo totalmente estirado (alcance máximo) o plegado sobre sí mismo |
| sin q₅ = 0 | de muñeca (interior) | Los ejes 4 y 6 quedan alineados y hacen lo mismo |

## 5.A.5 Poses de la tarea

Herramienta hacia abajo, R_toma = diag(1, −1, −1). Los dedos cierran según y₀. La colocación está girada
90° alrededor de z₀.

Tabla: Poses de la tarea y distancia a las singularidades

| Pose | p [m] | q [°] | ρ [m] | cos q₃ | sin q₅ |
|---|---|---|---|---|---|
| Aproximación de la toma | (0,30; 0; 0,10) | (0; 40,0; −9,0; 0; 59,0; 0) | 0,300 | 0,988 | 0,857 |
| Toma | (0,30; 0; 0,02) | (0; 54,6; −7,8; 0; 43,2; 0) | 0,300 | 0,991 | 0,685 |
| Aproximación de la colocación | (0; 0,30; 0,10) | (90; 40,0; −9,0; 0; 59,0; 0) | 0,300 | 0,988 | 0,857 |
| Colocación | (0; 0,30; 0,02) | (90; 54,6; −7,8; 0; 43,2; 0) | 0,300 | 0,991 | 0,685 |

Las cuatro poses están lejos de las tres singularidades y usan la configuración hombro adelante, codo arriba y
muñeca +. En el descenso cartesiano de 80 mm (100 muestras) el mayor cambio articular entre muestras es
0,18°: la solución es continua.

**Jacobiano inverso.** Para bajar a 0,05 m/s sin girar la herramienta, en la toma hacen falta
q̇ = (0; 10,5; −2,2; 0; −8,4; 0) °/s.

## 5.A.6 Relación estática entre fuerzas en la herramienta y pares articulares

Por igualdad de potencias, Fᵀ ẋ = τᵀ q̇ ⇒ τ = Jᵀ F (Tema III). Con la convención de la cátedra
(F_ext = fuerza del ambiente sobre el robot), el par motor de sostén es τ = g(q) − Jᵀ F_ext.

Caso de carga de la tarea, en la pose de colocación y solo con cargas externas (el peso propio entra en el
modelo dinámico):

| Carga en el TCP | τ_motor [N·m] (ejes 1…6) |
|---|---|
| Peso de la pieza, 0,1 kg | (0; −0,294; −0,134; 0; 0; 0) |
| Peso + reacción de inserción de 5 N hacia arriba | (0; +1,206; +0,550; 0; 0; 0) |

- Signo: el peso adelante del hombro tiende a inclinar el brazo hacia adelante (q₂ creciente), así que el motor
  aplica par negativo: 0,30 m · 0,981 N = 0,294 N·m ✓.
- Se comprobó la igualdad Fᵀ(J q̇) = (JᵀF)ᵀ q̇ (diferencia 3,5·10⁻¹⁷).
