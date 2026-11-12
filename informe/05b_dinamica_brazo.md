# 5.B Modelo dinámico y dimensionamiento del brazo 6R (borrador para el informe)

> Código: `brazo/octave/` (`modelo_dinamico_brazo.m`, `rne_brazo.m`, `lagrange_brazo.m`,
> `matrices_dinamicas.m`, `dinamica_directa.m`, `energia_brazo.m`).
> Verificación: `verificar_dinamica.m` y `dimensionamiento.m`. Los registros quedan en `brazo/resultados/`.
> Referencias: Barrientos et al. (2007), cap. 5 (§5.2 Lagrange, §5.3 Newton–Euler, §5.6 actuadores);
> apunte de cátedra Tema III.

## 5.B.1 Formulación

τ = M(q) q̈ + c(q, q̇) + g(q) + τ_f(q̇) − Jᵀ(q) F_ext

- **M** incluye la inercia de los eslabones, la de la pieza y la de los rotores reflejada a la junta,
  N²J_m, sobre la diagonal. Se modela la transmisión como rígida (§3.15 del apunte).
- **c** reúne los términos de Coriolis y centrífugos.
- **g** es el par de sostén gravitatorio, con g₀ = (0, 0, −9,81) m/s².
- **τ_f = b q̇ + τ_C sgn(q̇)** es la fricción del lado articular, con valores estimados en el JSON. En la
  simulación sgn se suaviza con tanh(q̇/10⁻³).
- **F_ext** es la acción del ambiente sobre el robot en el TCP (convención de la cátedra).

**Implementación principal: Newton–Euler recursivo** con todos los vectores en {0} (apunte §3.10).
1. Pasada hacia la punta: ωᵢ, αᵢ y las aceleraciones de los centros de masa.
2. Fuerzas propias de cada eslabón: Fᵢ = mᵢ(a_ci − g₀) y Nᵢ = Iᵢαᵢ + ωᵢ × Iᵢωᵢ.
3. Pasada hacia la base: fᵢ y nᵢ, con condición de borde f₇ = −F_ext en el TCP.
4. Par de cada junta: τᵢ = zᵢ₋₁ᵀ nᵢ.

M, c y g se obtienen evaluando Newton–Euler con q̇ y q̈ nulos o unitarios (método de Walker–Orin). La dinámica
directa resuelve M q̈ = τ − (c + g + τ_f − JᵀF_ext) sin invertir M.

**Contraste independiente: Lagrange.**
- M = Σ_k [m_k J_vkᵀ J_vk + J_ωkᵀ I_k J_ωk] + diag(N²J_m), con los jacobianos de los centros de masa.
- g = −Σ_k m_k J_vkᵀ g₀.
- c_i = Σ_j Ṁ_ij q̇_j − ½ q̇ᵀ(∂M/∂q_i)q̇, con las derivadas de M por diferencias centrales.

## 5.B.2 Masas e inercias

- **Estructura impresa:** a partir del STL que exporta Fusion. El sólido se descompone en tetraedros desde el
  origen y se suman volumen, primer y segundo momento. Luego se aplica Steiner al centro de masa y se rota el
  tensor a la trama DH de cada eslabón: ⁱI_c = R₀ᵢᵀ ⁰I_c R₀ᵢ.
- **Densidad efectiva del PLA:** 800 kg/m³ [estimado]. Supone 4 perímetros y 30 % de relleno; se verifica
  pesando una probeta impresa.
- **Motores:** prismas macizos con la masa del JSON, ubicados en las envolventes del CAD.
- **Prueba del algoritmo con el cubo de la pieza:** volumen e Ixx exactos.
- **Contraste con Fusion (estructura):** los volúmenes coinciden con un error ≤ 0,08 % y los centros de masa
  dentro de 0,01 mm. La diferencia viene de la teselación del STL.

Tabla: Parámetros inerciales de los eslabones (estructura con densidad efectiva más motores)

| Eslabón | m [kg] | de la cual estructura [kg] | r_c en trama i [mm] | diag(I_c) [kg·cm²] |
|---|---|---|---|---|
| L1 hombro | 1,538 | 1,058 | (0; 29,1; 32,8) | (78,1; 64,8; 32,3) |
| L2 brazo (dos mitades) | 0,967 | 0,487 | (−115,6; 0; −30,8) | (16,2; 45,7; 32,9) |
| L3 codo | 0,369 | 0,169 | (0; 0; −23,6) | (8,1; 8,2; 1,0) |
| L4 antebrazo | 0,361 | 0,181 | (21,2; −73,5; 0) | (3,9; 2,8; 5,3) |
| L5 muñeca | 0,089 | 0,029 | (0; 0; 18,5) | (0,26; 0,26; 0,15) |
| L6 brida + pinza | 0,147 | 0,132 | (3,2; 0; 23,1) | (1,31; 0,94; 0,95) |
| Pieza (en el TCP) | 0,100 | — | (0; 0; 80) en trama 6 | m a²/6 = 0,27 |

La masa móvil total es de 3,47 kg y los motores representan más de la mitad.

## 5.B.3 Comprobaciones

| Comprobación | Resultado |
|---|---|
| Newton–Euler vs Lagrange (50 estados aleatorios, pieza de 0,25 kg) | ‖ΔM‖ = 6·10⁻¹⁷ kg·m², ‖Δg‖ = 9·10⁻¹⁶ N·m, ‖Δc‖ = ‖Δτ‖ = 1·10⁻¹⁰ N·m |
| Simetría y definición de M (500 poses) | Asimetría 4·10⁻¹⁷; autovalor mínimo 1,3·10⁻⁴ kg·m² > 0 |
| Caída libre con τ = 0 y sin fricción (1 s, RK4) | E₀ = 7,35 J; error de energía 8·10⁻⁶ J con h = 2 ms y 5·10⁻⁷ J con h = 1 ms. Al reducir h el error baja unas 15 veces, lo esperable para un método de cuarto orden |
| Balance de trabajo con pares aplicados | E(1) − E(0) = −1,89637 J = ∫τᵀq̇ dt (diferencia 3·10⁻⁷ J) |
| Sostén en las poses de la tarea | En la toma, g = (0; −2,83; −0,63; 0; −0,005; 0) N·m sin pieza y g₂ = −3,12 N·m con la pieza. g₅ no cambia porque la herramienta está vertical y el peso de la pieza pasa por el eje 5 ✓ |

## 5.B.4 Accionamientos y carga admisible

**Método.**
- Se recorre una grilla de 14 553 posturas dentro de los límites (q₂, q₃, q₄, q₅) con la carga máxima.
- Para cada junta se toma el máximo de |g_i|, que es el sostén estático más desfavorable.
- Ese par se lleva al motor con τ_m = τ/(ηN) y se compara con τ_m,max/FS.
- FS = 1,5 cubre los pares de aceleración, que se verifican con las trayectorias, y la caída de par del paso a
  paso con la velocidad.

**Resultado con la reducción elegida:**

Tabla: Dimensionamiento estático de los accionamientos (carga 0,25 kg, FS = 1,5)

| Junta | máx \|g\| [N·m] | Postura crítica | N | τ_m requerido [N·m] | τ_m,max [N·m] | Uso con FS |
|---|---|---|---|---|---|---|
| 2 | 4,95 | Brazo estirado horizontal | 30 | 0,206 | 0,45 (NEMA 17) | 69 % |
| 3 | 1,76 | Antebrazo horizontal | 15 | 0,147 | 0,45 | 49 % |
| 4 | 0,52 | Muñeca a 90° con el antebrazo horizontal | 10 | 0,061 | 0,12 (NEMA 14) | 76 % |
| 5 | 0,44 | Herramienta horizontal | 10 | 0,052 | 0,12 | 65 % |
| 6 | 0,005 | — | 5 | 0,001 | 0,05 | 3 % |
| 1 | — | La gravedad no genera par en un eje vertical | 20 | — | — | Lo fija la aceleración (ver trayectorias) |

**Pregunta de investigación: efecto de la reducción en el eje más exigido.**
- Con N₂ = 20, el eje 2 quedaba al 103 % y la carga admisible en todo el espacio articular era 0,221 kg, menor
  que los 0,25 kg pedidos (R‑B‑3).
- Con N₂ = 30, el uso baja al 69 % y la carga admisible sube a **0,379 kg**. El límite pasa al eje 4.
- El costo es que el motor 2 gira más rápido (450 rpm a 90 °/s) y la inercia reflejada aumenta 2,25 veces
  (N²J_m = 0,0049 kg·m²).
- La comparación dinámica a lo largo de la trayectoria completa queda pendiente.

**Carga admisible declarada:**
- **0,25 kg** en todo el espacio articular. Es el requisito R‑B‑3 y queda con margen frente a los 0,379 kg
  del criterio estático.
- En las poses de la tarea, con la pieza de 0,1 kg, el motor más exigido (eje 2) trabaja al 43 % con FS.
- Queda **sin verificar** hasta tener las trayectorias: los picos de par en movimiento y el par del eje 1.
