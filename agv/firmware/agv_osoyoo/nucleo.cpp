// nucleo.cpp — Nucleo portable del AGV. No usa Arduino: se puede compilar en la PC para pruebas.
// Referencias: Lynch y Park, Modern Robotics §13.2 (mecanum) y §13.4 (odometria); apunte Tema IV
// (quintico, limites de duracion) y Tema V (PI, antiwindup por integracion condicionada).
#include "nucleo.h"
#include <math.h>
#include <string.h>

static const float PI_F = 3.14159265f;
static const float W_MIN = 0.05f;          // rad/s: por debajo se considera referencia nula

static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float signf(float x) { return x > 0.0f ? 1.0f : (x < 0.0f ? -1.0f : 0.0f); }

float envolver(float a) {
  a = fmodf(a + PI_F, 2.0f * PI_F);
  if (a < 0.0f) a += 2.0f * PI_F;
  return a - PI_F;
}

void nucleo_parametros_defecto(Parametros &p) {
  memset(&p, 0, sizeof(p));
  // Desde la 0.2.1 los valores por defecto son los MEDIDOS en el kit (07-11/11/2026): si la EEPROM no tiene
  // imagen valida (p. ej. despues de cargar otro programa en la Mega) el robot arranca igual calibrado.
  p.r = 0.040f; p.lx = 0.050f; p.ly = 0.100f; p.sy = 1.0f;     // Ø80 (manual OSOYOO); 2lx = 10 cm, 2ly = 20 cm medidos
  const float K0[4] = {0.1275f, 0.1267f, 0.1171f, 0.1296f};      // identificacion 11/11 (rad/s por PWM)
  const float U0[4] = {66.0f, 79.0f, 104.0f, 72.0f};             // zona muerta en el piso (PWM)
  for (uint8_t i = 0; i < 4; i++) {
    p.cpr[i] = 1482.0f;                                         // 5 vueltas a mano por rueda (x4); no 3300 de la ficha
    p.K[i] = K0[i];
    p.udz[i] = U0[i];
  }
  p.signo_motor = 15; p.signo_enc = 5;                          // sentidos verificados con el robot elevado
  p.kp = 8.0f; p.ki = 80.0f; p.alfa_vel = 0.5f;
  p.kx = 1.6f; p.ky = 1.6f; p.kphi = 2.4f;
  p.vmax = 0.25f; p.amax = 0.30f;   // 0,25 m/s: A->B en < 15 s (R-AGV-4); ver parametros.json
  p.wmax = 1.0f; p.alfamax = 2.0f;  // rad/s, rad/s^2 (en la v0.1.0 quedaron comentados por error: valian 0)
  p.wrueda_max = 10.0f;
  // calibracion en el piso del 11/11 (T1-T3, modo 2). El Loader borra la EEPROM al flashear (fusible EESAVE sin
  // programar, hfuse D8): por eso los valores medidos van tambien como valores por defecto.
  p.eta_x = 1.029f; p.eta_y = 0.805f; p.eta_w = 0.995f;
  p.ttl_ms = 500; p.wd_ms = 600; p.traba_ms = 1500;             // wd 600 ms por la latencia WiFi; traba 1,5 s en el piso
  p.bat_check = 1; p.bat_min_mV = 6400;
}

void nucleo_reset(Estado &s) {
  memset(&s, 0, sizeof(s));
  s.modo = M_DESARMADO;
}

// ------------------------------------------------------------------ cinematica (patron X)
// w = H vb, H = (1/r) [1 -1 -L; 1 1 L; 1 1 -L; 1 -1 L], L = lx + ly, vy multiplicado por sy.
void cin_inversa(const Parametros &p, const float vb[3], float w[4]) {
  const float L = p.lx + p.ly, vx = vb[0], vy = p.sy * vb[1], wz = vb[2];
  w[0] = (vx - vy - L * wz) / p.r;   // FL
  w[1] = (vx + vy + L * wz) / p.r;   // FR
  w[2] = (vx + vy - L * wz) / p.r;   // RL
  w[3] = (vx - vy + L * wz) / p.r;   // RR
}
// vb = H+ w. Las columnas de H son ortogonales: H+ = r [1/4 ...; ...; .../(4L)].
// La combinacion -w0 - w1 + w2 + w3 (delanteras contra traseras, espacio nulo de H^T) no produce
// movimiento: H+ la filtra (verificado en agv/octave/verificar_cinematica_agv.m).
void cin_directa(const Parametros &p, const float w[4], float vb[3]) {
  const float L = p.lx + p.ly;
  vb[0] = p.r * (w[0] + w[1] + w[2] + w[3]) * 0.25f;
  vb[1] = p.sy * p.r * (-w[0] + w[1] + w[2] - w[3]) * 0.25f;
  vb[2] = p.r * (-w[0] + w[1] - w[2] + w[3]) / (4.0f * L);
}

// ------------------------------------------------------------------ ordenes
void nucleo_falla(Estado &s, uint8_t bits) {
  s.falla |= bits; s.modo = M_FALLA;
  for (uint8_t i = 0; i < 4; i++) { s.u[i] = 0; s.integ[i] = 0; s.w_ref[i] = 0; }
  s.vb[0] = s.vb[1] = s.vb[2] = 0;
}
uint8_t nucleo_armar(Estado &s) {
  if (s.modo == M_FALLA) return 2;
  if (s.modo == M_DESARMADO) s.modo = M_QUIETO;
  return 0;
}
void nucleo_desarmar(Estado &s) {
  if (s.modo == M_FALLA) return;
  s.modo = M_DESARMADO;
  for (uint8_t i = 0; i < 4; i++) { s.u[i] = 0; s.integ[i] = 0; s.w_ref[i] = 0; }
  s.vb[0] = s.vb[1] = s.vb[2] = 0;
}
void nucleo_parar(Estado &s) {
  if (s.modo == M_VEL || s.modo == M_IRA) {          // rampa con amax / alfamax hasta detener
    s.modo = M_VEL; s.v_manual[0] = s.v_manual[1] = s.v_manual[2] = 0;
  } else if (s.modo == M_PWM || s.modo == M_PASO) {
    s.modo = M_QUIETO;
  }
}
uint8_t nucleo_limpiar(Estado &s) {
  if (s.modo != M_FALLA) return 1;
  s.falla = 0; s.modo = M_DESARMADO;
  for (uint8_t i = 0; i < 4; i++) { s.traba_t[i] = 0; s.integ[i] = 0; }
  s.bat_t = 0;
  return 0;
}
uint8_t nucleo_pwm(Estado &s, const int16_t u[4], uint32_t ahora_ms) {
  if (s.modo != M_QUIETO && s.modo != M_PWM) return 3;
  for (uint8_t i = 0; i < 4; i++) s.u_manual[i] = u[i] > 255 ? 255 : (u[i] < -255 ? -255 : u[i]);
  s.t_orden_ms = ahora_ms; s.modo = M_PWM;
  return 0;
}
uint8_t nucleo_paso_escalon(Estado &s, uint8_t rueda, int16_t u, uint16_t ms, uint32_t ahora_ms) {
  if (s.modo != M_QUIETO) return 3;
  if (rueda > 4 || ms < 100 || ms > 10000) return 4;
  s.paso_rueda = rueda; s.paso_u = u > 255 ? 255 : (u < -255 ? -255 : u);
  s.paso_ms = ms; s.paso_t0 = ahora_ms; s.modo = M_PASO;
  return 0;
}
uint8_t nucleo_vel(Estado &s, const Parametros &p, float vx, float vy, float wz, uint32_t ahora_ms) {
  if (s.modo != M_QUIETO && s.modo != M_VEL) return 3;
  const float v = sqrtf(vx * vx + vy * vy);
  if (v > p.vmax) { vx *= p.vmax / v; vy *= p.vmax / v; }
  s.v_manual[0] = vx; s.v_manual[1] = vy; s.v_manual[2] = clampf(wz, -p.wmax, p.wmax);
  if (s.modo == M_QUIETO) { s.vb[0] = s.vb[1] = s.vb[2] = 0; }
  s.t_orden_ms = ahora_ms; s.modo = M_VEL;
  return 0;
}
void nucleo_set_pose(Estado &s, float x, float y, float phi) { s.x = x; s.y = y; s.phi = phi; }

// Duracion minima de un quintico de reposo a reposo: |qd|max = 15 d / (8 T), |qdd|max = 10 sqrt(3) d / (3 T^2)
static float T_min_quintico(float d, float vmax, float amax) {
  if (d <= 0.0f) return 0.0f;
  const float Tv = 1.875f * d / vmax;
  const float Ta = sqrtf(5.7735f * d / amax);
  return Tv > Ta ? Tv : Ta;
}
uint8_t nucleo_ir_a(Estado &s, const Parametros &p, float x, float y, float phi, float T, uint8_t modo,
                    float *T_efectivo) {
  if (s.modo != M_QUIETO) return 3;
  if (modo > 2 || T < 0.0f || T > 120.0f) return 4;
  s.tr_x0[0] = s.x; s.tr_x0[1] = s.y; s.tr_x0[2] = s.phi;
  s.tr_d[0] = x - s.x; s.tr_d[1] = y - s.y; s.tr_d[2] = envolver(phi - s.phi);   // giro mas corto
  const float dist = sqrtf(s.tr_d[0] * s.tr_d[0] + s.tr_d[1] * s.tr_d[1]);
  float Tm = T_min_quintico(dist, p.vmax, p.amax);
  const float Tr = T_min_quintico(fabsf(s.tr_d[2]), p.wmax, p.alfamax);
  if (Tr > Tm) Tm = Tr;
  float Te = T > Tm ? T : Tm;
  if (Te < 0.5f) Te = 0.5f;
  s.tr_T = Te; s.tr_t = 0.0f; s.tr_modo = modo; s.tr_fin = 0;
  s.vb[0] = s.vb[1] = s.vb[2] = 0;
  for (uint8_t i = 0; i < 4; i++) s.integ[i] = 0;
  s.modo = M_IRA;
  if (T_efectivo) *T_efectivo = Te;
  return 0;
}

// ------------------------------------------------------------------ lazo de ruedas
static void ruedas(Estado &s, const Parametros &p, float dt, bool cerrado) {
  float w[4];
  cin_inversa(p, s.vb, w);
  float m = 0.0f;                                          // desaturacion con escala comun
  for (uint8_t i = 0; i < 4; i++) if (fabsf(w[i]) > m) m = fabsf(w[i]);
  if (m > p.wrueda_max) { const float k = p.wrueda_max / m; for (uint8_t i = 0; i < 4; i++) w[i] *= k; }
  for (uint8_t i = 0; i < 4; i++) {
    s.w_ref[i] = w[i];
    const float ff = fabsf(w[i]) > W_MIN ? signf(w[i]) * (fabsf(w[i]) / p.K[i] + p.udz[i]) : 0.0f;
    if (!cerrado) { s.u[i] = (int16_t)clampf(ff, -255.0f, 255.0f); s.integ[i] = 0; continue; }
    if (fabsf(w[i]) < W_MIN && fabsf(s.w_med[i]) < 0.3f) { s.u[i] = 0; s.integ[i] = 0; continue; }  // sin zumbido
    const float e = w[i] - s.w_med[i];
    const float u_tent = ff + p.kp * e + p.ki * (s.integ[i] + e * dt);
    const bool satura = fabsf(u_tent) > 255.0f;
    if (!satura || signf(u_tent) != signf(e)) s.integ[i] += e * dt;   // integracion condicionada
    s.u[i] = (int16_t)clampf(ff + p.kp * e + p.ki * s.integ[i], -255.0f, 255.0f);
  }
}

// ------------------------------------------------------------------ paso principal
void nucleo_paso(Estado &s, const Parametros &p, const int32_t cuentas[4], float dt,
                 uint16_t vbat_mV, uint32_t ahora_ms, bool tick_pose) {
  const float dt_ms = dt * 1000.0f;
  // 1) medicion de ruedas
  int32_t dc[4] = {0, 0, 0, 0};
  if (!s.cnt_init) { for (uint8_t i = 0; i < 4; i++) s.cnt_prev[i] = cuentas[i]; s.cnt_init = 1; }
  float ds[4];
  for (uint8_t i = 0; i < 4; i++) {
    dc[i] = (int32_t)((uint32_t)cuentas[i] - (uint32_t)s.cnt_prev[i]);
    s.cnt_prev[i] = cuentas[i];
    if (p.signo_enc & (1u << i)) dc[i] = -dc[i];
    const float dphi = 2.0f * PI_F * (float)dc[i] / p.cpr[i];
    const float wb = dt > 0.0f ? dphi / dt : 0.0f;
    s.w_med[i] = p.alfa_vel * wb + (1.0f - p.alfa_vel) * s.w_med[i];
    ds[i] = p.r * dphi;                                     // arco recorrido por la rueda
  }
  // 2) odometria con rumbo en el punto medio (integracion exacta para curvatura constante a 2do orden)
  {
    const float L = p.lx + p.ly;
    const float dx = (ds[0] + ds[1] + ds[2] + ds[3]) * 0.25f * p.eta_x;
    const float dy = p.sy * (-ds[0] + ds[1] + ds[2] - ds[3]) * 0.25f * p.eta_y;
    const float dth = (-ds[0] + ds[1] - ds[2] + ds[3]) / (4.0f * L) * p.eta_w;
    const float h = s.phi + 0.5f * dth;
    s.x += dx * cosf(h) - dy * sinf(h);
    s.y += dx * sinf(h) + dy * cosf(h);
    s.phi += dth;
  }
  // 3) bateria
  if (s.vbat_filt <= 0.0f) s.vbat_filt = vbat_mV; else s.vbat_filt = 0.95f * s.vbat_filt + 0.05f * vbat_mV;
  if (p.bat_check && s.modo != M_DESARMADO && s.modo != M_FALLA && s.vbat_filt < p.bat_min_mV) {
    s.bat_t += (uint16_t)dt_ms;
    if (s.bat_t > 1000) { nucleo_falla(s, F_BAT); return; }
  } else s.bat_t = 0;

  // 4) modos
  bool cerrado = false, moviendo = true;
  switch (s.modo) {
    case M_DESARMADO: case M_QUIETO: case M_FALLA:
      for (uint8_t i = 0; i < 4; i++) { s.u[i] = 0; s.integ[i] = 0; s.w_ref[i] = 0; }
      s.vb[0] = s.vb[1] = s.vb[2] = 0;
      moviendo = false;
      break;
    case M_PWM:
      if ((uint32_t)(ahora_ms - s.t_orden_ms) > p.ttl_ms) { s.modo = M_QUIETO; for (uint8_t i = 0; i < 4; i++) s.u[i] = 0; moviendo = false; break; }
      for (uint8_t i = 0; i < 4; i++) { s.u[i] = s.u_manual[i]; s.w_ref[i] = 0; }
      break;
    case M_PASO: {
      if ((uint32_t)(ahora_ms - s.paso_t0) >= s.paso_ms) { s.modo = M_QUIETO; for (uint8_t i = 0; i < 4; i++) s.u[i] = 0; moviendo = false; break; }
      for (uint8_t i = 0; i < 4; i++) { s.u[i] = (s.paso_rueda == 4 || s.paso_rueda == i) ? s.paso_u : 0; s.w_ref[i] = 0; }
      break;
    }
    case M_VEL: {
      if ((uint32_t)(ahora_ms - s.t_orden_ms) > p.ttl_ms) { s.v_manual[0] = s.v_manual[1] = s.v_manual[2] = 0; }
      const float dv = p.amax * dt, dw = p.alfamax * dt;   // rampa de aceleracion
      for (uint8_t k = 0; k < 2; k++) s.vb[k] += clampf(s.v_manual[k] - s.vb[k], -dv, dv);
      s.vb[2] += clampf(s.v_manual[2] - s.vb[2], -dw, dw);
      cerrado = true;
      const bool cero = s.v_manual[0] == 0 && s.v_manual[1] == 0 && s.v_manual[2] == 0 &&
                        fabsf(s.vb[0]) < 1e-4f && fabsf(s.vb[1]) < 1e-4f && fabsf(s.vb[2]) < 1e-4f;
      bool quietas = true;
      for (uint8_t i = 0; i < 4; i++) if (fabsf(s.w_med[i]) > 0.3f) quietas = false;
      if (cero && quietas) { s.modo = M_QUIETO; for (uint8_t i = 0; i < 4; i++) { s.u[i] = 0; s.integ[i] = 0; } moviendo = false; break; }
      ruedas(s, p, dt, true);
      break;
    }
    case M_IRA: {
      cerrado = s.tr_modo >= 1;
      if (s.pausa_com) {
        // corte breve del enlace (WiFi compartido): se congela el tiempo de la trayectoria y se frena; al volver
        // los mensajes sigue sola desde donde quedo (idea tomada del firmware BioLog de Mendelevich y Baldoni)
        s.vb[0] = s.vb[1] = s.vb[2] = 0;
        ruedas(s, p, dt, cerrado);
        break;
      }
      s.tr_t += dt;
      if (tick_pose) {
        const float T = s.tr_T, u = clampf(s.tr_t / T, 0.0f, 1.0f);
        const float u2 = u * u, u3 = u2 * u;
        const float sp = u3 * (10.0f - 15.0f * u + 6.0f * u2);
        const float sv = (u < 1.0f) ? (30.0f * u2 - 60.0f * u3 + 30.0f * u2 * u2) / T : 0.0f;
        float Vw[3], e[3] = {0, 0, 0};
        for (uint8_t k = 0; k < 3; k++) { s.ref[k] = s.tr_x0[k] + s.tr_d[k] * sp; s.ref_d[k] = s.tr_d[k] * sv; Vw[k] = s.ref_d[k]; }
        float phi_u = s.ref[2];
        if (s.tr_modo == 2) {
          e[0] = s.ref[0] - s.x; e[1] = s.ref[1] - s.y; e[2] = envolver(s.ref[2] - s.phi);
          Vw[0] += p.kx * e[0]; Vw[1] += p.ky * e[1]; Vw[2] += p.kphi * e[2];
          phi_u = s.phi;
        }
        const float c = cosf(phi_u), sn = sinf(phi_u);
        s.vb[0] = c * Vw[0] + sn * Vw[1];
        s.vb[1] = -sn * Vw[0] + c * Vw[1];
        s.vb[2] = Vw[2];
        // limite de seguridad del comando (la correccion de pose no puede pedir mas que 1,3 veces los topes)
        const float v = sqrtf(s.vb[0] * s.vb[0] + s.vb[1] * s.vb[1]), vl = 1.3f * p.vmax;
        if (v > vl) { s.vb[0] *= vl / v; s.vb[1] *= vl / v; }
        s.vb[2] = clampf(s.vb[2], -1.3f * p.wmax, 1.3f * p.wmax);
        if (u >= 1.0f) {
          const bool llego = sqrtf(e[0] * e[0] + e[1] * e[1]) < 0.005f && fabsf(e[2]) < 0.01f;
          if (s.tr_modo < 2 || llego || s.tr_t > T + 2.0f) {
            s.tr_fin = 1; s.modo = M_QUIETO;
            for (uint8_t i = 0; i < 4; i++) { s.u[i] = 0; s.integ[i] = 0; s.w_ref[i] = 0; }
            s.vb[0] = s.vb[1] = s.vb[2] = 0; moviendo = false; break;
          }
        }
      }
      ruedas(s, p, dt, cerrado);
      break;
    }
  }

  // 5) protecciones durante el movimiento
  if (moviendo && s.modo != M_FALLA) {
    for (uint8_t i = 0; i < 4; i++) {
      const int16_t ua = s.u[i] < 0 ? -s.u[i] : s.u[i];
      if (ua >= (int16_t)(p.udz[i] + 30.0f) && dc[i] == 0) {
        s.traba_t[i] += (uint16_t)dt_ms;
        if (s.traba_t[i] > p.traba_ms) { nucleo_falla(s, F_TRABA); return; }
      } else s.traba_t[i] = 0;
      if (cerrado && fabsf(s.w_med[i]) > 1.5f * p.wrueda_max) { nucleo_falla(s, F_SOBREVEL); return; }
    }
  } else {
    for (uint8_t i = 0; i < 4; i++) s.traba_t[i] = 0;
  }
}
