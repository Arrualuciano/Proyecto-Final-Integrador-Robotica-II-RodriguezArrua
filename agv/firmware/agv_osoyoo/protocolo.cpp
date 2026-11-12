// protocolo.cpp — Comandos, respuestas, telemetria y EEPROM.
#include <Arduino.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "hal.h"
#include "protocolo.h"

// ------------------------------------------------------------------ cola TX no bloqueante
// Serial.write bloquea cuando su buffer de 64 bytes se llena (~15 ms para una telemetria larga).
// Se encola aca y se vacia con availableForWrite() sin esperar nunca.
static char     txq[TX_BUF];
static uint16_t tx_h = 0, tx_t = 0;
static uint16_t tx_perdidas = 0;

static uint16_t tx_libre() { return (uint16_t)((tx_t + TX_BUF - tx_h - 1) % TX_BUF); }
static void tx_linea(const char *s) {
  const uint16_t n = (uint16_t)strlen(s);
  if (n + 1 > tx_libre()) { tx_perdidas++; return; }      // se descarta la linea completa, nunca a medias
  for (uint16_t i = 0; i < n; i++) { txq[tx_h] = s[i]; tx_h = (uint16_t)((tx_h + 1) % TX_BUF); }
  txq[tx_h] = '\n'; tx_h = (uint16_t)((tx_h + 1) % TX_BUF);
}
// Las respuestas y la telemetria salen por el enlace del ultimo comando recibido (USB o WiFi).
static uint8_t enlace_activo = 0;
uint8_t prot_enlace() { return enlace_activo; }
void prot_flush() {
  HardwareSerial &s = enlace_activo ? Serial1 : Serial;
  int n = s.availableForWrite();
  while (n-- > 0 && tx_t != tx_h) { s.write(txq[tx_t]); tx_t = (uint16_t)((tx_t + 1) % TX_BUF); }
}
// Agrega '$' y '*HH' a un cuerpo "CMD,..."
static void enviar(const char *cuerpo) {
  char buf[220];
  uint8_t x = 0;
  for (const char *p = cuerpo; *p; p++) x ^= (uint8_t)*p;
  snprintf(buf, sizeof(buf), "$%s*%02X", cuerpo, x);
  tx_linea(buf);
}
void prot_evento(const char *cuerpo) { enviar(cuerpo); }

// ------------------------------------------------------------------ EEPROM
struct Imagen { uint16_t magia; uint8_t version; Parametros p; uint8_t suma; };
static const uint16_t MAGIA = 0xA6E1;
static uint8_t suma_bytes(const uint8_t *b, uint16_t n) { uint8_t s = 0; while (n--) s ^= *b++; return s; }
bool prot_cargar_eeprom(Parametros &p) {
  Imagen im; EEPROM.get(0, im);
  if (im.magia != MAGIA || im.version != 1) return false;
  if (suma_bytes((const uint8_t *)&im.p, sizeof(Parametros)) != im.suma) return false;
  p = im.p; return true;
}
static void guardar_eeprom(const Parametros &p) {
  Imagen im; im.magia = MAGIA; im.version = 1; im.p = p;
  im.suma = suma_bytes((const uint8_t *)&im.p, sizeof(Parametros));
  // Cada byte de EEPROM tarda ~3,4 ms: la imagen completa (~130 B) lleva ~0,45 s, mas que el watchdog de
  // hardware (250 ms). En la 0.1.1 EEPROM.put() reiniciaba la placa a mitad de la escritura. Se escribe byte
  // a byte (update: solo los que cambian) alimentando el watchdog. Solo se permite con el robot detenido.
  const uint8_t *b = (const uint8_t *)&im;
  for (uint16_t i = 0; i < sizeof(Imagen); i++) { EEPROM.update(i, b[i]); wdt_reset(); }
}

// ------------------------------------------------------------------ estado del protocolo
static char     rx[2][RX_LINE];                 // un buffer de linea por enlace (USB, WiFi)
static uint8_t  rx_n[2] = {0, 0};
static uint32_t ultimo_msg_ms = 0;
static uint8_t  tel_hz = 20, tel_div = 0;
static uint32_t peor_periodo = 0, peor_ejec = 0;
static uint8_t  falla_ant = 0;

uint32_t prot_ultimo_msg_ms() { return ultimo_msg_ms; }

static const char *NOMBRE_MODO[] = {"DESARMADO", "QUIETO", "PWM", "PASO", "VEL", "IR_A", "FALLA"};

static long mil(float v) { return (long)(v * 1000.0f + (v >= 0 ? 0.5f : -0.5f)); }   // a milesimas

static void ack(const char *cmd, const char *extra = nullptr) {
  char b[120];
  if (extra) snprintf(b, sizeof(b), "ACK,%s,%s", cmd, extra); else snprintf(b, sizeof(b), "ACK,%s", cmd);
  enviar(b);
}
static void nak(const char *cmd, uint8_t codigo) {
  char b[60]; snprintf(b, sizeof(b), "NAK,%s,%u", cmd, codigo); enviar(b);
}

static void f2s(float v, char *out) { dtostrf(v, 1, 4, out); }

static void volcar_parametros() {
  char b[200], a[16], c[16], d[16], e[16], f[16];
  f2s(P.r, a); f2s(P.lx, c); f2s(P.ly, d); f2s(P.sy, e);
  snprintf(b, sizeof(b), "PAR,GEO,%s,%s,%s,%s", a, c, d, e); enviar(b);
  f2s(P.cpr[0], a); f2s(P.cpr[1], c); f2s(P.cpr[2], d); f2s(P.cpr[3], e);
  snprintf(b, sizeof(b), "PAR,CPR,%s,%s,%s,%s", a, c, d, e); enviar(b);
  snprintf(b, sizeof(b), "PAR,SIGN,%u,%u", P.signo_motor, P.signo_enc); enviar(b);
  for (uint8_t i = 0; i < 4; i++) {
    f2s(P.K[i], a); f2s(P.udz[i], c);
    snprintf(b, sizeof(b), "PAR,MDL,%u,%s,%s", i, a, c); enviar(b);
  }
  f2s(P.kp, a); f2s(P.ki, c); f2s(P.alfa_vel, d);
  snprintf(b, sizeof(b), "PAR,GAIN,%s,%s,FIL,%s", a, c, d); enviar(b);
  f2s(P.kx, a); f2s(P.ky, c); f2s(P.kphi, d);
  snprintf(b, sizeof(b), "PAR,KPOSE,%s,%s,%s", a, c, d); enviar(b);
  f2s(P.vmax, a); f2s(P.amax, c); f2s(P.wmax, d); f2s(P.alfamax, e); f2s(P.wrueda_max, f);
  snprintf(b, sizeof(b), "PAR,LIM,%s,%s,%s,%s,%s", a, c, d, e, f); enviar(b);
  f2s(P.eta_x, a); f2s(P.eta_y, c); f2s(P.eta_w, d);
  snprintf(b, sizeof(b), "PAR,CAL,%s,%s,%s", a, c, d); enviar(b);
  snprintf(b, sizeof(b), "PAR,SAFE,%u,%u,%u,%u,%u", P.ttl_ms, P.wd_ms, P.traba_ms, P.bat_check, P.bat_min_mV); enviar(b);
  snprintf(b, sizeof(b), "PAR,TEL,%u", tel_hz); enviar(b);
}

void prot_init(uint8_t mcusr) {
  char b[80];
  snprintf(b, sizeof(b), "HOLA,%s,MCUSR,%u,TS_US,%lu", FW_VERSION, mcusr, (unsigned long)TS_US);
  enviar(b);
}

// ------------------------------------------------------------------ interprete de comandos
static float argf(char **t, uint8_t n, uint8_t i, float def) { return i < n ? (float)atof(t[i]) : def; }
static long  argl(char **t, uint8_t n, uint8_t i, long def)  { return i < n ? atol(t[i]) : def; }

static void procesar(char *linea, uint32_t ahora) {
  // suma de verificacion opcional
  char *inicio = linea;
  if (*inicio == '$') inicio++;
  char *ast = strchr(inicio, '*');
  if (ast) {
    *ast = 0;
    uint8_t x = 0; for (char *p = inicio; *p; p++) x ^= (uint8_t)*p;
    if ((uint8_t)strtol(ast + 1, nullptr, 16) != x) { nak("CHK", 1); return; }
  }
  char *tok[12]; uint8_t n = 0;
  for (char *p = strtok(inicio, ", \t\r"); p && n < 12; p = strtok(nullptr, ", \t\r")) tok[n++] = p;
  if (!n) return;
  for (char *p = tok[0]; *p; p++) *p = (char)toupper(*p);
  const char *cmd = tok[0];
  ultimo_msg_ms = ahora;                        // cualquier linea valida alimenta el watchdog de comunicacion
  char b[120];
  uint8_t r;

  if (!strcmp(cmd, "PING")) {
    snprintf(b, sizeof(b), "PONG,%s,%lu,%s,%u", FW_VERSION, (unsigned long)ahora, NOMBRE_MODO[S.modo], S.falla); enviar(b);
  } else if (!strcmp(cmd, "ARM")) {
    r = nucleo_armar(S); if (r) nak(cmd, r); else ack(cmd);
  } else if (!strcmp(cmd, "DIS")) {
    nucleo_desarmar(S); hal_motors_off(); ack(cmd);
  } else if (!strcmp(cmd, "STOP")) {
    nucleo_parar(S); ack(cmd);
  } else if (!strcmp(cmd, "CLR")) {
    r = nucleo_limpiar(S); if (r) nak(cmd, r); else ack(cmd);
  } else if (!strcmp(cmd, "PWM")) {                       // PWM,u_FL,u_FR,u_RL,u_RR  (-255..255, lazo abierto)
    int16_t u[4]; for (uint8_t i = 0; i < 4; i++) u[i] = (int16_t)argl(tok, n, i + 1, 0);
    r = nucleo_pwm(S, u, ahora); if (r) nak(cmd, r); else ack(cmd);
  } else if (!strcmp(cmd, "STEP")) {                      // STEP,rueda(0..3, 4=todas),u,ms
    r = nucleo_paso_escalon(S, (uint8_t)argl(tok, n, 1, 4), (int16_t)argl(tok, n, 2, 0), (uint16_t)argl(tok, n, 3, 1000), ahora);
    if (r) nak(cmd, r); else ack(cmd);
  } else if (!strcmp(cmd, "VEL")) {                       // VEL,vx,vy,wz  (m/s, m/s, rad/s; marco del cuerpo)
    r = nucleo_vel(S, P, argf(tok, n, 1, 0), argf(tok, n, 2, 0), argf(tok, n, 3, 0), ahora);
    if (r) nak(cmd, r); else ack(cmd);
  } else if (!strcmp(cmd, "GOTO")) {                      // GOTO,x,y,phi,T,modo  ({G}; modo 0 abierto, 1 ruedas, 2 pose)
    float Te = 0;
    r = nucleo_ir_a(S, P, argf(tok, n, 1, S.x), argf(tok, n, 2, S.y), argf(tok, n, 3, S.phi), argf(tok, n, 4, 0),
                    (uint8_t)argl(tok, n, 5, 2), &Te);
    if (r) nak(cmd, r); else { char t[16]; dtostrf(Te, 1, 3, t); ack(cmd, t); }
  } else if (!strcmp(cmd, "POSE")) {
    nucleo_set_pose(S, argf(tok, n, 1, 0), argf(tok, n, 2, 0), argf(tok, n, 3, 0)); ack(cmd);
  } else if (!strcmp(cmd, "ZERO")) {
    if (S.modo != M_QUIETO && S.modo != M_DESARMADO) { nak(cmd, 3); return; }
    hal_counts_reset(); S.cnt_init = 0; nucleo_set_pose(S, 0, 0, 0); ack(cmd);
  } else if (!strcmp(cmd, "GAIN")) {
    P.kp = argf(tok, n, 1, P.kp); P.ki = argf(tok, n, 2, P.ki); ack(cmd);
  } else if (!strcmp(cmd, "FIL")) {
    P.alfa_vel = constrain(argf(tok, n, 1, P.alfa_vel), 0.05f, 1.0f); ack(cmd);
  } else if (!strcmp(cmd, "KPOSE")) {
    P.kx = argf(tok, n, 1, P.kx); P.ky = argf(tok, n, 2, P.ky); P.kphi = argf(tok, n, 3, P.kphi); ack(cmd);
  } else if (!strcmp(cmd, "LIM")) {
    P.vmax = constrain(argf(tok, n, 1, P.vmax), 0.01f, 0.5f);
    P.amax = constrain(argf(tok, n, 2, P.amax), 0.05f, 2.0f);
    P.wmax = constrain(argf(tok, n, 3, P.wmax), 0.1f, 4.0f);
    P.alfamax = constrain(argf(tok, n, 4, P.alfamax), 0.1f, 10.0f);
    P.wrueda_max = constrain(argf(tok, n, 5, P.wrueda_max), 1.0f, 15.0f);
    ack(cmd);
  } else if (!strcmp(cmd, "GEO")) {
    if (S.modo != M_QUIETO && S.modo != M_DESARMADO) { nak(cmd, 3); return; }
    P.r = constrain(argf(tok, n, 1, P.r), 0.02f, 0.06f);
    P.lx = constrain(argf(tok, n, 2, P.lx), 0.03f, 0.20f);
    P.ly = constrain(argf(tok, n, 3, P.ly), 0.03f, 0.20f);
    P.sy = argf(tok, n, 4, P.sy) < 0 ? -1.0f : 1.0f;
    ack(cmd);
  } else if (!strcmp(cmd, "CPR")) {
    for (uint8_t i = 0; i < 4; i++) P.cpr[i] = constrain(argf(tok, n, i + 1, P.cpr[i]), 50.0f, 100000.0f);
    ack(cmd);
  } else if (!strcmp(cmd, "SIGN")) {                      // SIGN,mascara_motor,mascara_encoder (bit 0 = FL ... bit 3 = RR)
    if (S.modo != M_QUIETO && S.modo != M_DESARMADO) { nak(cmd, 3); return; }
    P.signo_motor = (uint8_t)(argl(tok, n, 1, P.signo_motor) & 0x0F);
    P.signo_enc = (uint8_t)(argl(tok, n, 2, P.signo_enc) & 0x0F);
    ack(cmd);
  } else if (!strcmp(cmd, "MDL")) {                       // MDL,i(0..3, 4=todas),K,udz
    const long i = argl(tok, n, 1, 4);
    for (uint8_t k = 0; k < 4; k++) if (i == 4 || i == k) {
      P.K[k] = constrain(argf(tok, n, 2, P.K[k]), 0.005f, 1.0f);
      P.udz[k] = constrain(argf(tok, n, 3, P.udz[k]), 0.0f, 200.0f);
    }
    ack(cmd);
  } else if (!strcmp(cmd, "CAL")) {
    P.eta_x = constrain(argf(tok, n, 1, P.eta_x), 0.5f, 1.5f);
    P.eta_y = constrain(argf(tok, n, 2, P.eta_y), 0.5f, 1.5f);
    P.eta_w = constrain(argf(tok, n, 3, P.eta_w), 0.5f, 1.5f);
    ack(cmd);
  } else if (!strcmp(cmd, "SAFE")) {                      // SAFE,ttl_ms,wd_ms(0 = sin watchdog: solo pruebas manuales),traba_ms,bat_check,bat_min_mV
    P.ttl_ms = (uint16_t)constrain(argl(tok, n, 1, P.ttl_ms), 100, 5000);
    const long wd = argl(tok, n, 2, P.wd_ms);
    P.wd_ms = (uint16_t)(wd <= 0 ? 0 : constrain(wd, 100, 60000));
    P.traba_ms = (uint16_t)constrain(argl(tok, n, 3, P.traba_ms), 100, 5000);
    P.bat_check = (uint8_t)(argl(tok, n, 4, P.bat_check) ? 1 : 0);
    P.bat_min_mV = (uint16_t)constrain(argl(tok, n, 5, P.bat_min_mV), 5000, 8000);
    ack(cmd);
  } else if (!strcmp(cmd, "TEL")) {
    tel_hz = (uint8_t)constrain(argl(tok, n, 1, tel_hz), 0, TEL_HZ_MAX); tel_div = 0; ack(cmd);
  } else if (!strcmp(cmd, "HALL")) {                      // lectura cruda para revisar cableado: A/B por rueda
    const uint8_t k = hal_hall_raw();
    int32_t c[4]; hal_counts(c);
    snprintf(b, sizeof(b), "HALL,%u%u,%u%u,%u%u,%u%u,%ld,%ld,%ld,%ld,%lu",
             (k >> HALL_A_BIT[0]) & 1, (k >> HALL_B_BIT[0]) & 1, (k >> HALL_A_BIT[1]) & 1, (k >> HALL_B_BIT[1]) & 1,
             (k >> HALL_A_BIT[2]) & 1, (k >> HALL_B_BIT[2]) & 1, (k >> HALL_A_BIT[3]) & 1, (k >> HALL_B_BIT[3]) & 1,
             (long)c[0], (long)c[1], (long)c[2], (long)c[3], (unsigned long)hal_hall_errors());
    enviar(b);
  } else if (!strcmp(cmd, "GET")) {
    volcar_parametros();
  } else if (!strcmp(cmd, "SAVE")) {
    if (S.modo != M_QUIETO && S.modo != M_DESARMADO) { nak(cmd, 3); return; }
    guardar_eeprom(P); ack(cmd);
  } else if (!strcmp(cmd, "LOAD")) {
    if (S.modo != M_QUIETO && S.modo != M_DESARMADO) { nak(cmd, 3); return; }
    if (prot_cargar_eeprom(P)) ack(cmd); else nak(cmd, 5);
  } else if (!strcmp(cmd, "DEF")) {
    if (S.modo != M_QUIETO && S.modo != M_DESARMADO) { nak(cmd, 3); return; }
    nucleo_parametros_defecto(P); ack(cmd);
  } else {
    nak(cmd, 9);
  }
}

void prot_rx(char c, uint32_t ahora_ms, uint8_t enlace) {
  enlace = enlace ? 1 : 0;
  char *b = rx[enlace]; uint8_t &n = rx_n[enlace];
  if (c == '\n') {
    b[n] = 0;
    if (n) { enlace_activo = enlace; procesar(b, ahora_ms); }   // la respuesta sale por donde llego la orden
    n = 0;
  } else if (n < RX_LINE - 1) {
    b[n++] = c;
  } else {
    n = 0;                                       // linea demasiado larga: se descarta
  }
}

// ------------------------------------------------------------------ telemetria y eventos
// $TEL: t_ms, modo, falla, x, y, phi, xr, yr, phir, vx, vy, wz, wr[4], wm[4], u[4], cuentas[4],
//       vbat_mV, ejec_us, peor_periodo_us, errores_hall, lineas_perdidas
// Unidades: mm, mrad, mm/s, mrad/s, PWM (-255..255), cuentas x4 crudas.
void prot_periodo(uint32_t ahora_ms, uint32_t periodo_us, uint32_t ejec_us) {
  if (periodo_us > peor_periodo) peor_periodo = periodo_us;
  if (ejec_us > peor_ejec) peor_ejec = ejec_us;

  if (S.falla != falla_ant) {
    char b[48]; snprintf(b, sizeof(b), "EVT,FALLA,%u", S.falla); enviar(b);
    falla_ant = S.falla;
  }
  if (S.tr_fin) {
    char b[80];
    snprintf(b, sizeof(b), "EVT,FIN,%ld,%ld,%ld", mil(S.ref[0] - S.x), mil(S.ref[1] - S.y), mil(envolver(S.ref[2] - S.phi)));
    enviar(b); S.tr_fin = 0;
  }
  if (!tel_hz) return;
  if (++tel_div < (uint8_t)(100u / tel_hz)) return;
  tel_div = 0;
  int32_t c[4]; hal_counts(c);
  char b[220];
  snprintf(b, sizeof(b),
           "TEL,%lu,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%u,%lu,%lu,%lu,%u",
           (unsigned long)ahora_ms, S.modo, S.falla,
           mil(S.x), mil(S.y), mil(S.phi), mil(S.ref[0]), mil(S.ref[1]), mil(S.ref[2]),
           mil(S.vb[0]), mil(S.vb[1]), mil(S.vb[2]),
           mil(S.w_ref[0]), mil(S.w_ref[1]), mil(S.w_ref[2]), mil(S.w_ref[3]),
           mil(S.w_med[0]), mil(S.w_med[1]), mil(S.w_med[2]), mil(S.w_med[3]),
           S.u[0], S.u[1], S.u[2], S.u[3],
           (long)c[0], (long)c[1], (long)c[2], (long)c[3],
           (unsigned)S.vbat_filt, (unsigned long)peor_ejec, (unsigned long)peor_periodo,
           (unsigned long)hal_hall_errors(), tx_perdidas);
  enviar(b);
  peor_periodo = 0; peor_ejec = 0;
}
