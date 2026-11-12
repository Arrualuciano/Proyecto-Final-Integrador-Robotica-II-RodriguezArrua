// wifi.cpp — Configuracion no bloqueante del ESP8266 (firmware AT) y enlace UDP transparente por Serial1.
// Secuencia (modo estacion): +++ (sale de un transparente previo), AT, ATE0, CWMODE_CUR=1, CWJAP_CUR (red),
// CIFSR (IP propia), SLEEP=0 (sin modem-sleep: menos latencia), CIPCLOSE, CIPMUX=0,
// CIPSTART UDP con destino fijo (unico modo que acepta el transparente), CIPMODE=1, CIPSEND ('>').
// Si el ESP no contesta "AT" se prueba otra velocidad. Nunca se abandona: reintenta cada 3 s (10 s despues de
// 15 intentos) y el USB sigue funcionando. Esquema basado en el firmware de T. Mendelevich y M. Baldoni.
#include <Arduino.h>
#include <string.h>
#include "wifi.h"
#include "wifi_config.h"
#include "protocolo.h"

#if WIFI_HABILITADO
#define STR_(x) #x
#define STR(x) STR_(x)
static const char *const AT_CMD[] = {
  "+++", "AT", "ATE0", "AT+CWMODE_CUR=1",
  "AT+CWJAP_CUR=\"" WIFI_SSID "\",\"" WIFI_CLAVE "\"",
  "AT+CIFSR", "AT+SLEEP=0", "AT+CIPCLOSE", "AT+CIPMUX=0",
  "AT+CIPSTART=\"UDP\",\"" WIFI_PC_IP "\"," STR(WIFI_PUERTO_PC) "," STR(WIFI_PUERTO_ROBOT) ",0",
  "AT+CIPMODE=1", "AT+CIPSEND"};
static const uint8_t AT_N = sizeof(AT_CMD) / sizeof(AT_CMD[0]);
static const uint8_t PASO_RED = 4, PASO_IP = 5;
static const uint32_t BAUDS[] = {115200, 9600, 57600, 38400, 74880};
static const uint8_t NB = sizeof(BAUDS) / sizeof(BAUDS[0]);

static struct {
  uint8_t paso, ib, intentos, n;
  bool enviado, listo;
  uint32_t t0, t_aviso;
  char rx[140];
  char ip[16];
} w;

static void reportar(bool ok, const char *det) {
  char b[140];
  snprintf(b, sizeof(b), "WIFI,%s,%s", ok ? "ok" : "error", det);
  prot_evento(b);
}

static void reiniciar(const char *motivo, uint32_t ahora) {
  char d[100];
  snprintf(d, sizeof(d), "%s (paso %u, %lu baud)", motivo, w.paso, (unsigned long)BAUDS[w.ib]);
  reportar(false, d);
  if (w.paso <= 1) { w.ib = (uint8_t)((w.ib + 1) % NB); Serial1.end(); Serial1.begin(BAUDS[w.ib]); }
  w.intentos++;
  w.paso = 0; w.enviado = false; w.n = 0;
  w.t0 = ahora + (w.intentos > 15 ? 10000UL : 3000UL);
}

static void leer_ip() {                                   // +CIFSR:STAIP,"172.20.10.9"
  const char *p = strstr(w.rx, "STAIP,\"");
  if (!p) return;
  p += 7; uint8_t k = 0;
  while (*p && *p != '"' && k < sizeof(w.ip) - 1) w.ip[k++] = *p++;
  w.ip[k] = 0;
}

void wifi_init() {
  memset(&w, 0, sizeof(w));
  strcpy(w.ip, "0.0.0.0");
  Serial1.begin(BAUDS[0]);
  w.t0 = millis() + 2500;                                 // tiempo para que el ESP termine de arrancar
}

bool wifi_listo() { return w.listo; }
const char *wifi_ip() { return w.ip; }

void wifi_poll(uint32_t ahora) {
  if (w.listo) {
    while (Serial1.available() > 0) prot_rx((char)Serial1.read(), ahora, 1);
    // aviso cada 1 s mientras ninguna PC hable por WiFi: la consola aprende de aca la IP del robot
    if (prot_enlace() != 1 && (uint32_t)(ahora - w.t_aviso) >= 1000) {
      w.t_aviso = ahora;
      char c[48], b[56]; uint8_t x = 0;
      snprintf(c, sizeof(c), "HOLA,WIFI,%s,agv", w.ip);
      for (const char *p = c; *p; p++) x ^= (uint8_t)*p;
      snprintf(b, sizeof(b), "$%s*%02X\n", c, x);
      if (Serial1.availableForWrite() >= (int)strlen(b)) Serial1.print(b);
    }
    return;
  }
  while (Serial1.available() > 0) {                       // respuestas del ESP (ventana deslizante)
    char ch = (char)Serial1.read();
    if (ch == 0) continue;
    if (w.n >= sizeof(w.rx) - 1) { memmove(w.rx, w.rx + 60, w.n - 60); w.n -= 60; }
    w.rx[w.n++] = ch; w.rx[w.n] = 0;
  }
  if ((int32_t)(ahora - w.t0) < 0) return;                // esperando (arranque del ESP o reintento)
  if (!w.enviado) {
    w.n = 0; w.rx[0] = 0;
    if (w.paso == 0) Serial1.print("+++");               // sin CR/LF
    else { if (w.paso == 1) Serial1.print("\r\n"); Serial1.print(AT_CMD[w.paso]); Serial1.print("\r\n"); }
    w.enviado = true; w.t0 = ahora;
    return;
  }
  const uint32_t dt = ahora - w.t0;
  bool ok = false, error = false;
  if (w.paso == 0) ok = dt > 1200;                         // "+++" no tiene respuesta
  else if (w.paso == AT_N - 1) ok = strchr(w.rx, '>') != nullptr;
  else if (strstr(AT_CMD[w.paso], "CIPCLOSE")) ok = strstr(w.rx, "OK") || strstr(w.rx, "ERROR") || dt > 1500;
  else {
    ok = strstr(w.rx, "OK") || strstr(w.rx, "ALREADY") || strstr(w.rx, "no change");
    error = !ok && w.paso > 1 && (strstr(w.rx, "ERROR") || strstr(w.rx, "FAIL"));
  }
  const uint32_t tmax = (w.paso == PASO_RED) ? 20000UL : 3000UL;
  if (ok) {
    if (w.paso == PASO_IP) leer_ip();
    if (++w.paso >= AT_N) {
      w.listo = true;
      char d[100];
      snprintf(d, sizeof(d), "red %s IP %s -> PC %s:%u", WIFI_SSID, w.ip, WIFI_PC_IP, (unsigned)WIFI_PUERTO_PC);
      reportar(true, d);
    }
    w.enviado = false; w.t0 = ahora + 100;
  } else if (error) {
    reiniciar(w.paso == PASO_RED ? "no conecta a la red (SSID/clave?)" : "error", ahora);
  } else if (dt > tmax) {
    reiniciar("sin respuesta", ahora);
  }
}
#else
void wifi_init() {}
void wifi_poll(uint32_t) {}
bool wifi_listo() { return false; }
const char *wifi_ip() { return "-"; }
#endif
