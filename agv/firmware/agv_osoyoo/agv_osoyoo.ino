// agv_osoyoo.ino — Firmware del AGV OSOYOO M2.0 mecanum (Arduino Mega2560).
// Trabajo integrador Robotica II, FCAL-UNER, 2026.
//
// Lazo fijo de 10 ms (micros): lectura de encoders x4, odometria, PI de ruedas, protecciones;
// lazo de pose cada 20 ms. La PC envia ordenes por USB (115200 8N1) y recibe telemetria.
// Seguridad: watchdog de comunicacion (sin mensajes > wd_ms con el robot armado -> FALLA),
// watchdog de hardware (250 ms), rueda trabada, sobrevelocidad, bateria baja (si se habilita),
// periodo excedido. De FALLA solo se sale con CLR y luego ARM.
#include <avr/wdt.h>
#include "config.h"
#include "hal.h"
#include "nucleo.h"
#include "protocolo.h"
#include "wifi.h"

Parametros P;
Estado S;

static uint32_t proximo_us, ultimo_us;
static uint8_t  div_pose = 0, div_bat = 0;
static uint16_t vbat_mV = 0;

static bool en_movimiento() {
  return S.modo == M_PWM || S.modo == M_PASO || S.modo == M_VEL || S.modo == M_IRA;
}

void setup() {
  const uint8_t mcusr = MCUSR;
  MCUSR = 0;
  wdt_disable();
  Serial.begin(SERIAL_BAUD);
  hal_init();
  hal_motors_off();
  nucleo_parametros_defecto(P);
  prot_cargar_eeprom(P);                         // si hay parametros guardados validos, se usan
  nucleo_reset(S);
  if (mcusr & _BV(WDRF)) { S.falla = F_RESET; S.modo = M_FALLA; }   // reinicio por cuelgue: queda en FALLA
  vbat_mV = hal_battery_mV();
  prot_init(mcusr);
  wifi_init();                                   // ESP8266 del shield por Serial1: se configura sin bloquear
  wdt_enable(WDT_HW);
  ultimo_us = micros();
  proximo_us = ultimo_us + TS_US;
}

void loop() {
  wdt_reset();
  while (Serial.available() > 0) prot_rx((char)Serial.read(), millis(), 0);
  wifi_poll(millis());                           // configura el ESP o pasa los bytes UDP a prot_rx(…, 1)

  const uint32_t ahora = micros();
  if ((int32_t)(ahora - proximo_us) >= 0) {
    const uint32_t periodo = ahora - ultimo_us;
    ultimo_us = ahora;
    proximo_us += TS_US;
    if ((int32_t)(ahora - proximo_us) >= 0) proximo_us = ahora + TS_US;   // se perdio un periodo: resincronizar

    const uint32_t t0 = micros();
    const uint32_t ms = millis();
    int32_t c[4];
    hal_counts(c);
    if (++div_bat >= 10) { div_bat = 0; vbat_mV = hal_battery_mV(); }  // 10 Hz alcanza para la bateria

    // watchdog de comunicacion: en un GOTO (o quieto y armado) un corte de hasta WD_PAUSA_MAX_MS no es falla,
    // solo pausa (el GOTO congela la trayectoria y frena); con ordenes manuales (PWM, PASO, VEL) es falla enseguida
    const uint8_t pausa_antes = S.pausa_com;
    S.pausa_com = 0;
    if (S.modo != M_DESARMADO && S.modo != M_FALLA && P.wd_ms) {
      const uint32_t sin_msg = ms - prot_ultimo_msg_ms();
      if (sin_msg > P.wd_ms) {
        if (sin_msg > WD_PAUSA_MAX_MS || (S.modo != M_IRA && S.modo != M_QUIETO)) nucleo_falla(S, F_COM);
        else S.pausa_com = 1;
      }
    }
    if (S.pausa_com != pausa_antes) prot_evento(S.pausa_com ? "EVT,PAUSA_COM,1" : "EVT,PAUSA_COM,0");
    if (hal_estop_open() && S.modo != M_FALLA) nucleo_falla(S, F_PARADA);
    if (en_movimiento() && periodo > 3 * TS_US) nucleo_falla(S, F_PERIODO);

    const bool tick_pose = (++div_pose >= POSE_DIV);
    if (tick_pose) div_pose = 0;
    nucleo_paso(S, P, c, (float)periodo * 1e-6f, vbat_mV, ms, tick_pose);

    if (en_movimiento()) hal_motor_write(S.u, P.signo_motor);
    else hal_motors_off();

    prot_periodo(ms, periodo, micros() - t0);
  }
  prot_flush();
}
