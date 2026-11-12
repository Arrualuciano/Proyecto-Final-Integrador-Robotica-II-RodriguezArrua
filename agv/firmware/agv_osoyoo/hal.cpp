// hal.cpp — Implementacion de la capa de hardware para Arduino Mega2560.
#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "config.h"
#include "hal.h"

// ---------------------------------------------------------------- encoders
// Estado de cada canal = (A << 1) | B. Indice de tabla = (estado_anterior << 2) | estado_actual.
// Secuencia 0 -> 1 -> 3 -> 2 -> 0 suma +1; la inversa resta. Los indices 3, 6, 9, 12 son
// transiciones invalidas (cambian A y B a la vez): no suman y se cuentan como error.
static const int8_t QDEC[16] = { 0, +1, -1,  0,
                                -1,  0,  0, +1,
                                +1,  0,  0, -1,
                                 0, -1, +1,  0 };
static volatile int32_t  enc_cnt[4];
static volatile uint8_t  enc_prev[4];
static volatile uint32_t enc_err;

static inline uint8_t ab_state(uint8_t k, uint8_t i) {
  return (uint8_t)((((k >> HALL_A_BIT[i]) & 1u) << 1) | ((k >> HALL_B_BIT[i]) & 1u));
}

ISR(PCINT2_vect) {
  const uint8_t k = PINK;                         // una sola lectura para las 4 ruedas
  for (uint8_t i = 0; i < 4; i++) {
    const uint8_t s = ab_state(k, i);
    const uint8_t idx = (uint8_t)((enc_prev[i] << 2) | s);
    enc_cnt[i] += QDEC[idx];
    if (idx == 3 || idx == 6 || idx == 9 || idx == 12) enc_err++;
    enc_prev[i] = s;
  }
}

// ---------------------------------------------------------------- inicializacion
void hal_init() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(PIN_IN1[i], OUTPUT); digitalWrite(PIN_IN1[i], LOW);
    pinMode(PIN_IN2[i], OUTPUT); digitalWrite(PIN_IN2[i], LOW);
    pinMode(PIN_PWM[i], OUTPUT); analogWrite(PIN_PWM[i], 0);
  }
#if PWM_PRESCALER_8
  // Arduino deja Timer1 y Timer2 en PWM de fase correcta 8 bits: f = 16 MHz / (N * 510).
  // N = 8 -> 3,92 kHz (menos zumbido y mejor linealidad que 490 Hz). Timer0 (millis/micros) no se toca.
  TCCR1B = (uint8_t)((TCCR1B & 0xF8) | 0x02);
  TCCR2B = (uint8_t)((TCCR2B & 0xF8) | 0x02);
#endif
  // Puerto K como entrada, con o sin pull-ups
  DDRK = 0x00;
  PORTK = HALL_PULLUPS ? 0xFF : 0x00;
  delayMicroseconds(50);
  const uint8_t k = PINK;
  for (uint8_t i = 0; i < 4; i++) { enc_prev[i] = ab_state(k, i); enc_cnt[i] = 0; }
  enc_err = 0;
  PCMSK2 = 0xFF;                                  // los 8 canales (A y B) generan interrupcion -> x4
  PCIFR = _BV(PCIF2);
  PCICR |= _BV(PCIE2);

  pinMode(BAT_PIN, INPUT);
#if ESTOP_ENABLED
  pinMode(ESTOP_PIN, INPUT_PULLUP);
#endif
}

// ---------------------------------------------------------------- motores
void hal_motor_write(const int16_t u[4], uint8_t sign_mask) {
  for (uint8_t i = 0; i < 4; i++) {
    int16_t v = u[i];
    if (v > 255) v = 255; else if (v < -255) v = -255;
    if (sign_mask & (1u << i)) v = -v;
    if (v > 0)      { digitalWrite(PIN_IN2[i], LOW);  digitalWrite(PIN_IN1[i], HIGH); }
    else if (v < 0) { digitalWrite(PIN_IN1[i], LOW);  digitalWrite(PIN_IN2[i], HIGH); }
    else            { digitalWrite(PIN_IN1[i], LOW);  digitalWrite(PIN_IN2[i], LOW);  }   // rueda libre
    analogWrite(PIN_PWM[i], (uint8_t)(v < 0 ? -v : v));
  }
}

void hal_motors_off() {
  for (uint8_t i = 0; i < 4; i++) {
    analogWrite(PIN_PWM[i], 0);
    digitalWrite(PIN_IN1[i], LOW); digitalWrite(PIN_IN2[i], LOW);
  }
}

// ---------------------------------------------------------------- lecturas
void hal_counts(int32_t out[4]) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i = 0; i < 4; i++) out[i] = enc_cnt[i];
  }
}
void hal_counts_reset() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i = 0; i < 4; i++) enc_cnt[i] = 0;
    enc_err = 0;
  }
}
uint32_t hal_hall_errors() {
  uint32_t e;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { e = enc_err; }
  return e;
}
uint8_t hal_hall_raw() { return PINK; }

uint16_t hal_battery_mV() {
  const uint16_t raw = (uint16_t)analogRead(BAT_PIN);
  return (uint16_t)((float)raw * (VREF_MV / 1023.0f) * BAT_FACTOR);
}

bool hal_estop_open() {
#if ESTOP_ENABLED
  return digitalRead(ESTOP_PIN) == HIGH;          // NC a GND: abierto = HIGH por el pull-up
#else
  return false;
#endif
}
