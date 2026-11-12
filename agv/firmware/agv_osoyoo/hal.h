// hal.h — Capa de hardware: PWM de motores, encoders (puerto K, x4), bateria, parada de emergencia.
#pragma once
#include <stdint.h>

void     hal_init();
void     hal_motor_write(const int16_t u[4], uint8_t sign_mask);  // u en [-255, 255]; bit i invierte la rueda i
void     hal_motors_off();
void     hal_counts(int32_t out[4]);        // copia atomica de los contadores x4 (sin signo de calibracion)
void     hal_counts_reset();
uint32_t hal_hall_errors();                 // transiciones invalidas (A y B cambiaron juntos): ruido o perdida
uint8_t  hal_hall_raw();                    // lectura cruda de PINK (para revisar cableado y niveles)
uint16_t hal_battery_mV();
bool     hal_estop_open();                  // true si la parada de emergencia esta abierta (o no instalada: false)
