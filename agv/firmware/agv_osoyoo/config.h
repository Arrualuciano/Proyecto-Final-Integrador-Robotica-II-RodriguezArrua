// config.h — Configuracion fija del AGV OSOYOO M2.0 (Arduino Mega2560)
// Fuente de pines: manual OSOYOO M2.0 p.10 y parametros/parametros.json (bloque agv.pines).
// Orden logico de ruedas en TODO el firmware: 0 = FL, 1 = FR, 2 = RL, 3 = RR
// (delantera izquierda, delantera derecha, trasera izquierda, trasera derecha, mirando hacia +x).
#pragma once
#include <stdint.h>

#define FW_VERSION      "agv-0.2.3"     // 0.1.1: wmax y alfamax por defecto (en 0.1.0 valian 0)
                                        // 0.1.2: SAVE ya no dispara el watchdog de hardware
                                        // 0.2.0: enlace WiFi (ESP8266 por Serial1, UDP transparente)
                                        // 0.2.1: valores por defecto = parametros medidos en el kit
                                        // 0.2.2: pausa ante cortes breves de comunicacion en GOTO
                                        // 0.2.3: calibracion eta del piso como valor por defecto (el Loader borra la EEPROM)
#define WD_PAUSA_MAX_MS 3000UL    // corte de comunicacion tolerado como pausa (GOTO / quieto); mas largo -> F_COM
#define SERIAL_BAUD     115200UL

// Tiempos
#define TS_US           10000UL   // periodo del lazo de ruedas, odometria y seguridad: 10 ms (100 Hz)
#define POSE_DIV        2u        // lazo de pose cada 2 periodos: 20 ms (50 Hz)
#define WDT_HW          WDTO_250MS// watchdog de hardware del AVR

// Motores (driver Model Y, 2 x TB6612). IN1=1, IN2=0 -> sentido "+" del canal (a calibrar).
static const uint8_t PIN_PWM[4] = {10, 9, 12, 11};   // FL D10 (OC2A), FR D9 (OC2B), RL D12 (OC1B), RR D11 (OC1A)
static const uint8_t PIN_IN1[4] = {26, 22, 7, 5};
static const uint8_t PIN_IN2[4] = {28, 24, 8, 6};
#define PWM_PRESCALER_8 1         // Timer1/Timer2 a ~3,9 kHz (por defecto Arduino: ~490 Hz)

// Encoders Hall en el puerto K (A8..A15), una sola interrupcion PCINT2, decodificacion x4.
// Bits de PINK: A8=PK0 ... A15=PK7.  S1 = canal A (naranja), S2 = canal B (verde).
static const uint8_t HALL_A_BIT[4] = {1, 0, 3, 2};   // FL A9, FR A8, RL A11, RR A10
static const uint8_t HALL_B_BIT[4] = {5, 4, 7, 6};   // FL A13, FR A12, RL A15, RR A14
#define HALL_PULLUPS    1         // pull-ups internos del puerto K (verificar si las placas Hall los traen)

// Bateria: divisor 10k/10k + 100 nF en A5 (A0..A4 son la barra de linea)
#define BAT_PIN         A5
#define BAT_FACTOR      2.0f      // Vbat = V(A5) * factor
#define VREF_MV         5000.0f

// Parada de emergencia opcional: contacto NC a GND en este pin (pull-up interno). 0 = no instalada.
#define ESTOP_ENABLED   0
#define ESTOP_PIN       3

// Telemetria y buffers
#define TX_BUF          1024u     // cola de transmision no bloqueante
#define RX_LINE         96u
#define TEL_HZ_MAX      50u
