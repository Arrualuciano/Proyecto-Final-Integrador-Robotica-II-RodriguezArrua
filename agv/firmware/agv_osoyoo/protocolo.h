// protocolo.h — Protocolo de texto PC <-> AGV, telemetria no bloqueante y parametros en EEPROM.
// Formato: $CMD,arg1,arg2*HH\n  (HH = XOR en hexadecimal de los caracteres entre '$' y '*').
// Tambien se aceptan lineas escritas a mano sin '$' ni suma: "VEL 0.1 0 0". Ver LEEME_FIRMWARE.md.
#pragma once
#include <stdint.h>
#include "nucleo.h"

extern Parametros P;
extern Estado S;

void     prot_init(uint8_t mcusr);
void     prot_rx(char c, uint32_t ahora_ms, uint8_t enlace = 0);   // enlace 0 = USB (Serial), 1 = WiFi (Serial1)
void     prot_periodo(uint32_t ahora_ms, uint32_t periodo_us, uint32_t ejec_us);  // llamar en cada periodo de control
void     prot_flush();                       // vaciar cola TX sin bloquear, por el enlace del ultimo comando
void     prot_evento(const char *cuerpo);    // encolar "$cuerpo*HH" (avisos de otros modulos, p. ej. WiFi)
uint8_t  prot_enlace();                      // enlace por el que llego el ultimo comando
uint32_t prot_ultimo_msg_ms();
bool     prot_cargar_eeprom(Parametros &p);
