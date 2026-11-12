// wifi.h — Enlace WiFi por el ESP8266 del shield (Serial1, D18/D19) con su firmware AT de fabrica.
// Al arrancar se configura el ESP con una maquina de estados NO bloqueante (watchdog de 250 ms) y queda en
// modo UDP transparente: lo que el Mega escribe en Serial1 sale por UDP a la PC y lo que llega por UDP entra
// por Serial1. Esquema tomado del firmware de T. Mendelevich y M. Baldoni para el mismo kit (agv_hal.cpp),
// reescrito y simplificado para este proyecto.
#pragma once
#include <stdint.h>

void        wifi_init();
void        wifi_poll(uint32_t ahora_ms);   // llamar en cada vuelta del loop: configura o pasa bytes a prot_rx
bool        wifi_listo();
const char *wifi_ip();
