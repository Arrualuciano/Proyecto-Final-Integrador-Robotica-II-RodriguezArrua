// EJEMPLO: copiar como wifi_config.h y completar la red y la IP de la PC (wifi_config.h no se sube al repositorio).
// wifi_config.h — Datos de la red WiFi para el enlace PC <-> AGV (ESP8266 del shield, firmware AT de fabrica).
// COMPLETAR antes de compilar. Este archivo tiene la clave de la red: no compartirlo ni subirlo a un repositorio.
//
// El robot se conecta a la misma red que la PC (modo estacion) y abre un enlace UDP transparente:
//   robot (puerto WIFI_PUERTO_ROBOT)  <-->  PC (WIFI_PC_IP, puerto WIFI_PUERTO_PC)
// Si la PC cambia de IP (ipconfig), hay que actualizar WIFI_PC_IP y volver a compilar y cargar.
// Hotspot de un iPhone: activar "Maximizar compatibilidad" (el ESP8266 solo usa 2,4 GHz).
#pragma once

#define WIFI_HABILITADO   1                    // 0 = solo USB
#define WIFI_SSID         "LabPrototip"        // nombre de la red
#define WIFI_CLAVE        "COMPLETAR_CLAVE"          // clave de la red
#define WIFI_PC_IP        "192.168.0.10"      // IP de la PC con la consola en esa red (ipconfig, 11/11/2026)
#define WIFI_PUERTO_ROBOT 4210
#define WIFI_PUERTO_PC    4211
