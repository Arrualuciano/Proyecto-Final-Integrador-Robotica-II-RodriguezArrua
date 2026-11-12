# Makefile - adaptador para compilar el firmware del AGV desde el AFR2 Loader (Compilar ATmega / build_mega).
#
# El Loader 2.0.0 ejecuta en AtmegaDir (= esta carpeta), en una sola llamada (registro del 07/11/2026):
#     make -B firmware communication-firmware library CC="\".../avr-gcc.exe\"" AR="\".../avr-ar.exe\"" OBJCOPY=...
# (los valores llegan CON comillas dentro) y despues exige build/afr2.hex, build/afr2-communication.hex y
# libafr2_ui.a (en la raiz de AtmegaDir). Ese esquema es el del proyecto AFR2 de Koba (producto, firmware de comunicacion y
# biblioteca). Nuestro firmware es un sketch de Arduino (agv_osoyoo/) que se compila con arduino-cli
# (compilar_hex.ps1), con el mismo avr-gcc 7.3.0 de Arduino15. Este Makefile lo adapta asi:
#   firmware (= all)        compila con arduino-cli -> build/agv_osoyoo.ino.hex y su copia build/afr2.hex
#   communication-firmware  no hay firmware de comunicacion aparte (el protocolo esta en el mismo sketch):
#                           deja build/afr2-communication.hex como copia del mismo HEX
#   library                 no hay biblioteca aparte: genera libafr2_ui.a (en esta carpeta) como archivo ar VACIO con el
#                           AR que pasa el Loader, para cumplir su comprobacion de artefactos
# Uso manual:  make   (con make de GNU, p. ej. el de Atmel Studio 7)

AR ?= avr-ar
AR_SIN_COMILLAS := $(subst ",,$(AR))
PS = powershell -NoProfile -NonInteractive -ExecutionPolicy Bypass

.PHONY: all firmware communication-firmware library clean

all: firmware

firmware:
	$(PS) -File compilar_hex.ps1
	$(PS) -Command "Copy-Item -LiteralPath 'build/agv_osoyoo.ino.hex' -Destination 'build/afr2.hex' -Force; Write-Output 'Copia para el Loader: build/afr2.hex'"

communication-firmware:
	$(PS) -Command "if (-not (Test-Path -LiteralPath 'build/agv_osoyoo.ino.hex')) { throw 'Falta build/agv_osoyoo.ino.hex: correr make primero.' }; Copy-Item -LiteralPath 'build/agv_osoyoo.ino.hex' -Destination 'build/afr2-communication.hex' -Force; Write-Output 'agv_osoyoo: el protocolo esta en el mismo firmware; build/afr2-communication.hex es copia de afr2.hex'"

library:
	$(PS) -Command "foreach ($$a in 'libafr2_ui.a','build/libafr2_ui.a') { if (Test-Path -LiteralPath $$a) { Remove-Item -LiteralPath $$a } }; exit 0"
	"$(AR_SIN_COMILLAS)" rcs libafr2_ui.a
	$(PS) -Command "Write-Output 'agv_osoyoo: sin biblioteca aparte; libafr2_ui.a es un archivo ar vacio para el Loader.'"

clean:
	$(PS) -Command "Remove-Item -Recurse -Force -LiteralPath 'build' -ErrorAction SilentlyContinue"
