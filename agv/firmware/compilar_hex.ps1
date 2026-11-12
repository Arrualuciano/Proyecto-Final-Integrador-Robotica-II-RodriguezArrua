# compilar_hex.ps1 — Compila el firmware del AGV y deja el HEX en agv/firmware/build/ (ruta estable).
# No depende de rutas fijas: usa la carpeta de este script y busca arduino-cli en la instalación
# del Arduino IDE o en el PATH. No carga la placa: el HEX se carga con el AFR2 Loader o con el IDE.
#   powershell -ExecutionPolicy Bypass -File agv\firmware\compilar_hex.ps1
$ErrorActionPreference = 'Stop'
$sketch = Join-Path $PSScriptRoot 'agv_osoyoo'
$build  = Join-Path $PSScriptRoot 'build'
# ProgramW6432 = "Program Files" de 64 bits aunque este PowerShell sea de 32 bits (lo lanza el make de
# Atmel Studio cuando se compila desde el AFR2 Loader).
$cli = @(
  (Join-Path $env:ProgramW6432 'Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'),
  (Join-Path $env:ProgramFiles 'Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'),
  (Join-Path $env:LOCALAPPDATA 'Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'),
  ((Get-Command arduino-cli -ErrorAction SilentlyContinue).Source)
) | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1
if (-not $cli) { throw 'No se encontró arduino-cli (instalar Arduino IDE 2 o agregar arduino-cli al PATH).' }
& $cli compile --fqbn arduino:avr:mega --warnings default --build-path $build $sketch
if ($LASTEXITCODE -ne 0) { throw "La compilación falló (código $LASTEXITCODE)." }
$hex = Join-Path $build 'agv_osoyoo.ino.hex'
Write-Output "HEX listo: $hex"
