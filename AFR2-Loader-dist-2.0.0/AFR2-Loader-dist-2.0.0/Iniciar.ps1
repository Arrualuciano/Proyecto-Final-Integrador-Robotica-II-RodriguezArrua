[CmdletBinding()]
param([switch]$SoloCompilar, [ValidateSet('web','at')][string]$FirmwareEsp, [string]$Perfil, [string]$Datos)
$ErrorActionPreference = 'Stop'
try {
    if ($Datos) { $env:AFR2_LOADER_DATA_DIR = [IO.Path]::GetFullPath($Datos) }
    $program = Join-Path $PSScriptRoot 'AFR2 Loader.exe'
    if ($SoloCompilar) { Write-Output $program; exit 0 }
    $arguments = @('"' + $PSScriptRoot + '"')
    if ($Perfil) {
        if ($Perfil -ne [IO.Path]::GetFileName($Perfil) -or $Perfil.IndexOfAny([IO.Path]::GetInvalidFileNameChars()) -ge 0 -or -not $Perfil.EndsWith('.json', [StringComparison]::OrdinalIgnoreCase)) { throw 'Perfil debe ser el nombre de un JSON guardado en la biblioteca local.' }
        $arguments += @('--profile', ('"' + $Perfil + '"'))
    }
    if ($FirmwareEsp) { $arguments += @('--esp-mode', $FirmwareEsp) }
    Start-Process -FilePath $program -ArgumentList ($arguments -join ' ') -WorkingDirectory $PSScriptRoot
} catch { [Console]::Error.WriteLine($_.Exception.Message); exit 1 }
