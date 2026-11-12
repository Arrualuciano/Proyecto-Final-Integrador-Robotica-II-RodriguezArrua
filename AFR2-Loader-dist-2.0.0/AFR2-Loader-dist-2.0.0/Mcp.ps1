[CmdletBinding()]
param([string]$Datos)
$ErrorActionPreference = 'Stop'
try {
    if ($Datos) { $env:AFR2_LOADER_DATA_DIR = [IO.Path]::GetFullPath($Datos) }
    [Console]::InputEncoding = New-Object System.Text.UTF8Encoding($false)
    [Console]::OutputEncoding = New-Object System.Text.UTF8Encoding($false)
    $OutputEncoding = [Console]::OutputEncoding
    & (Join-Path $PSScriptRoot 'runtime/AFR2Loader.Mcp.exe') --mcp $PSScriptRoot
    exit $LASTEXITCODE
} catch { [Console]::Error.WriteLine('AFR2 MCP: ' + $_.Exception.Message); exit 1 }
