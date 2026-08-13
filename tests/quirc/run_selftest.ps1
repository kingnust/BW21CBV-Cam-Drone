$ErrorActionPreference = "Stop"

$root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$vendor = Join-Path $root "vendor\esp32-qrcode-reader\quirc"
$compiler = Get-Command gcc -ErrorAction Stop
$output = Join-Path ([IO.Path]::GetTempPath()) "bw21-quirc-selftest-$PID.exe"
$sources = @(
    (Join-Path $PSScriptRoot "quirc_selftest.c"),
    (Join-Path $vendor "quirc.c"),
    (Join-Path $vendor "identify.c"),
    (Join-Path $vendor "decode.c"),
    (Join-Path $vendor "version_db.c")
)

try {
    & $compiler.Source -std=c11 -O2 -Wall -Wextra -I $vendor @sources -lm -o $output
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $output
    exit $LASTEXITCODE
} finally {
    Remove-Item -LiteralPath $output -Force -ErrorAction SilentlyContinue
}
