param(
    [switch]$Run,
    [switch]$Clean
)

$ErrorActionPreference = "Stop"
$root = Resolve-Path (Join-Path $PSScriptRoot "..\..\..\..")
$debugDir = Join-Path $root "Debug"
$makefile = Join-Path $debugDir "makefile"

Write-Host "C07A Debug build helper"
Write-Host "Repository: $root"
Write-Host "Debug dir : $debugDir"

if (-not (Test-Path -LiteralPath $makefile)) {
    Write-Error "Missing Debug/makefile; cannot identify Debug build entry."
    exit 2
}

$before = @{}
Get-ChildItem -LiteralPath $debugDir -File -Recurse -ErrorAction SilentlyContinue | ForEach-Object {
    $before[$_.FullName] = $_.LastWriteTimeUtc
}

$make = Get-Command gmake, make -ErrorAction SilentlyContinue | Select-Object -First 1
if ($null -eq $make) {
    Write-Host "No gmake/make command found in PATH."
    Write-Host "Plan only: open CCS or provide a shell where GNU make is available, then build Debug using Debug/makefile."
    if ($Run) {
        exit 3
    }
    exit 0
}

$argsList = @("-C", $debugDir)
if ($Clean) {
    $argsList += "clean"
}

Write-Host "Command: $($make.Source) $($argsList -join ' ')"

if (-not $Run) {
    Write-Host "Plan only. Re-run with -Run to execute. A build can update Debug/ artifacts."
    exit 0
}

& $make.Source @argsList
$code = $LASTEXITCODE
Write-Host "Build exit code: $code"

$changed = @()
Get-ChildItem -LiteralPath $debugDir -File -Recurse -ErrorAction SilentlyContinue | ForEach-Object {
    if (-not $before.ContainsKey($_.FullName) -or $before[$_.FullName] -ne $_.LastWriteTimeUtc) {
        $changed += $_.FullName.Substring($root.Path.Length + 1)
    }
}

if ($changed.Count -gt 0) {
    Write-Host "Changed build artifacts:"
    $changed | Sort-Object | ForEach-Object { Write-Host "  $_" }
} else {
    Write-Host "No changed build artifacts detected."
}

exit $code
