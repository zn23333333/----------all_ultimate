param(
    [string]$WorkRoot = (Join-Path ([System.IO.Path]::GetTempPath()) ("esp32n2_light_human_ascii_" + (Get-Date -Format "yyyyMMdd_HHmmss"))),
    [string]$BuildDir = "build"
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

function Get-RequiredFile {
    param(
        [string]$SearchRoot,
        [string]$Filter
    )

    $item = Get-ChildItem -Path $SearchRoot -Recurse -File -Filter $Filter |
        Sort-Object FullName |
        Select-Object -First 1

    if ($null -eq $item) {
        throw "Could not find '$Filter' under '$SearchRoot'."
    }

    return $item.FullName
}

$projectRoot = $PSScriptRoot
$settingsPath = Join-Path $projectRoot ".vscode\settings.json"
$settings = $null

if (Test-Path $settingsPath) {
    $settings = Get-Content $settingsPath -Raw | ConvertFrom-Json
}

$idfPath = if ($env:IDF_PATH) {
    $env:IDF_PATH
} elseif ($settings -and $settings.'idf.espIdfPathWin') {
    $settings.'idf.espIdfPathWin'
} else {
    throw "IDF_PATH is not set and .vscode/settings.json does not define idf.espIdfPathWin."
}

$toolsPath = if ($env:IDF_TOOLS_PATH) {
    $env:IDF_TOOLS_PATH
} elseif ($settings -and $settings.'idf.toolsPathWin') {
    $settings.'idf.toolsPathWin'
} else {
    throw "IDF_TOOLS_PATH is not set and .vscode/settings.json does not define idf.toolsPathWin."
}

$idfVersionPrefix = $null
if ($idfPath -match "v(\d+\.\d+)") {
    $idfVersionPrefix = "idf$($Matches[1])_"
}

$pythonEnvDirs = Get-ChildItem -Path (Join-Path $toolsPath "python_env") -Directory |
    Where-Object { $null -eq $idfVersionPrefix -or $_.Name -like "$idfVersionPrefix*" } |
    Sort-Object Name -Descending

$pythonExe = $null
foreach ($dir in $pythonEnvDirs) {
    $candidate = Join-Path $dir.FullName "Scripts\python.exe"
    if (Test-Path $candidate) {
        $pythonExe = $candidate
        break
    }
}

if ($null -eq $pythonExe) {
    throw "Could not find an ESP-IDF Python virtual environment under '$toolsPath\python_env'."
}

$cmakeExe = Get-RequiredFile -SearchRoot (Join-Path $toolsPath "tools\cmake") -Filter "cmake.exe"
$ninjaExe = Get-RequiredFile -SearchRoot (Join-Path $toolsPath "tools\ninja") -Filter "ninja.exe"
$xtensaGcc = Get-RequiredFile -SearchRoot (Join-Path $toolsPath "tools\xtensa-esp-elf") -Filter "xtensa-esp32-elf-gcc.exe"
$espRomElfDir = Get-ChildItem -Path (Join-Path $toolsPath "tools\esp-rom-elfs") -Directory |
    Sort-Object Name -Descending |
    Select-Object -First 1 -ExpandProperty FullName

if ($null -eq $espRomElfDir) {
    throw "Could not find ESP ROM ELF files under '$toolsPath\tools\esp-rom-elfs'."
}

$sourceDir = Join-Path $WorkRoot "src"
New-Item -ItemType Directory -Force -Path $sourceDir | Out-Null

$excludeDirs = @(".vscode", ".devcontainer")
$excludeDirs += Get-ChildItem -Path $projectRoot -Directory |
    Where-Object { $_.Name -like "build*" } |
    Select-Object -ExpandProperty Name

$robocopyArgs = @($projectRoot, $sourceDir, "/E")
if ($excludeDirs.Count -gt 0) {
    $robocopyArgs += "/XD"
    $robocopyArgs += $excludeDirs
}

Write-Host "Mirroring project into ASCII-only path: $sourceDir"
& robocopy @robocopyArgs | Out-Host
if ($LASTEXITCODE -gt 7) {
    throw "robocopy failed with exit code $LASTEXITCODE."
}

$env:PYTHONUTF8 = "1"
$env:IDF_PATH = $idfPath
$env:IDF_TOOLS_PATH = $toolsPath
$env:IDF_PYTHON_ENV_PATH = Split-Path -Parent (Split-Path -Parent $pythonExe)
$env:ESP_ROM_ELF_DIR = $espRomElfDir

if ($settings -and $settings.'idf.customExtraVars' -and $settings.'idf.customExtraVars'.IDF_TARGET) {
    $env:IDF_TARGET = $settings.'idf.customExtraVars'.IDF_TARGET
}

$toolDirs = @(
    (Split-Path -Parent $pythonExe),
    (Split-Path -Parent $cmakeExe),
    (Split-Path -Parent $ninjaExe),
    (Split-Path -Parent $xtensaGcc)
)

$env:Path = (($toolDirs -join ";") + ";" + $env:Path)

$idfPy = Join-Path $idfPath "tools\idf.py"
if (-not (Test-Path $idfPy)) {
    throw "Could not find idf.py at '$idfPy'."
}

Push-Location $sourceDir
try {
    & $pythonExe $idfPy -B $BuildDir build
    if ($LASTEXITCODE -ne 0) {
        throw "idf.py build failed with exit code $LASTEXITCODE."
    }
} finally {
    Pop-Location
}

Write-Host ""
Write-Host "Build succeeded."
Write-Host "Build output: $(Join-Path $sourceDir $BuildDir)"
