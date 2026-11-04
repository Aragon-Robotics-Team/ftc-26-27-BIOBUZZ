# Sets up a Windows laptop to run the sim and watch it in AdvantageScope (replay-auto.bat). Run it with setup-sim.bat
# in the repo's folder. Running it again is safe: it skips what is already there.
#
# Installs JDK 17, Python with numpy/scipy/trimesh, and AdvantageScope; puts the sim field into AdvantageScope; and
# builds the sim tests once, which downloads SimLoop and the rest. The robot is drawn as a box.
$ErrorActionPreference = 'Stop'

# sim-layout.json is saved from this version
$AdvantageScopeVersion = '26.0.2'

$Root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$UserAssets = Join-Path $env:APPDATA 'AdvantageScope\userAssets'
$AdvantageScopeExe = Join-Path $env:LOCALAPPDATA 'Programs\advantagescope\AdvantageScope.exe'
$Problems = @()

function Step($text) { Write-Host "`n== $text" -ForegroundColor Cyan }

function Refresh-Path {
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
            [Environment]::GetEnvironmentVariable('Path', 'User')
}

function Winget-Install($id) {
    if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
        throw "winget is missing. Install 'App Installer' from the Microsoft Store, then run this again."
    }
    winget install --id $id --exact --silent --accept-package-agreements --accept-source-agreements
    Refresh-Path
}

function Find-Jdk17 {
    foreach ($vendor in 'Microsoft', 'Eclipse Adoptium', 'Java') {
        $dir = Get-ChildItem -Directory -Path (Join-Path $env:ProgramFiles $vendor) -Filter 'jdk-17*' `
                -ErrorAction SilentlyContinue | Sort-Object Name | Select-Object -Last 1
        if ($dir) { return $dir.FullName }
    }
    return $null
}

function Has-Python {
    if (-not (Get-Command py -ErrorAction SilentlyContinue)) { return $false }
    # Windows PowerShell turns a redirected stderr line into a stopping error
    try { py -3 --version *> $null } catch { return $false }
    return $LASTEXITCODE -eq 0
}

function Find-AndroidSdk {
    foreach ($dir in $env:ANDROID_HOME, $env:ANDROID_SDK_ROOT, (Join-Path $env:LOCALAPPDATA 'Android\Sdk')) {
        if ($dir -and (Test-Path (Join-Path $dir 'platforms'))) { return $dir }
    }
    return $null
}

# ---------------------------------------------------------------- JDK 17
Step 'JDK 17'
$Jdk = Find-Jdk17
if (-not $Jdk) {
    Winget-Install 'Microsoft.OpenJDK.17'
    $Jdk = Find-Jdk17
}
if ($Jdk) { Write-Host $Jdk } else { throw "Couldn't install JDK 17." }
$env:JAVA_HOME = $Jdk

# ---------------------------------------------------------------- Python
Step 'Python'
if (-not (Has-Python)) { Winget-Install 'Python.Python.3.12' }
if (-not (Has-Python)) { throw "Couldn't install Python. Close this window, open a new one and run setup-sim.bat again." }
py -3 --version
py -3 -m pip install --user --quiet --disable-pip-version-check numpy scipy trimesh
if ($LASTEXITCODE -ne 0) { $Problems += "pip couldn't install numpy, scipy and trimesh: the sim field won't be built." }

# ---------------------------------------------------------------- Android SDK (the build needs it)
Step 'Android SDK'
$LocalProperties = Join-Path $Root 'local.properties'
if (Test-Path $LocalProperties) {
    Write-Host 'local.properties is there'
} else {
    $Sdk = Find-AndroidSdk
    if ($Sdk) {
        "sdk.dir=$($Sdk.Replace('\', '\\'))" | Set-Content -Encoding ASCII $LocalProperties
        Write-Host $Sdk
    } else {
        throw 'No Android SDK. Install Android Studio, open this project in it once, then run this again.'
    }
}

# ---------------------------------------------------------------- AdvantageScope
Step "AdvantageScope $AdvantageScopeVersion"
if (Test-Path $AdvantageScopeExe) {
    Write-Host $AdvantageScopeExe
} else {
    $Arch = if ($env:PROCESSOR_ARCHITECTURE -eq 'ARM64') { 'arm64' } else { 'x64' }
    $Name = "advantagescope-win-$Arch-v$AdvantageScopeVersion.exe"
    $Installer = Join-Path $env:TEMP $Name
    Write-Host "Downloading $Name..."
    $ProgressPreference = 'SilentlyContinue'  # Invoke-WebRequest is very slow with its progress bar
    Invoke-WebRequest -UseBasicParsing -OutFile $Installer `
            "https://github.com/Mechanical-Advantage/AdvantageScope/releases/download/v$AdvantageScopeVersion/$Name"
    Start-Process -Wait -FilePath $Installer -ArgumentList '/S'
    Remove-Item $Installer
    if (-not (Test-Path $AdvantageScopeExe)) { throw "AdvantageScope didn't install to $AdvantageScopeExe." }
    Write-Host $AdvantageScopeExe
}

# ---------------------------------------------------------------- the sim field and hives
Step 'Sim field'
py -3 (Join-Path $PSScriptRoot 'build_sim_field.py') $UserAssets
if ($LASTEXITCODE -ne 0) { $Problems += "Couldn't build the sim field: AdvantageScope will show its default field." }

# ---------------------------------------------------------------- the sim tests, built once
Step 'Building the sim tests (the first time downloads a lot)'
Push-Location $Root
try {
    .\gradlew.bat :TeamCode:compileDebugUnitTestJavaWithJavac
    if ($LASTEXITCODE -ne 0) { throw 'The build failed (above).' }
} finally {
    Pop-Location
}

Write-Host ''
foreach ($problem in $Problems) { Write-Host $problem -ForegroundColor Yellow }
Write-Host 'Done. Run an Auto in the sim: replay-auto.bat (or replay-auto.bat one_cycle_garden_park --blue)' `
        -ForegroundColor Green
