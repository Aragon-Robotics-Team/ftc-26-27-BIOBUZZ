# Sets up a Windows laptop to run the sim and watch it in AdvantageScope (replay-auto.bat). Run it with setup-sim.bat
# in the repo's folder. Running it again is safe: it skips what is already there.
#
# Installs JDK 17, Python with numpy/scipy/trimesh (tools\advantagescope\.venv, which replay-auto.bat uses), and
# AdvantageScope, all in the user's own folders, so it doesn't need admin; puts the sim field into AdvantageScope; and
# builds the sim tests once, which downloads SimLoop and the rest. The robot is drawn as a box.
$ErrorActionPreference = 'Stop'

# sim-layout.json is saved from this version
$AdvantageScopeVersion = '26.0.2'

$Root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$UserAssets = Join-Path $env:APPDATA 'AdvantageScope\userAssets'
$AdvantageScopeExe = Join-Path $env:LOCALAPPDATA 'Programs\advantagescope\AdvantageScope.exe'
$Programs = Join-Path $env:LOCALAPPDATA 'Programs'
$Uv = Join-Path $env:USERPROFILE '.local\bin\uv.exe'
$Python = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
$Arm = $env:PROCESSOR_ARCHITECTURE -eq 'ARM64'
$Problems = @()

function Step($text) { Write-Host "`n== $text" -ForegroundColor Cyan }

function Download-Unpack($url, $destination) {
    $Zip = Join-Path $env:TEMP ([IO.Path]::GetRandomFileName() + '.zip')
    $ProgressPreference = 'SilentlyContinue'  # Invoke-WebRequest is very slow with its progress bar
    Invoke-WebRequest -UseBasicParsing -OutFile $Zip $url
    New-Item -ItemType Directory -Force $destination | Out-Null
    tar.exe -xf $Zip -C $destination
    if ($LASTEXITCODE -ne 0) { throw "Couldn't unpack $url." }
    Remove-Item $Zip
}

# Where setup-sim.bat puts it, or an installed one
function Find-Jdk17 {
    $places = @($Programs) + ('Microsoft', 'Eclipse Adoptium', 'Java' | ForEach-Object { Join-Path $env:ProgramFiles $_ })
    foreach ($place in $places) {
        $dir = Get-ChildItem -Directory -Path $place -Filter 'jdk-17*' -ErrorAction SilentlyContinue |
                Sort-Object Name | Select-Object -Last 1
        if ($dir) { return $dir.FullName }
    }
    return $null
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
    Write-Host 'Downloading Microsoft OpenJDK 17...'
    Download-Unpack "https://aka.ms/download-jdk/microsoft-jdk-17-windows-$(if ($Arm) { 'aarch64' } else { 'x64' }).zip" $Programs
    $Jdk = Find-Jdk17
}
if ($Jdk) { Write-Host $Jdk } else { throw "Couldn't install JDK 17." }
$env:JAVA_HOME = $Jdk

# ---------------------------------------------------------------- Python (uv brings its own)
Step 'Python'
if (-not (Test-Path $Uv)) {
    Write-Host 'Downloading uv...'
    $Target = if ($Arm) { 'aarch64-pc-windows-msvc' } else { 'x86_64-pc-windows-msvc' }
    Download-Unpack "https://github.com/astral-sh/uv/releases/latest/download/uv-$Target.zip" (Split-Path $Uv)
}
if (-not (Test-Path $Python)) { & $Uv venv --python 3.12 (Join-Path $PSScriptRoot '.venv') }
if (-not (Test-Path $Python)) { throw "Couldn't set up Python (above)." }
& $Python --version
& $Uv pip install --quiet --python $Python numpy scipy trimesh
if ($LASTEXITCODE -ne 0) { $Problems += "Couldn't install numpy, scipy and trimesh: the sim field won't be built." }

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
    $Name = "advantagescope-win-$(if ($Arm) { 'arm64' } else { 'x64' })-v$AdvantageScopeVersion.exe"
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
& $Python (Join-Path $PSScriptRoot 'build_sim_field.py') $UserAssets
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
