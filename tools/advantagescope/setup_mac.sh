#!/bin/bash
# Sets up a Mac to run the sim and watch it in AdvantageScope (replay-auto.sh). Run it with setup-sim.command in the
# repo's folder (double-click it). Running it again is safe: it skips what is already there.
#
# Installs JDK 17, Python with numpy/scipy/trimesh (tools/advantagescope/.venv, which replay-auto.sh uses), and
# AdvantageScope, all in the user's own folders, so it doesn't need admin; puts the sim field into AdvantageScope; and
# builds the sim tests once, which downloads SimLoop and the rest. The robot is drawn as a box.
set -e

ADVANTAGESCOPE_VERSION=26.0.2  # sim-layout.json is saved from this version
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
USER_ASSETS="$HOME/Library/Application Support/AdvantageScope/userAssets"
VENV="$HERE/.venv"
UV="$HOME/.local/bin/uv"
PROBLEMS=()

step() { printf '\n\033[36m== %s\033[0m\n' "$1"; }
fail() { printf '\033[31m%s\033[0m\n' "$1"; exit 1; }

if [ "$(uname -m)" = arm64 ]; then ARCH=arm64; JDK_ARCH=aarch64; else ARCH=x64; JDK_ARCH=x64; fi
UV_TARGET=$([ "$ARCH" = arm64 ] && echo aarch64 || echo x86_64)-apple-darwin

# ---------------------------------------------------------------- JDK 17
step 'JDK 17'
if ! JAVA_HOME=$(/usr/libexec/java_home -v 17 2>/dev/null); then
    # Into the user's own JDK folder, where java_home finds it without an install
    JVMS="$HOME/Library/Java/JavaVirtualMachines"
    mkdir -p "$JVMS"
    echo 'Downloading Temurin 17...'
    curl -fsSL "https://api.adoptium.net/v3/binary/latest/17/ga/mac/$JDK_ARCH/jdk/hotspot/normal/eclipse" |
        tar -xz -C "$JVMS"
    JAVA_HOME=$(/usr/libexec/java_home -v 17 2>/dev/null) || fail "Couldn't install JDK 17."
fi
export JAVA_HOME
echo "$JAVA_HOME"

# ---------------------------------------------------------------- Python (uv brings its own: a Mac's python3 needs
# Apple's command line tools, which need admin)
step 'Python'
if [ ! -x "$UV" ]; then
    echo 'Downloading uv...'
    mkdir -p "$(dirname "$UV")"
    curl -fsSL "https://github.com/astral-sh/uv/releases/latest/download/uv-$UV_TARGET.tar.gz" |
        tar -xz --strip-components 1 -C "$(dirname "$UV")"
fi
[ -x "$VENV/bin/python" ] || "$UV" venv --python 3.12 "$VENV"
[ -x "$VENV/bin/python" ] || fail "Couldn't set up Python (above)."
"$VENV/bin/python" --version
"$UV" pip install --quiet --python "$VENV/bin/python" numpy scipy trimesh ||
    PROBLEMS+=("Couldn't install numpy, scipy and trimesh: the sim field won't be built.")

# ---------------------------------------------------------------- Android SDK (the build needs it)
step 'Android SDK'
if [ -f "$ROOT/local.properties" ]; then
    echo 'local.properties is there'
else
    SDK=
    for dir in "$ANDROID_HOME" "$ANDROID_SDK_ROOT" "$HOME/Library/Android/sdk"; do
        if [ -n "$dir" ] && [ -d "$dir/platforms" ]; then SDK=$dir; break; fi
    done
    [ -n "$SDK" ] || fail 'No Android SDK. Install Android Studio, open this project in it once, then run this again.'
    echo "sdk.dir=$SDK" > "$ROOT/local.properties"
    echo "$SDK"
fi

# ---------------------------------------------------------------- AdvantageScope
step "AdvantageScope $ADVANTAGESCOPE_VERSION"
if [ -d /Applications/AdvantageScope.app ] || [ -d "$HOME/Applications/AdvantageScope.app" ]; then
    echo 'AdvantageScope is there'
else
    NAME="advantagescope-mac-$ARCH-v$ADVANTAGESCOPE_VERSION.dmg"
    TMP=$(mktemp -d)
    echo "Downloading $NAME..."
    curl -fL --progress-bar -o "$TMP/$NAME" \
        "https://github.com/Mechanical-Advantage/AdvantageScope/releases/download/v$ADVANTAGESCOPE_VERSION/$NAME"
    hdiutil attach -quiet -nobrowse -readonly -mountpoint "$TMP/mount" "$TMP/$NAME"
    DEST=/Applications
    if [ ! -w "$DEST" ]; then DEST="$HOME/Applications"; mkdir -p "$DEST"; fi
    cp -R "$TMP/mount/AdvantageScope.app" "$DEST/"
    hdiutil detach -quiet "$TMP/mount"
    rm -rf "$TMP"
    echo "$DEST/AdvantageScope.app"
fi

# ---------------------------------------------------------------- the sim field and hives
step 'Sim field'
"$VENV/bin/python" "$HERE/build_sim_field.py" "$USER_ASSETS" ||
    PROBLEMS+=("Couldn't build the sim field: AdvantageScope will show its default field.")

# ---------------------------------------------------------------- the sim tests, built once
step 'Building the sim tests (the first time downloads a lot)'
(cd "$ROOT" && ./gradlew :TeamCode:compileDebugUnitTestJavaWithJavac) || fail 'The build failed (above).'

echo
for problem in "${PROBLEMS[@]}"; do printf '\033[33m%s\033[0m\n' "$problem"; done
printf '\033[32mDone. Run an Auto in the sim: ./replay-auto.sh (or ./replay-auto.sh one_cycle_garden_park --blue)\033[0m\n'
