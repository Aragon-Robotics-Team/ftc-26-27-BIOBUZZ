"""Runs an Auto in the sim and opens it in AdvantageScope, with the tabs set up. One step:

    python3 tools/advantagescope/replay_auto.py                         every Auto on one field
    python3 tools/advantagescope/replay_auto.py one_cycle_garden_park   just that one, with the balls
    python3 tools/advantagescope/replay_auto.py one_cycle_garden_park --blue
    python3 tools/advantagescope/replay_auto.py --no-sim                reopen the last run's logs

Then press play (space) in AdvantageScope. It replaces AdvantageScope's open tabs with the sim's (sim-layout.json for
one Auto, an "All autos" tab for all of them), quitting AdvantageScope first if it's open: it only reads its saved tabs
when it starts. The tabs it had are kept in state-*.json.before-replay next to its settings.
"""
import argparse
import glob
import json
import os
import platform
import re
import secrets
import shutil
import string
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SIM = os.path.join(ROOT, "TeamCode", "build", "sim")
ROUTINES_JAVA = os.path.join(ROOT, "TeamCode", "src", "main", "java", "org", "firstinspires", "ftc", "teamcode",
                             "routines", "Routines.java")
SYSTEM = platform.system()
GAME = "FTC:BIOBUZZ Sim (tilting hives)"
COLORS = ["#00ff00", "#ff8c00", "#00ffff", "#ffff00", "#ff00ff", "#ff0000", "#0000ff"]

sys.path.insert(0, HERE)
import merge_autos  # noqa: E402


def routines():
    """Every routine in Routines.java, lower case, as the logs name them (the @Disabled ones never get a log)."""
    source = open(ROUTINES_JAVA).read()
    body = source[source.index("enum Routines"):source.index(";", source.index("enum Routines"))]
    return [name.lower() for name in re.findall(r"^\s*([A-Z0-9_]+)\(", body, re.M)]


def run_sim(routine, alliance):
    gradlew = os.path.join(ROOT, "gradlew.bat" if SYSTEM == "Windows" else "gradlew")
    command = [gradlew, ":TeamCode:testDebugUnitTest", "--tests", "*AutoSimTest*", "--rerun", "--info",
               f"-PsimRoutine={routine}", f"-PsimAlliance={alliance}"]
    if SYSTEM != "Windows":
        command.insert(0, "sh")
    # A SimLoop built and published locally (./gradlew :SimLoop:publishToMavenLocal in the SimLoop clone)
    if os.path.isdir(os.path.expanduser("~/.m2/repository/org/horizon36596/SimLoop")):
        command.append("-PsimloopLocal")
    env = dict(os.environ)
    if SYSTEM == "Darwin":
        try:
            env["JAVA_HOME"] = subprocess.check_output(["/usr/libexec/java_home", "-v", "17"], text=True).strip()
        except (OSError, subprocess.CalledProcessError):
            pass  # whatever JAVA_HOME says
    print(f"Running {routine if routine != 'all' else 'every Auto'} ({alliance}) in the sim...")
    result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True)
    for line in result.stdout.splitlines():
        if re.search(r"Auto \S+: \d+ shots|FAILED", line):
            print("  " + line.strip())
    if result.returncode != 0:
        # A failed check still leaves its log, so show what there is
        print("  The sim test failed (above); opening its log anyway. Details: TeamCode/build/reports/tests/")


def assets_dir():
    if SYSTEM == "Darwin":
        base = os.path.expanduser("~/Library/Application Support/AdvantageScope")
    elif SYSTEM == "Windows":
        base = os.path.join(os.environ.get("APPDATA", ""), "AdvantageScope")
    else:
        base = os.path.expanduser("~/.config/AdvantageScope")
    return base


def check_assets(base):
    user_assets = os.path.join(base, "userAssets")
    needed = ["Field3d_BIOBUZZSim", "Robot_BIOBUZZ Hive Red", "Robot_BIOBUZZ Hive Blue"]
    if not all(os.path.isdir(os.path.join(user_assets, name)) for name in needed):
        print("Building the sim field and hive models into AdvantageScope (build_sim_field.py)...")
        if subprocess.run([sys.executable, os.path.join(HERE, "build_sim_field.py")]).returncode != 0:
            print("  Couldn't build them (needs numpy and trimesh). AdvantageScope will show its default field.")
    if not os.path.isdir(os.path.join(user_assets, "Robot_BIOBUZZ")):
        print("  No Robot_BIOBUZZ model in AdvantageScope's userAssets: the robot is drawn as a plain box. "
              "Copy that folder from a computer that has it.")


def uuid():
    return "".join(secrets.choice(string.ascii_lowercase + string.digits) for _ in range(32))


def all_autos_tabs(names):
    def source(kind, key, log_type, **options):
        return {"type": kind, "logKey": key, "logType": log_type, "visible": True, "options": options}
    sources = []
    for name, color in zip(names, COLORS * 3):
        sources.append(source("ghost", f"{name}/RealOutputs/Sim/RobotPose", "Pose2d", model="BIOBUZZ", color=color))
        sources.append(source("trajectory", f"{name}/RealOutputs/Path", "Translation2d[]", color=color, size="bold"))
    first = names[0] + "/RealOutputs/"
    sources += [source("robot", first + "Hive/red/trayPose3d", "Pose3d", model="BIOBUZZ Hive Red"),
                source("robot", first + "Hive/blue/trayPose3d", "Pose3d", model="BIOBUZZ Hive Blue"),
                source("robot", first + "Sim/partner/pose2d", "Pose2d", model="Drive Base (Simple)")]
    return [{"type": 0, "title": "", "controller": None, "controllerUUID": uuid(), "renderer": "", "controlsHeight": 0},
            {"type": 3, "title": "All autos", "controller": {"sources": sources, "game": GAME},
             "controllerUUID": uuid(), "controlsHeight": 200,
             "renderer": {"cameraIndex": -1, "orbitFov": 50, "cameraPosition": [0, 4.2, 1.4], "cameraTarget": [0, 0, 0]}}]


def quit_advantagescope():
    if SYSTEM == "Darwin":
        if subprocess.run(["pgrep", "-x", "AdvantageScope"], capture_output=True).returncode != 0:
            return
        print("Closing AdvantageScope to set up its tabs...")
        subprocess.run(["osascript", "-e", 'tell application "AdvantageScope" to quit'], capture_output=True)
        for _ in range(40):
            if subprocess.run(["pgrep", "-x", "AdvantageScope"], capture_output=True).returncode != 0:
                return
            time.sleep(0.25)
        raise SystemExit("AdvantageScope didn't close; close it and run this again.")
    if SYSTEM == "Windows":
        running = subprocess.run(["tasklist", "/FI", "IMAGENAME eq AdvantageScope.exe"], capture_output=True, text=True)
        if "AdvantageScope.exe" in running.stdout:
            raise SystemExit("Close AdvantageScope first: it only reads its saved tabs when it starts.")


def install_tabs(base, tabs):
    """Puts our tabs in AdvantageScope's saved state, which it restores when it starts."""
    states = sorted(glob.glob(os.path.join(base, "state-*.json")), key=os.path.getmtime)
    if states:
        path = states[-1]
        state = json.load(open(path))
        backup = path + ".before-replay"
        if not os.path.exists(backup):
            shutil.copy(path, backup)
    else:
        os.makedirs(base, exist_ok=True)
        layout = json.load(open(os.path.join(HERE, "sim-layout.json")))
        path = os.path.join(base, f"state-{layout['version'].replace('.', '_')}.json")
        state = {"hubs": layout["hubs"], "satellites": []}
    state["hubs"][0]["state"]["tabs"] = {"selected": 1, "tabs": tabs}
    json.dump(state, open(path, "w"))


def open_log(path):
    if SYSTEM == "Darwin":
        subprocess.run(["open", "-a", "AdvantageScope", path], check=True)
    elif SYSTEM == "Windows":
        exe = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs", "advantagescope", "AdvantageScope.exe")
        if not os.path.exists(exe):
            raise SystemExit(f"Couldn't find {exe}. Open {path} in AdvantageScope (File > Open Log).")
        subprocess.Popen([exe, path])
    else:
        subprocess.Popen(["advantagescope", path])


def main():
    parser = argparse.ArgumentParser(description="Run an Auto in the sim and open it in AdvantageScope.")
    parser.add_argument("routine", nargs="?", default="all", help="e.g. one_cycle_garden_park; leave out for every Auto")
    parser.add_argument("--blue", action="store_true", help="blue alliance (red otherwise)")
    parser.add_argument("--no-sim", action="store_true", help="don't rerun the sim, open the last logs")
    args = parser.parse_args()
    routine = args.routine.lower()
    alliance = "blue" if args.blue else "red"
    known = routines()
    if routine != "all" and routine not in known:
        raise SystemExit(f"No routine {args.routine}. Routines: {', '.join(known)}")

    if not args.no_sim:
        run_sim(routine, alliance)
    names = [f"{r}-{alliance}" for r in (known if routine == "all" else [routine])]
    logs = [os.path.join(SIM, f"auto-{name}.rlog") for name in names]
    found = [(name, log) for name, log in zip(names, logs) if os.path.exists(log)]
    if not found:
        raise SystemExit(f"No sim log for {routine} ({alliance}) in {SIM}.")

    base = assets_dir()
    check_assets(base)
    if routine == "all":
        log = os.path.join(SIM, f"all-autos-{alliance}.rlog")
        merge_autos.main([path for _, path in found], log)
        tabs = all_autos_tabs([name for name, _ in found])
    else:
        log = found[0][1]
        tabs = json.load(open(os.path.join(HERE, "sim-layout.json")))["hubs"][0]["state"]["tabs"]["tabs"]
    quit_advantagescope()
    install_tabs(base, tabs)
    open_log(log)
    print(f"Opened {os.path.relpath(log, ROOT)} in AdvantageScope. Press play (space) or click the timeline.")


if __name__ == "__main__":
    main()
