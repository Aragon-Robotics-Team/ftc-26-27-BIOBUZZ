"""Puts every Auto's sim log on one field: TeamCode/build/sim/auto-*.rlog -> TeamCode/build/sim/all-autos.rlog.

Each log's fields go under its routine's name (one_cycle_garden_park-red/RealOutputs/...), with the robot's whole route
added as <name>/RealOutputs/Path (a Translation2d[], so AdvantageScope draws it as a line from the start). AutoSimTest
gives every routine the same init length, so their Autos start together.

    ./gradlew :TeamCode:testDebugUnitTest --tests '*AutoSimTest*'
    python3 tools/advantagescope/merge_autos.py
"""
import glob
import os
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SIM = os.path.join(ROOT, "TeamCode", "build", "sim")
POSE_KEY = "RealOutputs/Sim/RobotPose"
PATH_EVERY = 5  # route points: one every 5 ticks (0.1 s)


def read(path):
    """An RLOG (revision 2) as a list of (timestamp, [(key, type, value bytes)])."""
    data = open(path, "rb").read()
    if data[0] != 2:
        raise SystemExit(f"{path}: RLOG revision {data[0]}, expected 2")
    i, keys, cycles = 1, {}, []
    while i < len(data):
        record = data[i]
        i += 1
        if record == 0:
            cycles.append((struct.unpack(">d", data[i:i + 8])[0], []))
            i += 8
        elif record == 1:
            key_id, length = struct.unpack(">HH", data[i:i + 4])
            i += 4
            key = data[i:i + length].decode()
            i += length
            length = struct.unpack(">H", data[i:i + 2])[0]
            i += 2
            keys[key_id] = (key, data[i:i + length].decode())
            i += length
        elif record == 2:
            key_id, length = struct.unpack(">HH", data[i:i + 4])
            i += 4
            cycles[-1][1].append((*keys[key_id], data[i:i + length]))
            i += length
        else:
            raise SystemExit(f"{path}: unknown record {record} at byte {i - 1}")
    return cycles


def route(cycles):
    """The robot's poses over the run, as a Translation2d[] value."""
    points = [value[:16] for _, values in cycles for key, _, value in values if key == POSE_KEY]
    return b"".join(points[::PATH_EVERY] + points[-1:])


class Writer:
    def __init__(self):
        self.out = bytearray([2])
        self.ids = {}

    def timestamp(self, t):
        self.out += b"\x00" + struct.pack(">d", t)

    def value(self, key, kind, value):
        if key not in self.ids:
            self.ids[key] = len(self.ids)
            k, t = key.encode(), kind.encode()
            self.out += b"\x01" + struct.pack(">HH", self.ids[key], len(k)) + k + struct.pack(">H", len(t)) + t
        self.out += b"\x02" + struct.pack(">HH", self.ids[key], len(value)) + value


def main(paths, output):
    logs = {}
    for path in sorted(paths):
        name = os.path.basename(path)[len("auto-"):-len(".rlog")]
        logs[name] = read(path)
        print(f"{name}: {len(logs[name])} cycles, {logs[name][-1][0]:.1f} s")

    by_time = {}
    for name, cycles in logs.items():
        for t, values in cycles:
            by_time.setdefault(round(t, 6), []).append((name, values))
    writer = Writer()
    for n, t in enumerate(sorted(by_time)):
        writer.timestamp(t)
        for name, values in by_time[t]:
            for key, kind, value in values:
                # Struct schemas stay where AdvantageScope looks for them; every log has the same ones
                writer.value(key if key.startswith("/.schema/") else f"{name}/{key}", kind, value)
        if n == 0:
            for name, cycles in logs.items():
                writer.value(f"{name}/RealOutputs/Path", "struct:Translation2d[]", route(cycles))
    open(output, "wb").write(writer.out)
    print(f"{output}: {len(logs)} Autos, {len(writer.out) / 1e6:.1f} MB")


if __name__ == "__main__":
    inputs = sys.argv[1:] or [p for p in glob.glob(os.path.join(SIM, "auto-*.rlog"))]
    main(inputs, os.path.join(SIM, "all-autos.rlog"))
