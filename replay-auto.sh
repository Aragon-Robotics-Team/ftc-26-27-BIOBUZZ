#!/bin/sh
# Runs an Auto in the sim and opens it in AdvantageScope (tools/advantagescope/replay_auto.py). Works from any folder:
#
#   ./replay-auto.sh                                  every Auto on one field
#   ./replay-auto.sh one_cycle_garden_park            just that one, with the balls
#   ./replay-auto.sh one_cycle_garden_park --blue
#   ./replay-auto.sh --no-sim                         reopen the last run's logs
#
# Then press play (space) in AdvantageScope.
cd "$(dirname "$0")" || exit 1
# setup-sim.command's Python, which has what the sim field needs
PYTHON=tools/advantagescope/.venv/bin/python
[ -x "$PYTHON" ] || PYTHON=python3
exec "$PYTHON" tools/advantagescope/replay_auto.py "$@"
