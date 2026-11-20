#!/usr/bin/env bash

# Runs drivebrain on the sim of one track automatically.
# Exits 0 if the car finishes the laps, nonzero if it doesn't. 
# Usage ./scripts/run_sim_test.sh [track] [laps] [timeout_seconds]

set -euo pipefail

TRACK="${1:-acceleration}"
LAPS="${2:-1}"
TIMEOUT_S="${3:-300}"

cd "$(dirname "$0")/.." #move into main drivebrain folder from wherever you are 


#drivebrain starts with the controller disabled, so make a copy of the config with it on

python3 -c ' 
import json
c = json.load(open("config/drivebrain_config.json"))
c["DrivebrainControllerInterface"]["enabled"] = True
json.dump(c, open("/tmp/drivebrain_sim_config.json", "w"), indent = 4)
'

cleanup() {
    docker stop drivebrain-sim > /dev/null 2>&1 || true
}
trap cleanup EXIT



# Start drivebrain in the background inside the container.
# SIM_HOST only gets passed in if it was set (needed on Windows, not on GitHub).
# Output goes to drivebrain_sim.log so it doesn't mix with the sim's output.

mkdir -p recordings
echo "Starting drivebrain"

docker compose run --rm -T --name drivebrain-sim ${SIM_HOST:+-e SIM_HOST=$SIM_HOST} dev \
    ./build-native/drivebrain -c /tmp/drivebrain_sim_config.json -d dbc/ht_can.dbc \
    > drivebrain_sim.log 2>&1 &

# Run the sim until the car finishes the laps or the time limit is reached
# Its exit code is the result: 0 = finished, 1 = didn't finish, 124 = timed out

echo "Running sim: track=$TRACK laps=$LAPS timeout=${TIMEOUT_S}s"
set +e
(cd simulation && timeout "$TIMEOUT_S" uv run main.py --track "$TRACK" --laps "$LAPS" --logless)
RESULT=$?

set -e

if [ "$RESULT" -eq 0 ]; then 
    echo "PASS: finished $LAPS lap(s) on $TRACK"
elif [ "$RESULT" -eq 124 ]; then
    echo "FAIL: hit the ${TIMEOUT_S}s time limit on $TRACK"
else 
    echo "FAIL: sim exited with code $RESULT on $TRACK (check the output above)"
fi 

exit "$RESULT"