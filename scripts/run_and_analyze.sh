#!/usr/bin/env bash
# Run drivebrain, and when it exits (Ctrl+C, or by itself once racing_line/end_run_when_done has finished the laps),
# run compare_paths.py on the recording it just wrote and open the plot.
#
#   scripts/run_and_analyze.sh            # from anywhere; uses python3, or set PYTHON=/path/to/python
#
# Start the sim (xcel) first, as for any run. The script needs: pip install mcap mcap-protobuf-support matplotlib numpy
set -u
cd "$(dirname "$0")/.."

PYTHON="${PYTHON:-python3}"
if ! "$PYTHON" -c "import mcap, mcap_protobuf, matplotlib, numpy" 2>/dev/null; then
    echo "run_and_analyze: $PYTHON is missing packages the analysis needs; install them first, or run the analysis elsewhere:"
    echo "    $PYTHON -m pip install mcap mcap-protobuf-support matplotlib numpy"
    exit 1
fi
mkdir -p recordings
newest() { ls -t recordings/*.mcap 2>/dev/null | head -n 1; }
before="$(newest)"

./run_local.sh
status=$?

after="$(newest)"
if [[ -z "$after" || "$after" == "$before" ]]; then
    echo "run_and_analyze: no new recording was written, nothing to analyze (drivebrain exited with $status)"
    exit "$status"
fi

echo "run_and_analyze: analyzing $after"
"$PYTHON" scripts/compare_paths.py "$after" --out recordings/analysis || exit $?

png="recordings/analysis/$(basename "$after" .mcap)_paths.png"
if command -v open >/dev/null 2>&1 && [[ -f "$png" ]]; then open "$png"; fi
exit "$status"
