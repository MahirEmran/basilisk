#!/usr/bin/env bash
# test_fov_combinations.sh
# Run all combinations of LOST/FOUND FOVs for a month and save unique outputs.
# Does NOT rebuild Basilisk or rerun conanfile.py.

set -euo pipefail

SIM_DIR="$(dirname "$0")/simulation"
PYTHON_BIN="$(dirname "$0")/.venv/bin/python"
SIM_SCRIPT="$SIM_DIR/simulate_cubesat.py"

# FOV ranges
LOST_FOVS=(10 15 20)
FOUND_FOVS=(50 55 60 65 70 75 80 85)

MONTH_HOURS=744  # 31 days

MAX_PROCS=4

run_sim() {
  local LOST="$1"
  local FOUND="$2"
  local OUT_FILE="$SIM_DIR/output_LOST${LOST}_FOUND${FOUND}.bin"
  local LOG_FILE="$SIM_DIR/output_LOST${LOST}_FOUND${FOUND}.log"
  local SUMMARY_FILE="$SIM_DIR/output_LOST${LOST}_FOUND${FOUND}_plots/summary.txt"
  echo "[RUN] LOST_FOV=$LOST, FOUND_FOV=$FOUND -> $OUT_FILE"
  "$PYTHON_BIN" "$SIM_SCRIPT" \
    --lost-fov "$LOST" \
    --found-fov "$FOUND" \
    --hours "$MONTH_HOURS" \
    --bin-path "$OUT_FILE" \
    > "$LOG_FILE" 2>&1

  # Extract all [UPTIME] blocks and their breakdowns/notes
  awk '/^\[UPTIME\]/ {p=1} p && (/^\[UPTIME\]/ || /^\s/ || /^$/) {print} p && !(/^\[UPTIME\]/ || /^\s/ || /^$/) {p=0}' "$LOG_FILE" > "$SUMMARY_FILE"
}

PIDS=()

for LOST in "${LOST_FOVS[@]}"; do
  for FOUND in "${FOUND_FOVS[@]}"; do
    run_sim "$LOST" "$FOUND" &
    PIDS+=("$!")
    # If we've hit the max, wait for the first to finish
    if (( ${#PIDS[@]} >= MAX_PROCS )); then
      wait "${PIDS[0]}"
      PIDS=("${PIDS[@]:1}")
    fi
  done
done

# Wait for all remaining jobs
for pid in "${PIDS[@]}"; do
  wait "$pid"
done

rm $SIM_DIR/*.log

echo "[DONE] All FOV combinations completed."
