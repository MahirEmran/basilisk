#!/usr/bin/env bash

set -euo pipefail

SIM_DIR="."

# FOV ranges for this test
LOST_FOVS=(20 25 30)
FOUND_FOVS=(60 65 70 75 80 85 90)

MONTH_HOURS=744 # 31 days

MAX_PROCS=2

run_sim() {
  local LOST="$1"
  local FOUND="$2"
  local OUT_FILE="$SIM_DIR/output_LOST${LOST}_FOUND${FOUND}.bin"
  local PLOTS_DIR="$SIM_DIR/output_LOST${LOST}_FOUND${FOUND}_plots"
  local LOG_FILE="$SIM_DIR/output_LOST${LOST}_FOUND${FOUND}.log"
  local SUMMARY_FILE="$PLOTS_DIR/summary.txt"

  echo "[RUN] LOST_FOV=$LOST, FOUND_FOV=$FOUND -> $OUT_FILE"

  "$(dirname "$0")/build.sh" \
    --lost-fov "$LOST" \
    --found-fov "$FOUND" \
    --output "$OUT_FILE" \
    "$MONTH_HOURS" \
    > "$LOG_FILE" 2>&1

  rm -f "$OUT_FILE"
  rm -f $PLOTS_DIR/*.csv
  rm -f $PLOTS_DIR/*.png

}

PIDS=()

for LOST in "${LOST_FOVS[@]}"; do
  for FOUND in "${FOUND_FOVS[@]}"; do
    run_sim "$LOST" "$FOUND" &
    PIDS+=("$!")
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

# rm -f $SIM_DIR/*.log

echo "[DONE] All FOV combinations completed."
