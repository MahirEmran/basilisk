#!/usr/bin/env bash

set -euo pipefail

SIM_DIR="."

# Fixed camera FOVs
LOST_FOV=25
FOUND_FOV=75

# Exclusion buffer ranges for FOUND baffle trade study
FOUND_EXCLUSION_BUFFERS=(5 10 15 20 25 30)

MONTH_HOURS=744 # 31 days

MAX_PROCS=1

run_sim() {
  local FOUND_EXCLUSION_BUFFER="$1"
  local OUT_FILE="$SIM_DIR/output_LOST${LOST_FOV}_FOUND${FOUND_FOV}_FOUND_EXCL${FOUND_EXCLUSION_BUFFER}.bin"
  local PLOTS_DIR="$SIM_DIR/output_LOST${LOST_FOV}_FOUND${FOUND_FOV}_FOUND_EXCL${FOUND_EXCLUSION_BUFFER}_plots"
  local LOG_FILE="$SIM_DIR/output_LOST${LOST_FOV}_FOUND${FOUND_FOV}_FOUND_EXCL${FOUND_EXCLUSION_BUFFER}.log"
  local SUMMARY_FILE="$PLOTS_DIR/summary.txt"

  echo "[RUN] LOST_FOV=$LOST_FOV, FOUND_FOV=$FOUND_FOV, FOUND_EXCLUSION_BUFFER=$FOUND_EXCLUSION_BUFFER -> $OUT_FILE"

  "$(dirname "$0")/build.sh" \
    --lost-fov "$LOST_FOV" \
    --found-fov "$FOUND_FOV" \
    --found-exclusion-buffer "$FOUND_EXCLUSION_BUFFER" \
    --output "$OUT_FILE" \
    "$MONTH_HOURS" \
    > "$LOG_FILE" 2>&1

  rm -f "$OUT_FILE"
  rm -f $PLOTS_DIR/*.csv
  rm -f $PLOTS_DIR/*.png

}

PIDS=()

for FOUND_EXCLUSION_BUFFER in "${FOUND_EXCLUSION_BUFFERS[@]}"; do
  run_sim "$FOUND_EXCLUSION_BUFFER" &
  PIDS+=("$!")
  if (( ${#PIDS[@]} >= MAX_PROCS )); then
    wait "${PIDS[0]}"
    PIDS=("${PIDS[@]:1}")
  fi
done

# Wait for all remaining jobs
for pid in "${PIDS[@]}"; do
  wait "$pid"
done

# rm -f $SIM_DIR/*.log

echo "[DONE] All FOUND exclusion buffer combinations completed."
