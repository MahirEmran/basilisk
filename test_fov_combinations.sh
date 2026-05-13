#!/usr/bin/env bash

set -euo pipefail

BSK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="${BSK_ROOT}/simulation"
PYTHON_BIN="${BSK_ROOT}/.venv/bin/python"

# Fixed camera FOVs
LOST_FOV=25
FOUND_FOV=75

# Exclusion buffer ranges for FOUND baffle trade study
FOUND_EXCLUSION_BUFFERS=(5 10 15 20 25 30 35 40 45 50 55 60 65 70 75 80 85 90)

START_EPOCH_UTC="2026-01-01T00:00:00.000Z"
LOOP_HOURS=2922 # [hr]
LOOP_COUNT=3 # [-]

MAX_PROCS=1

epoch_plus_hours() {
  local BASE_EPOCH="$1"
  local OFFSET_HOURS="$2"
  "${PYTHON_BIN}" - "$BASE_EPOCH" "$OFFSET_HOURS" <<'PY'
import sys
from datetime import datetime, timezone, timedelta

epoch = sys.argv[1]
offset_hours = float(sys.argv[2])
dt = datetime.fromisoformat(epoch.replace("Z", "+00:00"))
dt = dt + timedelta(hours=offset_hours)
print(dt.astimezone(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.000Z"))
PY
}

run_sim() {
  local FOUND_EXCLUSION_BUFFER="$1"
  local LOOP_INDEX="$2"
  local EPOCH_UTC="$3"
  local OUT_FILE="$SIM_DIR/output_LOST${LOST_FOV}_FOUND${FOUND_FOV}_FOUND_EXCL${FOUND_EXCLUSION_BUFFER}_LOOP${LOOP_INDEX}.bin"
  local PLOTS_DIR="$SIM_DIR/output_LOST${LOST_FOV}_FOUND${FOUND_FOV}_FOUND_EXCL${FOUND_EXCLUSION_BUFFER}_LOOP${LOOP_INDEX}_plots"
  local LOG_FILE="$SIM_DIR/output_LOST${LOST_FOV}_FOUND${FOUND_FOV}_FOUND_EXCL${FOUND_EXCLUSION_BUFFER}_LOOP${LOOP_INDEX}.log"

  echo "[RUN] LOOP=${LOOP_INDEX}/${LOOP_COUNT}, LOST_FOV=$LOST_FOV, FOUND_FOV=$FOUND_FOV, FOUND_EXCLUSION_BUFFER=$FOUND_EXCLUSION_BUFFER, EPOCH_UTC=$EPOCH_UTC -> $OUT_FILE"

  "$(dirname "$0")/build.sh" \
    --lost-fov "$LOST_FOV" \
    --found-fov "$FOUND_FOV" \
    --found-exclusion-buffer "$FOUND_EXCLUSION_BUFFER" \
    --epoch-utc "$EPOCH_UTC" \
    --output "$OUT_FILE" \
    "$LOOP_HOURS" \
    > "$LOG_FILE" 2>&1

  rm -f "$OUT_FILE"
  rm -f $PLOTS_DIR/*.csv
  rm -f $PLOTS_DIR/*.png

}

PIDS=()

for FOUND_EXCLUSION_BUFFER in "${FOUND_EXCLUSION_BUFFERS[@]}"; do
  (
    for ((loop_idx = 0; loop_idx < LOOP_COUNT; loop_idx++)); do
      offset_hours=$((loop_idx * LOOP_HOURS))
      epoch_utc="$(epoch_plus_hours "$START_EPOCH_UTC" "$offset_hours")"
      run_sim "$FOUND_EXCLUSION_BUFFER" "$((loop_idx + 1))" "$epoch_utc"
    done
  ) &
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
