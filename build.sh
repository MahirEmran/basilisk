#!/usr/bin/env bash
set -euo pipefail

BSK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${BSK_ROOT}/.venv/bin/python"
SIM_DIR="${BSK_ROOT}/simulation/"
EXTERNAL_DIR="${SIM_DIR}/External"
MODE="HYBRID"
ENABLE_LIVE_EPS_PLOTS=0
REBUILD=0
LOST_FOV=25.5
FOUND_FOV=75.6
LOST_EXCLUSION_BUFFER=10.0
FOUND_EXCLUSION_BUFFER=10.0
OUTPUT_PATH=""

usage() {
  cat <<'EOF'
Usage:
  ./build.sh test        Rebuild Basilisk, run a short smoke simulation, then remove generated output.
  ./build.sh <hours>     Rebuild Basilisk and run simulation for <hours> (examples: 24, 744).

Options:
  --rebuild              Full rebuild of Basilisk from scratch
  --enable-live-eps-plots Enable live EPS telemetry plotting during simulation (disabled by default for performance)
  --lost-fov <deg>       LOST camera full FOV in degrees (default: 25.5)
  --found-fov <deg>      FOUND camera full FOV in degrees (default: 75.6)
  --lost-exclusion-buffer <deg> LOST exclusion cone buffer in degrees added to half-FOV (default: 10.0)
  --found-exclusion-buffer <deg> FOUND exclusion cone buffer in degrees added to half-FOV (default: 10.0)
  --output <path>        Custom output .bin file path (default: simulation/output_<hours>h.bin)

Notes:
  - If no argument is provided, the default is 1 hour.
  - End-of-run uptime plots are generated automatically by default.
  - Live EPS telemetry plotting is disabled by default for performance. Use --enable-live-eps-plots to enable.
  - Full rebuild is disabled by default. Use --rebuild.
  - Outputs are written under simulation/ unless --output is specified.
EOF
}

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "Error: expected Basilisk virtualenv python at ${PYTHON_BIN}" >&2
  echo "Create it first, for example: python3 -m venv ${BSK_ROOT}/.venv" >&2
  exit 1
fi

if [[ ! -d "${EXTERNAL_DIR}" ]]; then
  echo "Error: external modules folder not found: ${EXTERNAL_DIR}" >&2
  exit 1
fi

POSITIONAL=()
while [[ $# -gt 0 ]]; do
  case "${1}" in
    --rebuild)
      REBUILD=1
      shift
      ;;
    --enable-live-eps-plots)
      ENABLE_LIVE_EPS_PLOTS=1
      shift
      ;;
    --lost-fov)
      LOST_FOV="$2"
      shift 2
      ;;
    --found-fov)
      FOUND_FOV="$2"
      shift 2
      ;;
    --lost-exclusion-buffer)
      LOST_EXCLUSION_BUFFER="$2"
      shift 2
      ;;
    --found-exclusion-buffer)
      FOUND_EXCLUSION_BUFFER="$2"
      shift 2
      ;;
    --output)
      OUTPUT_PATH="$2"
      shift 2
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      POSITIONAL+=("${1}")
      shift
      ;;
  esac
done

if (( ${#POSITIONAL[@]} > 1 )); then
  echo "Error: too many arguments. Use 'test' or a numeric hour value." >&2
  usage >&2
  exit 1
fi

ARG="${POSITIONAL[0]:-1}"
HOURS=""
RUN_KIND="normal"

case "${ARG}" in
  test)
    RUN_KIND="test"
    HOURS="0.01"
    ;;
  *)
    if [[ "${ARG}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
      HOURS="${ARG}"
    else
      echo "Error: invalid argument '${ARG}'. Use 'test' or a numeric hour value." >&2
      usage >&2
      exit 1
    fi
    ;;
esac

HOURS_TAG="${HOURS//./p}"
if [[ -n "${OUTPUT_PATH}" ]]; then
  SIM_BIN_PATH="${OUTPUT_PATH}"
else
  SIM_BIN_PATH="${SIM_DIR}/output_${HOURS_TAG}h.bin"
fi
PLOTS_DIR="${SIM_BIN_PATH%.bin}_plots"

if [[ "${RUN_KIND}" == "test" ]]; then
  cleanup_test_artifacts() {
    rm -f "${SIM_BIN_PATH}"
    rm -rf "${PLOTS_DIR}"
  }
  trap cleanup_test_artifacts EXIT
fi

if [[ "${REBUILD}" == "1" ]]; then
  echo "[REBUILD] Rebuilding Basilisk from scratch with external modules from: ${EXTERNAL_DIR}"
  "${PYTHON_BIN}" "${BSK_ROOT}/conanfile.py" --clean --pathToExternalModules "${EXTERNAL_DIR}"
fi


echo "[RUN] Running HuskySat simulation for ${HOURS} hour(s) in mode ${MODE}"
cd "${SIM_DIR}"
if [[ "${ENABLE_LIVE_EPS_PLOTS}" == "1" ]]; then
  PYTHONPATH="${BSK_ROOT}/dist3${PYTHONPATH:+:${PYTHONPATH}}" \
    "${PYTHON_BIN}" "${SIM_DIR}/simulate_cubesat.py" \
    --guidance-backend EXTERNAL_CPP \
    --mode "${MODE}" \
    --hours "${HOURS}" \
    --lost-fov "${LOST_FOV}" \
    --found-fov "${FOUND_FOV}" \
    --lost-exclusion-buffer "${LOST_EXCLUSION_BUFFER}" \
    --found-exclusion-buffer "${FOUND_EXCLUSION_BUFFER}" \
    --bin-path "${SIM_BIN_PATH}" \
    --enable-live-eps-plots
else
  PYTHONPATH="${BSK_ROOT}/dist3${PYTHONPATH:+:${PYTHONPATH}}" \
    "${PYTHON_BIN}" "${SIM_DIR}/simulate_cubesat.py" \
    --guidance-backend EXTERNAL_CPP \
    --mode "${MODE}" \
    --hours "${HOURS}" \
    --lost-fov "${LOST_FOV}" \
    --found-fov "${FOUND_FOV}" \
    --lost-exclusion-buffer "${LOST_EXCLUSION_BUFFER}" \
    --found-exclusion-buffer "${FOUND_EXCLUSION_BUFFER}" \
    --bin-path "${SIM_BIN_PATH}"
fi

if [[ "${RUN_KIND}" == "test" ]]; then
  echo "[DONE] Test run complete. Removed ${SIM_BIN_PATH} and ${PLOTS_DIR}."
else
  echo "[DONE] Output written to: ${SIM_BIN_PATH}"
fi
