#!/usr/bin/env bash
set -euo pipefail

BSK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${BSK_ROOT}/.venv/bin/python"
SIM_DIR="${BSK_ROOT}/simulation/"
EXTERNAL_DIR="${SIM_DIR}/External"
MODE="HYBRID"
ENABLE_PLOTS=0
REBUILD=0

usage() {
  cat <<'EOF'
Usage:
  ./build.sh test        Rebuild Basilisk, run a short smoke simulation, then remove generated output.
  ./build.sh <hours>     Rebuild Basilisk and run simulation for <hours> (examples: 24, 744).

Notes:
  - If no argument is provided, the default is 1 hour.
  - Plot windows and saved plot outputs are disabled by default. Use --enable-plots.
  - Full rebuild is disabled by default. Use --rebuild.
  - Outputs are written under simulation/.
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
for arg in "$@"; do
  case "${arg}" in
    --rebuild)
      REBUILD=1
      ;;
    --enable-plots)
      ENABLE_PLOTS=1
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      POSITIONAL+=("${arg}")
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
SIM_BIN_PATH="${SIM_DIR}/output_${HOURS_TAG}h.bin"
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
if [[ "${ENABLE_PLOTS}" == "1" ]]; then
  PYTHONPATH="${BSK_ROOT}/dist3${PYTHONPATH:+:${PYTHONPATH}}" \
    "${PYTHON_BIN}" "${SIM_DIR}/simulate_cubesat.py" \
    --guidance-backend EXTERNAL_CPP \
    --mode "${MODE}" \
    --hours "${HOURS}" \
    --bin-path "${SIM_BIN_PATH}" \
    --enable-plots
else
  PYTHONPATH="${BSK_ROOT}/dist3${PYTHONPATH:+:${PYTHONPATH}}" \
    "${PYTHON_BIN}" "${SIM_DIR}/simulate_cubesat.py" \
    --guidance-backend EXTERNAL_CPP \
    --mode "${MODE}" \
    --hours "${HOURS}" \
    --bin-path "${SIM_BIN_PATH}"
fi

if [[ "${RUN_KIND}" == "test" ]]; then
  echo "[DONE] Test run complete. Removed ${SIM_BIN_PATH} and ${PLOTS_DIR}."
else
  echo "[DONE] Output written to: ${SIM_BIN_PATH}"
fi
