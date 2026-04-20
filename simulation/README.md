# HuskySat Camera Simulation (In-Repo Copy)

This folder contains a copy of the HuskySat camera simulation and external C++ guidance module wired for a
from-source Basilisk build.

## Quick Start

From the Basilisk repository root:

```bash
source .venv/bin/activate
./build.sh test
```

`./build.sh test` will:

1. Rebuild Basilisk with `--pathToExternalModules simulation/huskysat_camera_sim/External`
2. Run a short simulation using the external C++ guidance backend
3. Clean up the generated `.bin` file and plots folder after the run

To run for a specific duration (in hours), pass a number:

```bash
./build.sh 24
./build.sh 744
```

## Manual Run

```bash
source .venv/bin/activate
python conanfile.py --clean --pathToExternalModules simulation/huskysat_camera_sim/External
cd simulation/huskysat_camera_sim
PYTHONPATH=../../dist3 python simulate_cubesat.py --guidance-backend EXTERNAL_CPP --mode HYBRID --hours 24 --bin-path ./output_external.bin
```

Open Vizard and load the generated `.bin` file.
