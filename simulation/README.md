# HuskySat Camera Simulation (In-Repo Copy)

This folder contains a HuskySat-oriented simulation flow plus a generalized pattern for integrating external Basilisk C/C++ modules.

## HuskySat Quick Start

From the Basilisk repository root:

```bash
source .venv/bin/activate
./build.sh test
```

To run for a specific duration (in hours), pass a number:

```bash
./build.sh 24
./build.sh 744
```

## HuskySat Manual Run

```bash
source .venv/bin/activate
python conanfile.py --clean --pathToExternalModules simulation/External
cd simulation
PYTHONPATH=../dist3 python simulate_cubesat.py --guidance-backend EXTERNAL_CPP --mode HYBRID --hours 24 --bin-path ./output_external.bin
```

Open Vizard and load the generated `.bin` file.

# Basilisk External Module Integration Guide (Generalized)

This guide is integration-first and module-agnostic. It applies to guidance, navigation, estimation, power, thermal, fault-management, payload processing, and other module types.

## 1. Integrate First: Minimum Path to a Working Module

If your goal is "make it run in sim today," follow this exact sequence.

## 1.1 Create the External Module Folder

Create this structure under `simulation/External`:

```text
simulation/External/
  ExternalModules/
    YourModule/
      yourModule.h
      yourModule.cpp
      yourModuleMath.h
      yourModuleMath.cpp
      yourModule.i
      yourModule.rst
      _UnitTest/
        test_yourModule.py
```

Use these references while authoring code:

- `src/moduleTemplates/cppModuleTemplate`
- existing external modules under `simulation/External/ExternalModules`

## 1.2 Implement the C++ SysModel Shell

In `yourModule.h/.cpp`, implement at least:

1. A class deriving from `SysModel`.
2. Required inputs as `ReadFunctor<>` fields.
3. Required outputs as `Message<>` fields.
4. `Reset(uint64_t)`.
5. `UpdateState(uint64_t)`.

Recommended separation pattern:

1. Keep `yourModule.*` as a thin Basilisk wrapper (messages, state handling, mode selection, logging, output writes).
2. Move math/geometry/optimization-heavy logic into helper files such as `yourModuleMath.h/.cpp`.
3. Keep wrapper code readable by delegating algorithm details to helper functions.

Concrete `UpdateState()` flow:

1. Guard input availability (`isLinked()`, `isWritten()`).
2. Read payloads and assemble algorithm inputs.
3. Call helper/algorithm functions.
4. Populate output payload(s) from `zeroMsgPayload`.
5. Write output(s) with `this->moduleID` and `CurrentSimNanos`.

## 1.3 Write SWIG Interface

In `yourModule.i`:

1. Declare `%module yourModule`.
2. Include exception wrapper:

```swig
%include "architecture/utilities/bskException.swg"
%default_bsk_exception();
```

3. Put header include in `%{ ... %}` block.
4. Include `sys_model.i` and your header.
5. Include all payload headers used by your message fields.
6. Keep `protectAllClasses` block.

## 1.4 Build and Import Immediately

From repo root:

```bash
source .venv/bin/activate
python conanfile.py --clean --pathToExternalModules simulation/External
PYTHONPATH=dist3 python -c "from Basilisk.ExternalModules import yourModule; print('import ok')"
```

If import succeeds, your C++/SWIG wiring is valid.

## 1.5 Wire into Simulation Script

In your simulation script:

1. Import module:

```python
from Basilisk.ExternalModules import yourModule
```

2. Instantiate/configure:

```python
module = yourModule.YourModule()
module.ModelTag = "yourModule"
```

3. Subscribe inputs.
4. Connect outputs to downstream modules.
5. Add module to a task with `AddModelToTask`.

## 2. Keep Algorithm Details in Module Documentation

This README focuses on integration mechanics.

For algorithm-specific theory, assumptions, and equations, document details in `yourModule.rst` next to implementation code.

## 3. Build Pipeline Details (How Import Works)

At build time:

1. Run:

```bash
python conanfile.py --clean --pathToExternalModules simulation/External
```

2. `conanfile.py` forwards `EXTERNAL_MODULES_PATH`.
3. `src/CMakeLists.txt` discovers SWIG targets under that path.
4. Generated wrappers are placed in `dist3/Basilisk/ExternalModules`.
5. Python imports from `Basilisk.ExternalModules`.

## 4. Wrapping Existing Legacy Algorithms

Use Basilisk as an adapter shell rather than rewriting trusted legacy logic.

Recommended split:

```text
ExternalModules/YourModule/
  yourModule.h/.cpp/.i             # Basilisk SysModel wrapper and message I/O
  yourModuleMath.h/.cpp            # Math/geometry/helper logic (optional)
  legacyAdapter.h/.cpp             # Payload/struct marshaling (optional)
  legacyAlgo.h/.cpp                # Legacy/core algorithm (optional)
```

Adapter flow each tick:

1. Map Basilisk payloads to algorithm input structs.
2. Call algorithm step/update.
3. Map algorithm output back to Basilisk payloads.
4. Write output messages.

Critical discipline for every mapped field:

1. Units (m, km, rad, deg, s, etc.).
2. Frames (body, inertial, sensor, local-level, etc.).
3. Direction conventions (A->B versus B->A).

## 5. Common Integration Failure Modes

1. Import failure.
   - Rebuild with correct `--pathToExternalModules`.
   - Ensure `PYTHONPATH=dist3`.
2. Module outputs never update.
   - Required inputs not linked/written.
   - Module not added to intended task.
3. SWIG compile errors.
   - Missing includes in `.i`.
   - `%module` mismatch with import name.
4. Works in isolation but fails in simulation.
   - Frame/unit mismatch at adapter boundaries.
   - Task timing/order mismatch.

## 6. LAST: Testing

## 6.1 Unit Test (Module-Level)

Create `_UnitTest/test_yourModule.py` with:

1. Module instantiation from `Basilisk.ExternalModules`.
2. Input message creation and writes.
3. Input subscriptions.
4. Output recorders.
5. Short simulation run.
6. Assertions on finite outputs and expected interface behavior.

## 6.2 Integration Smoke Test

```bash
source .venv/bin/activate
python conanfile.py --clean --pathToExternalModules simulation/External
PYTHONPATH=dist3 python -c "from Basilisk.ExternalModules import yourModule; print('import ok')"
```

## 6.3 Multi-Module HIL and Stress Testing

Use module composition to switch between SIL, hybrid, and full HIL without changing core flight logic modules.

Recommended module stack:

1. Sensor-in bridge module: hardware/simulator packets to Basilisk messages.
2. Preprocess module: calibration, unit conversion, timestamp normalization.
3. Estimation module: state reconstruction.
4. Guidance/planning module: target generation (if applicable).
5. Control/allocation module: command generation.
6. Actuator-out bridge module: Basilisk commands to hardware/simulator packets.
7. Monitor/FDIR module: limits, health flags, fallback mode triggers.
8. Telemetry/KPI module: latency, dropouts, saturation, performance metrics.

Execution profiles:

1. SIL: simulated sensor and simulated actuator bridges.
2. Hybrid-in: real sensor bridge, simulated actuator bridge.
3. Hybrid-out: simulated sensor bridge, real actuator bridge.
4. Full HIL: real sensor and actuator bridges.
5. Shadow mode: run candidate module in parallel with baseline and compare outputs online.

Tasking pattern:

1. Fast I/O task: bridges at hardware cadence.
2. Mid-rate control task: estimation/guidance/control modules.
3. Slow supervision task: FDIR, mode management, health logging.

Stress injection modules:

1. Delay/jitter module: transport timing effects.
2. Drop/corrupt module: packet loss and corruption.
3. Noise/bias/drift module: sensor imperfections.
4. Saturation/rate-limit module: actuator constraints.

Campaign matrix:

1. Profile axis: SIL, hybrid-in, hybrid-out, full HIL.
2. Fault axis: none, delay, dropout, corruption, saturation.
3. Environment axis: nominal and worst-case scenario sets.
4. Duration axis: short regression, medium scenario tests, long soak.

Keep message interfaces stable across all profiles so test campaigns swap bridge/fault modules only, not flight logic wiring.

---

If you create another module, clone this structure and replace only:

1. class/file/module names,
2. payload types and fields,
3. algorithm internals,
4. SWIG `%module` and include list,
5. unit test assertions.



