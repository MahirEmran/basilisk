# Basilisk Module Integration Guide (External C/C++ Modules)

This guide is intentionally integration-first. It starts with "how to wire a working module" and moves testing/validation to the end.

The naming below uses **activeGuidance** as the canonical module name.

## 1. Integrate First: Minimum Path to a Working Module

If your goal is "make it run in sim today," follow this exact sequence.

## 1.1 Create the External Module Folder

Create this structure under `simulation/External`:

```text
simulation/External/
  ExternalModules/
    ActiveGuidance/
      activeGuidance.h
      activeGuidance.cpp
         activeGuidanceMath.h
         activeGuidanceMath.cpp
      activeGuidance.i
      activeGuidance.rst
      _UnitTest/
        test_activeGuidance.py
```

Use these references while authoring code:

- `src/moduleTemplates/cppModuleTemplate`
- existing external guidance module code under `simulation/External/ExternalModules`

## 1.2 Implement the C++ SysModel Shell

In `activeGuidance.h/.cpp`, implement at least:

1. Class deriving from `SysModel`.
2. Inputs: spacecraft state + Sun (and Moon if needed) as `ReadFunctor<>`.
3. Output: attitude reference as `Message<AttRefMsgPayload>`.
4. `Reset(uint64_t)`.
5. `UpdateState(uint64_t)`.

Recommended separation pattern:

1. Keep `activeGuidance.*` as a thin Basilisk wrapper (messages, mode selection, logging, output writes).
2. Move geometry/math-heavy code into helper files such as `activeGuidanceMath.h/.cpp`.
3. Keep the wrapper readable by delegating roll solves, vector operations, and targeting geometry to helper functions.

Concrete `UpdateState()` flow:

1. Guard input availability (`isLinked()`, `isWritten()`).
2. Read message payloads and compute your guidance/estimation/control outputs.
3. Populate output payload(s) from `zeroMsgPayload`.
4. Write output(s) with `this->moduleID` and `CurrentSimNanos`.

## 1.3 Write SWIG Interface

In `activeGuidance.i`:

1. `%module activeGuidance`
2. Include exception wrapper:

```swig
%include "architecture/utilities/bskException.swg"
%default_bsk_exception();
```

3. Put header include in `%{ ... %}` block.
4. Include `sys_model.i` and `activeGuidance.h`.
5. Include all payload headers used by your message fields.
6. Keep `protectAllClasses` block.

## 1.4 Build and Import Immediately

From repo root:

```bash
source .venv/bin/activate
python conanfile.py --clean --pathToExternalModules simulation/External
PYTHONPATH=dist3 python -c "from Basilisk.ExternalModules import activeGuidance; print('import ok')"
```

If the import works, your C++/SWIG wiring is valid.

## 1.5 Wire into Simulation Script

In your sim script (for example `simulation/simulate_cubesat.py`):

1. Import module:

```python
from Basilisk.ExternalModules import activeGuidance
```

2. Instantiate and configure:

```python
guidance = activeGuidance.ActiveGuidance()
guidance.ModelTag = "activeGuidance"
guidance.setModeString("HYBRID")
```

3. Subscribe inputs:

```python
guidance.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
guidance.sunStateInMsg.subscribeTo(sunEphemObject.planetOutMsgs[0])
```

4. Connect output downstream:

```python
attError.attRefInMsg.subscribeTo(guidance.attRefOutMsg)
```

5. Add to task:

```python
scSim.AddModelToTask(taskName, guidance)
```

## 2. Keep Algorithm Details in Module Documentation

This integration guide intentionally excludes module-specific algorithm math.

For algorithm explanations, add and maintain details in the module's own `.rst` file (for example `activeGuidance.rst`) alongside the implementation source.

## 3. Build Pipeline Details (How Import Works)

At build time:

1. Run:

```bash
python conanfile.py --clean --pathToExternalModules simulation/External
```

2. `conanfile.py` forwards path as `EXTERNAL_MODULES_PATH`.
3. `src/CMakeLists.txt` discovers SWIG targets under that path.
4. Generated Python wrappers are placed in `dist3/Basilisk/ExternalModules`.
5. Python imports from `Basilisk.ExternalModules.activeGuidance`.

## 4. Wrapping Existing Legacy Algorithms

Use Basilisk as an adapter shell rather than rewriting trusted legacy math.

Recommended split:

```text
ExternalModules/ActiveGuidance/
   activeGuidance.h/.cpp/.i          # Basilisk SysModel wrapper and message I/O
   activeGuidanceMath.h/.cpp         # Geometry and optimization math helpers
  legacyAdapter.h/.cpp
  legacyAlgo.h/.cpp
```

Adapter flow each tick:

1. Map Basilisk payloads to legacy input struct.
2. Call `legacy_step(...)`.
3. Map legacy outputs to Basilisk output payload.
4. Write message with `moduleID` and current sim time.

Critical discipline for every mapped field:

1. Unit (m, km, rad, deg, s).
2. Frame (body, inertial, sensor).
3. Direction convention (A->B versus B->A LOS).

## 5. Common Integration Failure Modes

1. Import failure.
   - Rebuild with correct `--pathToExternalModules`.
   - Ensure `PYTHONPATH=dist3`.
2. Module outputs identity forever.
   - Required input not linked/written.
   - Position magnitude invalid near zero.
3. SWIG compile errors.
   - Missing payload/header includes in `.i`.
   - `%module` mismatch with Python import name.
4. Logic is right standalone but wrong in simulation.
   - Frame/unit mismatch at adapter boundary.
   - Message timing/order mismatch between tasks.

## 6. LAST: Testing, Validation, and HIL Stress

## 6.1 Unit Test (Module-Level)

Create `_UnitTest/test_activeGuidance.py` with:

1. Module instantiation from `Basilisk.ExternalModules.activeGuidance`.
2. Input message creation and writes.
3. Input subscriptions.
4. Output recorders.
5. Short sim run.
6. Assertions on finite outputs and expected interface behavior.

## 6.2 Integration Smoke Test

```bash
source .venv/bin/activate
python conanfile.py --clean --pathToExternalModules simulation/External
PYTHONPATH=dist3 python -c "from Basilisk.ExternalModules import activeGuidance; print('import ok')"
cd simulation
PYTHONPATH=../dist3 python simulate_cubesat.py --guidance-backend EXTERNAL_CPP --mode HYBRID --hours 24 --bin-path ./output_24h.bin
```

## 6.3 HIL/Stress Recommendations

1. Add bridge modules for sensor in and actuator out.
2. Inject delay, jitter, and dropout before hardware-in-loop runs.
3. Extend `test_fov_combinations.sh` to sweep:
   - initial attitude/rates,
   - orbit/epoch,
   - sensor noise/bias,
   - transport fault parameters.

## 6.4 PR Checklist

1. Build passes with external path.
2. Module imports from `Basilisk.ExternalModules`.
3. Unit test exists and passes.
4. Module `.rst` docs exist.
5. Release note snippet added under `docs/source/Support/bskReleaseNotesSnippets/`.
6. If fixing a known issue, update `docs/source/Support/bskKnownIssues.rst`.

---

If you clone this pattern for a second module, copy the ActiveGuidance structure and replace only:

1. class/file/module names,
2. payload types and fields,
3. algorithm body in `UpdateState()`,
4. SWIG `%module` and include list,
5. unit test assertions.

