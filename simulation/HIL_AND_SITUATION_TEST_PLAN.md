# HIL Implementation and Situation Testing Plan

## Scope
This document defines:
- HIL implementation plan for the current HuskySat-style Basilisk simulation flow.
- Situation testing plan focused on mission-operational scenarios and edge conditions.

The goal is to validate guidance state behavior, timing robustness, and mission KPIs under realistic and stressed conditions.

## System Under Test
Primary runtime and interfaces:
- Simulation orchestrator: simulation/simulate_cubesat.py
- Guidance (Python backend): simulation/active_guidance.py
- Guidance (external C++ backend): simulation/External/ExternalModules/ActiveGuidance/activeGuidance.cpp
- Guidance math: simulation/guidance_math.py and simulation/External/ExternalModules/ActiveGuidance/activeGuidanceMath.cpp
- Uptime metrics: simulation/uptime_metrics.py
- Visualization overlays and status: simulation/vizard_scene.py

## Acceptance Criteria (Global)
- No uncommanded instability or uncontrolled mode chatter.
- Guidance transitions are explainable from threshold logic and visibility events.
- LOST and FOUND availability metrics remain inside expected envelope for each test profile.
- Battery state of charge and payload thermal values remain within defined mission constraints.
- C++ and Python guidance backends show equivalent mission behavior within approved tolerance.

## HIL Implementation Plan

### Objective
Introduce a practical HIL-ready architecture with minimal disruption to the current simulation and guidance logic.

### Architecture Target for BeagleBone Black + F'
HIL split:
1. Basilisk host process (Linux laptop or workstation): truth dynamics, environment, scenario control, KPI generation.
2. BeagleBone Black process: transport gateway and optional sensor and actuator proxy.
3. F' flight computer software: runs separately and owns flight state machine and control decisions.

Integration boundary rule:
- Basilisk does not embed F' logic.
- Basilisk exchanges packets with F' through a strict wire protocol adapter.
- Guidance in this repo remains available for comparison and fallback, not as the primary FC implementation during HIL with F'.

### Concrete Code Changes by Area

#### 1. Python Orchestrator Changes in simulation/simulate_cubesat.py
Add new command-line options:
1. --hil-enable [bool]
2. --hil-role [truth or proxy]
3. --hil-transport [udp or uart]
4. --hil-bind-ip [string]
5. --hil-bind-port [int]
6. --hil-peer-ip [string]
7. --hil-peer-port [int]
8. --hil-cycle-ms [int]
9. --hil-timeout-ms [int]
10. --hil-hold-last-max-ms [int]

Add new execution branch in main loop:
1. Current branch: pure simulation path stays unchanged.
2. New branch: HIL path reads nav and environment truth, serializes packets, sends to F' endpoint, receives command packet, applies command to Basilisk actuators.

Add telemetry logging fields:
1. seq_tx, seq_rx, dropped_frames, stale_frames.
2. loop_jitter_ms, loop_latency_ms, command_age_ms.
3. watchdog trips and auto-reconnect count.

Expected file additions under simulation/hil:
1. simulation/hil/hil_orchestrator.py
2. simulation/hil/hil_packet_codec.py
3. simulation/hil/hil_transport_udp.py
4. simulation/hil/hil_transport_uart.py
5. simulation/hil/hil_watchdog.py

#### 2. Basilisk Interface Changes in simulation/active_guidance.py and External module
Guidance role update for HIL mode:
1. Add mode HIL_EXTERNAL_FC to bypass local state selection output.
2. Keep producing diagnostic state estimates for comparison against F' decisions.
3. Publish comparison metrics between local expected state and commanded FC state.

Python guidance file changes:
1. Add optional external command input object consumed each cycle.
2. Add output status structure for mode mismatch diagnostics.

External C++ guidance module changes:
1. Update simulation/External/ExternalModules/ActiveGuidance/activeGuidance.h with a new optional FC command input message.
2. Update simulation/External/ExternalModules/ActiveGuidance/activeGuidance.cpp to support pass-through reference mode in HIL.
3. Update simulation/External/ExternalModules/ActiveGuidance/activeGuidance.i to expose new setters and state fields.
4. Add unit tests in simulation/External/ExternalModules/ActiveGuidance/_UnitTest/test_activeGuidance.py for HIL pass-through behavior.

#### 3. New Message Contract for F' Interop
Define packet groups:
1. TruthNavPacket: position, velocity, attitude, rate, timestamp, seq.
2. EnvPacket: sun vector, moon vector, eclipse flag, station visibility summary, timestamp, seq.
3. PowerThermalPacket: SOC, net power, payload temp, timestamp, seq.
4. FcCommandPacket: desired attitude reference or torque command, mode tag, command validity window, timestamp, seq.
5. HealthPacket: heartbeat, process uptime, watchdog flags.

Serialization recommendation for BBB:
1. Use compact binary framing for runtime transport.
2. Keep schema deterministic and fixed-width where possible.
3. Add crc32 and sequence ID to every frame.

#### 4. BeagleBone Black Specific Implementation
Runtime choices:
1. Prefer UDP over Ethernet for initial bring-up.
2. Add UART fallback for direct tether if network isolation is required.

BBB process responsibilities:
1. Receive Basilisk truth packets.
2. Forward to F' process endpoint.
3. Receive FcCommandPacket from F'.
4. Forward command packet back to Basilisk.
5. Enforce watchdog and timeout policy.

BBB timing and OS setup:
1. Use monotonic clock for scheduling and latency measurements.
2. Pin HIL gateway process to one CPU core when possible.
3. Raise process priority for gateway task.
4. Log clock offset and drift statistics between host and BBB.

BBB watchdog policy:
1. If no valid command for timeout window, command hold-last until hold-last-max.
2. If hold-last exceeds limit, switch to safe command profile.
3. Emit watchdog event and state transition reason code.

#### 5. F' Boundary Definition
F' ownership:
1. FC mode logic.
2. Attitude target or torque command generation.
3. Fault response logic.

Basilisk ownership:
1. Dynamics and environment truth.
2. Sensor and bus emulation.
3. KPI computation and campaign automation.

Interface control document tasks:
1. Define exact field units and scaling for all packets.
2. Define command validity semantics and timeout behavior.
3. Define startup handshake state machine.
4. Define error codes and reconnect behavior.

#### 6. Build and Repo Integration Tasks
Add source layout:
1. simulation/hil for Python gateway and transport code.
2. simulation/config/hil for profile files.
3. simulation/tests/hil for transport and integration tests.

Add configuration files:
1. simulation/config/hil/nominal_udp.yaml
2. simulation/config/hil/latency_sweep.yaml
3. simulation/config/hil/dropout_profile.yaml

Add automation scripts:
1. simulation/run_hil_campaign.py for profile-based execution.
2. simulation/tools/replay_hil_log.py for offline decode and analysis.

### Phase Plan with Explicit Implementation Output

### Phase 0: Baseline Freeze
Duration: 2 to 3 days

Implementation output:
1. Baseline artifacts from current pure-sim run.
2. Saved KPI envelopes used as HIL regression gates.

Code-level changes:
1. Add baseline artifact manifest writer to simulation/simulate_cubesat.py.

### Phase 1: Packet Contract and Adapter Layer
Duration: 1 week

Implementation output:
1. Packet codec and adapter interfaces.
2. Basilisk adapter implementation.

Code-level changes:
1. Add simulation/hil/hil_packet_codec.py.
2. Add simulation/hil/hil_orchestrator.py.
3. Wire adapter selection in simulation/simulate_cubesat.py.

### Phase 2: BBB Transport Bring-Up
Duration: 1 week

Implementation output:
1. UDP path host to BBB to F' to BBB to host.
2. Health and heartbeat support.

Code-level changes:
1. Add simulation/hil/hil_transport_udp.py.
2. Add basic heartbeat and timeout in simulation/hil/hil_watchdog.py.

### Phase 3: Real-Time Loop and Safety Policies
Duration: 1 week

Implementation output:
1. Deterministic cycle loop with jitter and overrun stats.
2. Hold-last and safe-mode fallback path.

Code-level changes:
1. Add HIL loop scheduler branch in simulation/simulate_cubesat.py.
2. Add command age checks before applying control outputs.

### Phase 4: External Guidance and Comparison Hooks
Duration: 1 week

Implementation output:
1. HIL pass-through support in Python and C++ guidance paths.
2. Mode mismatch and command quality diagnostics.

Code-level changes:
1. Update simulation/active_guidance.py for pass-through diagnostic mode.
2. Update simulation/External/ExternalModules/ActiveGuidance/activeGuidance.h.
3. Update simulation/External/ExternalModules/ActiveGuidance/activeGuidance.cpp.
4. Update simulation/External/ExternalModules/ActiveGuidance/activeGuidance.i.

### Phase 5: Campaign Automation and Reporting
Duration: 1 week

Implementation output:
1. Repeatable campaign runner and profile library.
2. Unified KPI summary including transport quality metrics.

Code-level changes:
1. Add simulation/run_hil_campaign.py.
2. Add profile YAML files under simulation/config/hil.
3. Extend CSV summary outputs from simulation/simulate_cubesat.py.

### Phase 6: Bench Readiness
Duration: 3 to 4 days

Implementation output:
1. Bring-up checklist for host, BBB, and F'.
2. Known limitations and risk log.

Code-level changes:
1. Add bench startup verification script simulation/tools/check_hil_stack.py.

### Implementation Risks and Mitigations
1. BeagleBone Black CPU saturation under high packet rates.
Mitigation: fixed packet cadence, compact binary frames, and process priority tuning.

2. Clock misalignment across host, BBB, and F'.
Mitigation: monotonic timestamps, periodic clock offset estimation, and command age filtering.

3. F' restart or link flap causing stale command replay.
Mitigation: sequence checks, timeout invalidation, watchdog safe-mode command.

4. Hidden logic divergence between local guidance and F' outputs.
Mitigation: side-by-side diagnostic mode and mismatch counters in every run summary.

### Definition of Done
1. Host and BBB exchange deterministic packet streams with CRC and sequence checks.
2. F' command path closes loop with timeout-safe fallback behavior.
3. Basilisk can run in pure-sim mode and HIL mode using the same scenario file.
4. Python and C++ guidance support HIL diagnostic pass-through paths.
5. Campaign tool produces KPI and transport-health reports for each profile.

## Situation Testing Plan

### Objectives
- Validate behavior under mission situations rather than only component faults.
- Confirm state-machine logic remains correct across operational context changes.
- Quantify mission KPI sensitivity to realistic environmental and ops variations.

### Situation Categories
- Illumination geometry changes.
- Ground-network visibility changes.
- Navigation quality degradation.
- Power and thermal stress windows.
- Boundary-threshold and hysteresis edge cases.

### Situation Test Matrix

| Test ID | Situation | Configuration | What to Verify | Pass Criteria |
|---|---|---|---|---|
| SIT-001 | Nominal LEO Day/Night | Default stations and default profile | Stable mode cycling and KPI baseline | Matches expected nominal envelope |
| SIT-002 | Prolonged Eclipse | Orbit phase/profile emphasizing eclipse | CHARGING prioritization and SOC resilience | No SOC safety violation |
| SIT-003 | Sun-Earth Boundary Crossing | Geometry near charge enter/exit thresholds | Hysteresis prevents rapid toggling | Transition count remains bounded |
| SIT-004 | Sparse Ground Coverage | Remove all but one station | DOWNLINK opportunities reduce gracefully | No invalid station state behavior |
| SIT-005 | Dense Ground Coverage | Add multiple well-spaced stations | Station selection and window logic | Correct active station labeling and dwell limits |
| SIT-006 | GNSS Degraded | Tight zenith half-angle plus dead-reckoning stress | GNSS_FIX scheduling correctness | Expected GNSS_FIX frequency and exits |
| SIT-007 | Payload High-Duty Mission | Increase payload power duty | Thermal and power feedback into operations | Temperature/SOC stay in allowable band |
| SIT-008 | Moon Keepout Stress | Geometry emphasizing lunar proximity effects | LOST keepout handling with moon in loop | LOST uptime degradation is explainable |
| SIT-009 | Edge-of-FOV Earth Visibility | Configure near FOUND keep-in boundary | FOUND validity logic near edge | No discontinuous behavior spikes |
| SIT-010 | Long-Horizon Drift Check | 744 h and 3000 h campaigns | KPI convergence and no long-run drift anomalies | Convergence trend and stable state proportions |

### Situation Test Procedure (Per Case)
1. Select situation profile and fixed random seed.
2. Run with Python guidance backend and record artifacts.
3. Run the identical profile with external C++ backend.
4. Compare mode timelines, transition counts, and KPI deltas.
5. Record verdict and attach CSV/plot artifacts.

### Situation Test Outputs
Per test case, archive:
- Run configuration and seed.
- Console transition log.
- Uptime CSV and generated plots.
- KPI summary table.
- Pass/fail decision with short rationale.

## Severity and Triage
- Critical: unsafe dynamic behavior, control divergence, or persistent invalid state.
- High: repeated incorrect transitions or major KPI regression.
- Medium: bounded KPI drift beyond planned tolerance.
- Low: visualization or non-critical telemetry inconsistency.

## Recommended Execution Cadence
- Per PR: fast subset (nominal + one stressed situation + backend equivalence quick check).
- Nightly: full situation suite up to medium duration.
- Weekly: long-horizon runs and implementation readiness checks.

## Minimal First Campaign
Run first if schedule is tight:
- Phase 0 baseline freeze and artifact capture.
- Phase 1 adapter abstraction with Basilisk message adapter.
- Phase 2 real-time loop with basic jitter and overrun telemetry.
- SIT-001, SIT-003, SIT-006, SIT-010 situation tests.

This campaign gives early confidence in implementation direction, timing behavior, threshold behavior, and long-horizon stability.
