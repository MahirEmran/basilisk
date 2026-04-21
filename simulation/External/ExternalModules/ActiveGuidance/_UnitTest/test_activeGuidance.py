#
# ISC License
#
# Copyright (c) 2026, huskysat-camera-sim contributors
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

import numpy as np

from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.architecture import messaging
from Basilisk.ExternalModules import activeGuidance


def test_active_guidance_smoke():
    r"""Run a minimal simulation and verify the external guidance module writes a finite AttRef.

    This test checks that:

    - The module can be instantiated from ``Basilisk.ExternalModules``.
    - Required messages can be connected.
    - The module writes finite attitude reference MRPs after propagation.
    """
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("unitProcess")
    task_name = "unitTask"
    task_rate = macros.sec2nano(0.5)
    proc.addTask(sim.CreateNewTask(task_name, task_rate))

    module = activeGuidance.ActiveGuidance()
    module.ModelTag = "activeGuidance"
    module.setModeString("ROLL_ONLY")
    module.setLostExclHalfDeg(17.5)
    module.setStatusPeriodSec(10.0)
    module.setPosFound_B(0.05, 0.0, 0.105)
    module.setDefaultSunHat_N(1.0, 0.0, 0.0)
    sim.AddModelToTask(task_name, module)

    sc_state = messaging.SCStatesMsgPayload()
    sc_state.r_BN_N = [6771000.0, 0.0, 0.0]
    sc_state.v_BN_N = [0.0, 7660.0, 0.0]
    sc_state.r_CN_N = [6771000.0, 0.0, 0.0]
    sc_state.v_CN_N = [0.0, 7660.0, 0.0]
    sc_state.sigma_BN = [0.0, 0.0, 0.0]
    sc_state.omega_BN_B = [0.0, 0.0, 0.0]
    sc_msg = messaging.SCStatesMsg().write(sc_state)

    sun_state = messaging.SpicePlanetStateMsgPayload()
    sun_state.PositionVector = [1.496e11, 0.0, 0.0]
    sun_msg = messaging.SpicePlanetStateMsg().write(sun_state)

    moon_state = messaging.SpicePlanetStateMsgPayload()
    moon_state.PositionVector = [3.84e8, 0.0, 0.0]
    moon_msg = messaging.SpicePlanetStateMsg().write(moon_state)

    module.scStateInMsg.subscribeTo(sc_msg)
    module.sunStateInMsg.subscribeTo(sun_msg)
    module.moonStateInMsg.subscribeTo(moon_msg)

    ref_log = module.attRefOutMsg.recorder()
    sim.AddModelToTask(task_name, ref_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(2.0))
    sim.ExecuteSimulation()

    assert len(ref_log.sigma_RN) > 0
    sigma_last = np.array(ref_log.sigma_RN[-1], dtype=float)
    assert np.all(np.isfinite(sigma_last))
    assert module.getState() in ("CHARGING", "EXPERIMENT")
