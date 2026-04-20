HuskyActiveGuidance
===================

Executive Summary
-----------------

This external C++ Basilisk module recreates the active guidance behavior from the
Python HuskySat simulation. It publishes an AttRef output and supports the same
high-level modes:

- ``ROLL_ONLY``
- ``EXPERIMENT``
- ``HYBRID``

Message Connection Descriptions
-------------------------------

.. list-table:: Module I/O Messages
   :widths: 30 25 45
   :header-rows: 1

   * - Msg Variable Name
     - Msg Type
     - Description
   * - ``scStateInMsg``
     - :ref:`SCStatesMsgPayload`
     - Required spacecraft state input.
   * - ``sunStateInMsg``
     - :ref:`SpicePlanetStateMsgPayload`
     - Optional Sun SPICE state input. Falls back to a configured default vector when unavailable.
   * - ``moonStateInMsg``
     - :ref:`SpicePlanetStateMsgPayload`
     - Optional Moon SPICE state input used as an additional LOST keep-out body.
   * - ``attRefOutMsg``
     - :ref:`AttRefMsgPayload`
     - Output attitude reference message used by tracking controllers.

Configuration Notes
-------------------

The module is configured through setter methods exposed in SWIG:

- ``setModeString()``
- ``setLostExclHalfDeg()``
- ``setStatusPeriodSec()``
- ``setPosFound_B()``
- ``setDefaultSunHat_N()``

The current state machine mode is available through ``getState()``.
