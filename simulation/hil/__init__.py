"""
HIL (Hardware-in-the-Loop) Module for Basilisk Simulation

This module provides HIL functionality for integrating Basilisk simulations
with external hardware such as the BeagleBone Black.
"""

from .hil_orchestrator import HilOrchestrator, create_hil_orchestrator
from .hil_packet_codec import (
    HilPacketCodec,
    TruthNavPacket,
    EnvPacket,
    PowerThermalPacket,
    FcCommandPacket,
    HealthPacket,
    create_truth_nav_packet,
    create_fc_command_packet,
    create_health_packet
)
from .hil_transport_udp import HilTransportUDP, create_udp_transport
from .hil_transport_zmq import HilTransportZMQ, create_zmq_transport
from .hil_watchdog import HilWatchdog, WatchdogConfig, create_watchdog

__all__ = [
    'HilOrchestrator',
    'create_hil_orchestrator',
    'HilPacketCodec',
    'TruthNavPacket',
    'EnvPacket',
    'PowerThermalPacket',
    'FcCommandPacket',
    'HealthPacket',
    'create_truth_nav_packet',
    'create_fc_command_packet',
    'create_health_packet',
    'HilTransportUDP',
    'create_udp_transport',
    'HilTransportZMQ',
    'create_zmq_transport',
    'HilWatchdog',
    'WatchdogConfig',
    'create_watchdog'
]

__version__ = '1.0.0'