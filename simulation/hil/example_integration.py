"""
HIL Integration Example for Basilisk Simulation

This example demonstrates how to integrate the HIL orchestrator
with the existing Basilisk simulation for Hardware-in-the-Loop testing.
"""

import numpy as np
import time
from typing import Dict, Any

# Import HIL components
from hil import (
    HilOrchestrator,
    create_hil_orchestrator,
    create_truth_nav_packet,
    create_fc_command_packet
)


class HilSimulationIntegration:
    """Integration of HIL with Basilisk simulation"""

    def __init__(self, sim_config: Dict[str, Any]):
        """
        Initialize HIL simulation integration

        Args:
            sim_config: Simulation configuration dictionary
        """
        self.sim_config = sim_config

        # Extract HIL configuration
        hil_config = {
            'hil_enable': sim_config.get('hil_enable', False),
            'hil_role': sim_config.get('hil_role', 'truth'),
            'hil_bind_ip': sim_config.get('hil_bind_ip', '0.0.0.0'),
            'hil_bind_port': sim_config.get('hil_bind_port', 5555),
            'hil_peer_ip': sim_config.get('hil_peer_ip', '127.0.0.1'),
            'hil_peer_port': sim_config.get('hil_peer_port', 5556),
            'hil_cycle_ms': sim_config.get('hil_cycle_ms', 10),
            'hil_timeout_ms': sim_config.get('hil_timeout_ms', 5000),
            'hil_hold_last_max_ms': sim_config.get('hil_hold_last_max_ms', 10000)
        }

        # Create HIL orchestrator
        self.hil_orchestrator = create_hil_orchestrator(hil_config)

        # Set command callback
        self.hil_orchestrator.set_command_callback(self._handle_command)

        # State
        self.last_command = None
        self.command_count = 0

    def initialize(self) -> bool:
        """
        Initialize HIL integration

        Returns:
            True if initialization successful, False otherwise
        """
        print("Initializing HIL integration...")

        if not self.hil_orchestrator.initialize():
            print("Failed to initialize HIL orchestrator")
            return False

        if not self.hil_orchestrator.start():
            print("Failed to start HIL orchestrator")
            return False

        print("HIL integration initialized successfully")
        return True

    def shutdown(self):
        """Shutdown HIL integration"""
        print("Shutting down HIL integration...")
        self.hil_orchestrator.stop()
        print("HIL integration shutdown complete")

    def update_simulation_step(self, sc_state: Dict[str, np.ndarray]) -> bool:
        """
        Update simulation step with HIL integration

        Args:
            sc_state: Spacecraft state dictionary containing:
                - position: [x, y, z] in meters
                - velocity: [vx, vy, vz] in m/s
                - attitude: quaternion [q0, q1, q2, q3]
                - rate: [wx, wy, wz] in rad/s

        Returns:
            True if update successful, False otherwise
        """
        # Send truth navigation data to BBB
        success = self.hil_orchestrator.send_truth_nav(
            position=sc_state['position'],
            velocity=sc_state['velocity'],
            attitude=sc_state['attitude'],
            rate=sc_state['rate']
        )

        return success

    def get_external_command(self) -> Dict[str, Any]:
        """
        Get external command from BBB

        Returns:
            Command dictionary or None if no valid command
        """
        command_packet = self.hil_orchestrator.get_command()

        if command_packet is None:
            return None

        # Convert command packet to command dictionary
        command = {
            'timestamp': command_packet.timestamp,
            'type': command_packet.command_type,
            'data': command_packet.command_data,
            'validity_window': command_packet.validity_window,
            'sequence': command_packet.sequence
        }

        return command

    def send_health_status(self, heartbeat: int, uptime_ns: int):
        """
        Send health status to BBB

        Args:
            heartbeat: Heartbeat counter
            uptime_ns: Process uptime in nanoseconds
        """
        self.hil_orchestrator.send_health(heartbeat, uptime_ns)

    def get_statistics(self) -> Dict[str, Any]:
        """
        Get HIL statistics

        Returns:
            Dictionary with HIL statistics
        """
        stats = self.hil_orchestrator.get_statistics()
        stats['command_count'] = self.command_count
        return stats

    def _handle_command(self, command_packet):
        """
        Handle received command from BBB

        Args:
            command_packet: Received command packet
        """
        self.last_command = command_packet
        self.command_count += 1

        print(f"Received command: type={command_packet.command_type}, "
              f"seq={command_packet.sequence}")


def example_hil_integration():
    """Example of HIL integration with simulation"""

    # Simulation configuration
    sim_config = {
        'hil_enable': True,
        'hil_role': 'truth',
        'hil_bind_ip': '0.0.0.0',
        'hil_bind_port': 5555,
        'hil_peer_ip': '127.0.0.1',
        'hil_peer_port': 5556,
        'hil_cycle_ms': 10,
        'hil_timeout_ms': 5000,
        'hil_hold_last_max_ms': 10000
    }

    # Create HIL integration
    hil_integration = HilSimulationIntegration(sim_config)

    # Initialize
    if not hil_integration.initialize():
        print("Failed to initialize HIL integration")
        return

    try:
        # Simulate spacecraft state
        sc_state = {
            'position': np.array([7000e3, 0.0, 0.0]),  # 7000 km orbit
            'velocity': np.array([0.0, 7500.0, 0.0]),  # ~7.5 km/s
            'attitude': np.array([1.0, 0.0, 0.0, 0.0]),  # Identity quaternion
            'rate': np.array([0.0, 0.0, 0.0])  # Zero rate
        }

        # Simulation loop
        print("Starting simulation loop...")
        heartbeat = 0
        start_time = time.time()

        for i in range(1000):  # Run for 1000 iterations
            # Update simulation step
            hil_integration.update_simulation_step(sc_state)

            # Get external command
            command = hil_integration.get_external_command()
            if command:
                print(f"Iteration {i}: Received command {command['type']}")

            # Send health status every 10 iterations
            if i % 10 == 0:
                uptime_ns = int((time.time() - start_time) * 1e9)
                hil_integration.send_health_status(heartbeat, uptime_ns)
                heartbeat += 1

            # Simulate some dynamics
            sc_state['position'] += sc_state['velocity'] * 0.01  # 10ms timestep

            # Sleep for cycle time
            time.sleep(0.01)

        # Print statistics
        stats = hil_integration.get_statistics()
        print("\nHIL Statistics:")
        print(f"  Messages sent: {stats['seq_tx']}")
        print(f"  Messages received: {stats['seq_rx']}")
        print(f"  Dropped frames: {stats['dropped_frames']}")
        print(f"  Stale frames: {stats['stale_frames']}")
        print(f"  Average loop latency: {stats['avg_loop_latency_ms']:.2f} ms")
        print(f"  Commands received: {stats['command_count']}")

    finally:
        # Shutdown
        hil_integration.shutdown()


if __name__ == '__main__':
    example_hil_integration()