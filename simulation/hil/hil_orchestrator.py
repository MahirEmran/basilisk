"""
HIL Orchestrator for Basilisk Simulation

This module provides the main orchestration for HIL operations,
integrating with the Basilisk simulation and managing communication
with the BeagleBone Black.
"""

import time
import threading
from typing import Optional, Dict, Any
import numpy as np

from .hil_packet_codec import (
    HilPacketCodec, FcCommandPacket,
    create_truth_nav_packet, create_health_packet
)
from .hil_transport_udp import create_udp_transport
from .hil_transport_zmq import create_zmq_transport
from .hil_watchdog import HilWatchdog, create_watchdog


class HilOrchestrator:
    """Orchestrator for HIL operations"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize HIL orchestrator

        Args:
            config: Configuration dictionary
        """
        self.config = config or {}
        self.enabled = self.config.get('hil_enable', False)
        self.role = self.config.get('hil_role', 'truth')  # 'truth' or 'proxy'

        # Transport configuration
        self.bind_ip = self.config.get('hil_bind_ip', '0.0.0.0')
        self.bind_port = self.config.get('hil_bind_port', 5555)
        self.peer_ip = self.config.get('hil_peer_ip', '127.0.0.1')
        self.peer_port = self.config.get('hil_peer_port', 5556)
        self.transport_type = self.config.get('hil_transport', 'udp')

        # Timing configuration
        self.cycle_ms = self.config.get('hil_cycle_ms', 10)  # 10ms = 100Hz
        self.timeout_ms = self.config.get('hil_timeout_ms', 5000)
        self.hold_last_max_ms = self.config.get('hil_hold_last_max_ms', 10000)

        # Components
        self.codec = HilPacketCodec()
        self.transport: Optional[object] = None
        self.watchdog: Optional[HilWatchdog] = None

        # State
        self.running = False
        self.orchestrator_thread: Optional[threading.Thread] = None
        self.last_command_time = 0.0
        self.last_command_packet: Optional[FcCommandPacket] = None
        self.hold_last_active = False
        self.hold_last_start_time = 0.0

        # Statistics
        self.stats = {
            'seq_tx': 0,
            'seq_rx': 0,
            'dropped_frames': 0,
            'stale_frames': 0,
            'loop_jitter_ms': [],
            'loop_latency_ms': [],
            'command_age_ms': [],
            'watchdog_trips': 0,
            'auto_reconnect_count': 0
        }

        # Callbacks
        self.command_callback: Optional[callable] = None

    def initialize(self) -> bool:
        """
        Initialize HIL orchestrator

        Returns:
            True if initialization successful, False otherwise
        """
        if not self.enabled:
            print("HIL orchestrator disabled")
            return True

        print(f"Initializing HIL orchestrator (role: {self.role})...")

        try:
            # Create transport
            if self.transport_type == 'zmq':
                self.transport = create_zmq_transport(
                    local_port=self.bind_port,
                    remote_address=self.peer_ip,
                    remote_port=self.peer_port,
                )
            else:
                self.transport = create_udp_transport(
                    local_port=self.bind_port,
                    remote_address=self.peer_ip,
                    remote_port=self.peer_port
                )

            # Set message callback
            self.transport.set_message_callback(self._handle_received_message)

            # Start transport
            if not self.transport.start():
                print("Failed to start transport")
                return False

            # Create watchdog
            self.watchdog = create_watchdog(self.timeout_ms)

            # Set watchdog callbacks
            self.watchdog.set_warning_callback(self._handle_watchdog_warning)
            self.watchdog.set_critical_callback(self._handle_watchdog_critical)
            self.watchdog.set_safe_mode_callback(self._handle_watchdog_safe_mode)

            # Start watchdog
            if not self.watchdog.start():
                print("Failed to start watchdog")
                return False

            print("HIL orchestrator initialized successfully")
            return True

        except Exception as e:
            print(f"Failed to initialize HIL orchestrator: {e}")
            return False

    def start(self) -> bool:
        """
        Start HIL orchestrator

        Returns:
            True if start successful, False otherwise
        """
        if not self.enabled:
            return True

        if self.running:
            return True

        print("Starting HIL orchestrator...")

        try:
            self.running = True

            # Start orchestrator thread
            self.orchestrator_thread = threading.Thread(
                target=self._orchestrator_loop,
                daemon=True
            )
            self.orchestrator_thread.start()

            print("HIL orchestrator started")
            return True

        except Exception as e:
            print(f"Failed to start HIL orchestrator: {e}")
            return False

    def stop(self):
        """Stop HIL orchestrator"""
        if not self.running:
            return

        print("Stopping HIL orchestrator...")

        self.running = False

        # Wait for orchestrator thread to finish
        if self.orchestrator_thread:
            self.orchestrator_thread.join(timeout=1.0)

        # Stop watchdog
        if self.watchdog:
            self.watchdog.stop()

        # Stop transport
        if self.transport:
            self.transport.stop()

        print("HIL orchestrator stopped")

    def send_truth_nav(self, position: np.ndarray, velocity: np.ndarray,
                      attitude: np.ndarray, rate: np.ndarray) -> bool:
        """
        Send truth navigation data to BBB

        Args:
            position: Position vector [x, y, z] in meters
            velocity: Velocity vector [vx, vy, vz] in m/s
            attitude: Attitude quaternion [q0, q1, q2, q3]
            rate: Angular rate vector [wx, wy, wz] in rad/s

        Returns:
            True if send successful, False otherwise
        """
        if not self.enabled or not self.running:
            return False

        try:
            # Create packet
            packet = create_truth_nav_packet(position, velocity, attitude, rate)
            packet.sequence = self.codec.get_next_sequence(0x01)

            # Encode packet
            data = self.codec.encode_truth_nav(packet)

            # Debug: print what we're sending
            print(f"[HIL TX] TruthNav seq={packet.sequence} pos={position} vel={velocity} attitude={attitude} rate={rate} len={len(data)}")

            # Send packet
            success = self.transport.send(data)

            if success:
                self.stats['seq_tx'] += 1
            else:
                self.stats['dropped_frames'] += 1

            return success

        except Exception as e:
            print(f"Failed to send truth nav: {e}")
            self.stats['dropped_frames'] += 1
            return False

    def get_command(self) -> Optional[FcCommandPacket]:
        """
        Get latest command from BBB

        Returns:
            Latest command packet or None if no valid command
        """
        if not self.enabled or not self.running:
            return None

        # Check if command is valid
        if self.last_command_packet is None:
            return None

        # Check command age
        current_time = time.time()
        command_age_ms = (current_time - self.last_command_time) * 1000

        if command_age_ms > self.timeout_ms:
            # Command is stale
            self.stats['stale_frames'] += 1

            # Check if we should activate hold-last
            if not self.hold_last_active:
                self.hold_last_active = True
                self.hold_last_start_time = current_time
                print("HIL: Hold-last mode activated")

            # Check if hold-last has exceeded maximum
            hold_last_duration_ms = (current_time - self.hold_last_start_time) * 1000
            if hold_last_duration_ms > self.hold_last_max_ms:
                print("HIL: Hold-last exceeded maximum, switching to safe mode")
                return None

            # Return last command (hold-last)
            return self.last_command_packet

        # Command is valid
        self.hold_last_active = False
        return self.last_command_packet

    def send_health(self, heartbeat: int, process_uptime: int,
                   watchdog_flags: int = 0) -> bool:
        """
        Send health status to BBB

        Args:
            heartbeat: Heartbeat counter
            process_uptime: Process uptime in nanoseconds
            watchdog_flags: Watchdog status flags

        Returns:
            True if send successful, False otherwise
        """
        if not self.enabled or not self.running:
            return False

        try:
            # Create packet
            packet = create_health_packet(heartbeat, process_uptime, watchdog_flags)
            packet.sequence = self.codec.get_next_sequence(0x05)

            # Encode packet
            data = self.codec.encode_health(packet)

            # Send packet
            return self.transport.send(data)

        except Exception as e:
            print(f"Failed to send health: {e}")
            return False

    def set_command_callback(self, callback: callable):
        """
        Set callback for received commands

        Args:
            callback: Function to call when command received
        """
        self.command_callback = callback

    def get_statistics(self) -> Dict[str, Any]:
        """
        Get HIL statistics

        Returns:
            Dictionary with statistics
        """
        stats = self.stats.copy()

        # Calculate averages
        if stats['loop_jitter_ms']:
            stats['avg_loop_jitter_ms'] = np.mean(stats['loop_jitter_ms'])
        else:
            stats['avg_loop_jitter_ms'] = 0.0

        if stats['loop_latency_ms']:
            stats['avg_loop_latency_ms'] = np.mean(stats['loop_latency_ms'])
        else:
            stats['avg_loop_latency_ms'] = 0.0

        if stats['command_age_ms']:
            stats['avg_command_age_ms'] = np.mean(stats['command_age_ms'])
        else:
            stats['avg_command_age_ms'] = 0.0

        # Add transport statistics
        if self.transport:
            stats['transport'] = self.transport.get_statistics()

        # Add watchdog status
        if self.watchdog:
            stats['watchdog'] = self.watchdog.get_status()

        return stats

    def reset_statistics(self):
        """Reset statistics"""
        self.stats = {
            'seq_tx': 0,
            'seq_rx': 0,
            'dropped_frames': 0,
            'stale_frames': 0,
            'loop_jitter_ms': [],
            'loop_latency_ms': [],
            'command_age_ms': [],
            'watchdog_trips': 0,
            'auto_reconnect_count': 0
        }

    def _handle_received_message(self, data: bytes, address: tuple):
        """Handle received message from transport"""
        try:
            # Try to decode as FC command packet
            packet = self.codec.decode_fc_command(data)

            if packet:
                self.last_command_packet = packet
                self.last_command_time = time.time()
                self.stats['seq_rx'] += 1

                # Feed watchdog
                if self.watchdog:
                    self.watchdog.feed()

                # Call callback if set
                if self.command_callback:
                    self.command_callback(packet)

        except Exception as e:
            print(f"Failed to handle received message: {e}")

    def _orchestrator_loop(self):
        """Main orchestrator loop"""
        while self.running:
            try:
                loop_start = time.time()

                # Process queued messages
                messages = self.transport.get_queued_messages()
                for data, addr in messages:
                    self._handle_received_message(data, addr)

                # Calculate loop timing
                loop_end = time.time()
                loop_time_ms = (loop_end - loop_start) * 1000

                # Update statistics
                self.stats['loop_latency_ms'].append(loop_time_ms)

                # Sleep for remaining cycle time
                sleep_time = max(0, (self.cycle_ms / 1000.0) - (loop_end - loop_start))
                time.sleep(sleep_time)

            except Exception as e:
                if self.running:
                    print(f"Orchestrator loop error: {e}")
                break

    def _handle_watchdog_warning(self):
        """Handle watchdog warning state"""
        print("HIL: Watchdog warning state")
        self.stats['watchdog_trips'] += 1

    def _handle_watchdog_critical(self):
        """Handle watchdog critical state"""
        print("HIL: Watchdog critical state")
        self.stats['watchdog_trips'] += 1

    def _handle_watchdog_safe_mode(self):
        """Handle watchdog safe mode activation"""
        print("HIL: Watchdog safe mode activated")
        self.stats['watchdog_trips'] += 1


# Convenience function for creating orchestrator
def create_hil_orchestrator(config: Optional[Dict[str, Any]] = None) -> HilOrchestrator:
    """
    Create HIL orchestrator with configuration

    Args:
        config: Configuration dictionary

    Returns:
        HilOrchestrator instance
    """
    return HilOrchestrator(config)
