"""
HIL Watchdog for Safety and Monitoring

This module provides watchdog functionality for HIL operations,
including timeout detection, safe mode activation, and health monitoring.
"""

import time
import threading
from typing import Optional, Callable
from enum import Enum
from dataclasses import dataclass


class WatchdogState(Enum):
    """Watchdog state"""
    HEALTHY = "healthy"
    WARNING = "warning"
    CRITICAL = "critical"
    SAFE_MODE = "safe_mode"


@dataclass
class WatchdogConfig:
    """Watchdog configuration"""
    timeout_ms: int = 5000  # Timeout in milliseconds
    hold_last_max_ms: int = 10000  # Maximum hold-last duration
    warning_threshold: int = 3  # Number of timeouts before warning
    critical_threshold: int = 5  # Number of timeouts before critical
    safe_mode_threshold: int = 10  # Number of timeouts before safe mode
    heartbeat_interval_ms: int = 1000  # Heartbeat interval


@dataclass
class WatchdogStatus:
    """Watchdog status"""
    state: WatchdogState
    last_heartbeat_time: float
    timeout_count: int
    total_timeouts: int
    safe_mode_active: bool
    uptime_seconds: float


class HilWatchdog:
    """Watchdog for HIL operations"""

    def __init__(self, config: Optional[WatchdogConfig] = None):
        """
        Initialize watchdog

        Args:
            config: Watchdog configuration (uses defaults if None)
        """
        self.config = config or WatchdogConfig()
        self.state = WatchdogState.HEALTHY
        self.last_heartbeat_time = time.time()
        self.timeout_count = 0
        self.total_timeouts = 0
        self.safe_mode_active = False
        self.start_time = time.time()
        self.heartbeat_counter = 0

        self.running = False
        self.watchdog_thread: Optional[threading.Thread] = None

        # Callbacks
        self.warning_callback: Optional[Callable] = None
        self.critical_callback: Optional[Callable] = None
        self.safe_mode_callback: Optional[Callable] = None
        self.heartbeat_callback: Optional[Callable] = None

    def start(self) -> bool:
        """Start watchdog monitoring"""
        if self.running:
            return True

        try:
            self.running = True
            self.start_time = time.time()
            self.last_heartbeat_time = time.time()

            # Start watchdog thread
            self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
            self.watchdog_thread.start()

            print("Watchdog started")
            return True

        except Exception as e:
            print(f"Failed to start watchdog: {e}")
            return False

    def stop(self):
        """Stop watchdog monitoring"""
        if not self.running:
            return

        self.running = False

        # Wait for watchdog thread to finish
        if self.watchdog_thread:
            self.watchdog_thread.join(timeout=1.0)

        print("Watchdog stopped")

    def feed(self):
        """Feed the watchdog (called when heartbeat received)"""
        self.last_heartbeat_time = time.time()
        self.timeout_count = 0
        self.heartbeat_counter += 1

        # Update state if we were in warning/critical state
        if self.state in [WatchdogState.WARNING, WatchdogState.CRITICAL]:
            self.state = WatchdogState.HEALTHY
            print("Watchdog: Returned to healthy state")

    def get_status(self) -> WatchdogStatus:
        """
        Get current watchdog status

        Returns:
            WatchdogStatus with current state
        """
        return WatchdogStatus(
            state=self.state,
            last_heartbeat_time=self.last_heartbeat_time,
            timeout_count=self.timeout_count,
            total_timeouts=self.total_timeouts,
            safe_mode_active=self.safe_mode_active,
            uptime_seconds=time.time() - self.start_time
        )

    def set_warning_callback(self, callback: Callable):
        """Set callback for warning state"""
        self.warning_callback = callback

    def set_critical_callback(self, callback: Callable):
        """Set callback for critical state"""
        self.critical_callback = callback

    def set_safe_mode_callback(self, callback: Callable):
        """Set callback for safe mode activation"""
        self.safe_mode_callback = callback

    def set_heartbeat_callback(self, callback: Callable):
        """Set callback for heartbeat events"""
        self.heartbeat_callback = callback

    def reset(self):
        """Reset watchdog state"""
        self.state = WatchdogState.HEALTHY
        self.timeout_count = 0
        self.safe_mode_active = False
        self.last_heartbeat_time = time.time()
        print("Watchdog: State reset")

    def activate_safe_mode(self):
        """Activate safe mode"""
        if not self.safe_mode_active:
            self.safe_mode_active = True
            self.state = WatchdogState.SAFE_MODE
            print("Watchdog: Safe mode activated")

            if self.safe_mode_callback:
                self.safe_mode_callback()

    def deactivate_safe_mode(self):
        """Deactivate safe mode"""
        if self.safe_mode_active:
            self.safe_mode_active = False
            self.state = WatchdogState.HEALTHY
            self.timeout_count = 0
            print("Watchdog: Safe mode deactivated")

    def _watchdog_loop(self):
        """Watchdog monitoring loop"""
        while self.running:
            try:
                current_time = time.time()
                time_since_heartbeat = (current_time - self.last_heartbeat_time) * 1000  # Convert to ms

                # Check for timeout
                if time_since_heartbeat > self.config.timeout_ms:
                    self.timeout_count += 1
                    self.total_timeouts += 1

                    print(f"Watchdog: Timeout detected (count: {self.timeout_count}, "
                          f"time since heartbeat: {time_since_heartbeat:.1f}ms)")

                    # Update state based on timeout count
                    if self.timeout_count >= self.config.safe_mode_threshold:
                        self.activate_safe_mode()
                    elif self.timeout_count >= self.config.critical_threshold:
                        self.state = WatchdogState.CRITICAL
                        if self.critical_callback:
                            self.critical_callback()
                    elif self.timeout_count >= self.config.warning_threshold:
                        self.state = WatchdogState.WARNING
                        if self.warning_callback:
                            self.warning_callback()

                # Send heartbeat callback
                if self.heartbeat_callback:
                    self.heartbeat_callback(self.heartbeat_counter)

                # Sleep for heartbeat interval
                time.sleep(self.config.heartbeat_interval_ms / 1000.0)

            except Exception as e:
                if self.running:
                    print(f"Watchdog loop error: {e}")
                break


class CommandWatchdog:
    """Watchdog for monitoring command validity and age"""

    def __init__(self, max_age_ms: int = 5000):
        """
        Initialize command watchdog

        Args:
            max_age_ms: Maximum command age in milliseconds
        """
        self.max_age_ms = max_age_ms
        self.last_command_time = 0.0
        self.last_command_timestamp = 0
        self.command_count = 0
        self.stale_command_count = 0

    def update_command(self, command_timestamp: int) -> bool:
        """
        Update with new command

        Args:
            command_timestamp: Command timestamp in nanoseconds

        Returns:
            True if command is valid, False if stale
        """
        current_time = time.time()
        command_age_ms = (current_time - self.last_command_time) * 1000

        # Check if command is stale
        if command_age_ms > self.max_age_ms:
            self.stale_command_count += 1
            print(f"Command watchdog: Stale command detected (age: {command_age_ms:.1f}ms)")
            return False

        self.last_command_time = current_time
        self.last_command_timestamp = command_timestamp
        self.command_count += 1
        return True

    def is_command_valid(self) -> bool:
        """
        Check if current command is valid

        Returns:
            True if command is valid, False otherwise
        """
        current_time = time.time()
        command_age_ms = (current_time - self.last_command_time) * 1000
        return command_age_ms <= self.max_age_ms

    def get_command_age_ms(self) -> float:
        """
        Get age of last command

        Returns:
            Command age in milliseconds
        """
        current_time = time.time()
        return (current_time - self.last_command_time) * 1000

    def get_statistics(self) -> dict:
        """
        Get command watchdog statistics

        Returns:
            Dictionary with statistics
        """
        return {
            'command_count': self.command_count,
            'stale_command_count': self.stale_command_count,
            'last_command_timestamp': self.last_command_timestamp,
            'command_age_ms': self.get_command_age_ms(),
            'command_valid': self.is_command_valid()
        }

    def reset(self):
        """Reset command watchdog state"""
        self.last_command_time = 0.0
        self.last_command_timestamp = 0
        self.command_count = 0
        self.stale_command_count = 0


# Convenience function for creating watchdog
def create_watchdog(timeout_ms: int = 5000) -> HilWatchdog:
    """
    Create watchdog with default settings

    Args:
        timeout_ms: Timeout in milliseconds

    Returns:
        HilWatchdog instance
    """
    config = WatchdogConfig(timeout_ms=timeout_ms)
    return HilWatchdog(config)