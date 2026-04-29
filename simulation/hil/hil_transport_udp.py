"""
HIL Transport Layer for UDP Communication

This module provides UDP-based transport for HIL communication between
Basilisk and the BeagleBone Black.
"""

import socket
import threading
import time
from typing import Optional, Callable, Tuple
import queue


class HilTransportUDP:
    """UDP transport for HIL communication"""

    def __init__(self, bind_address: str = "0.0.0.0", bind_port: int = 5555,
                 peer_address: str = "127.0.0.1", peer_port: int = 5556):
        """
        Initialize UDP transport

        Args:
            bind_address: Local address to bind to
            bind_port: Local port to bind to
            peer_address: Peer address to send to
            peer_port: Peer port to send to
        """
        self.bind_address = bind_address
        self.bind_port = bind_port
        self.peer_address = peer_address
        self.peer_port = peer_port

        self.socket: Optional[socket.socket] = None
        self.running = False
        self.receive_thread: Optional[threading.Thread] = None
        self.message_queue = queue.Queue()
        self.message_callback: Optional[Callable] = None

        # Statistics
        self.messages_sent = 0
        self.messages_received = 0
        self.bytes_sent = 0
        self.bytes_received = 0
        self.send_errors = 0
        self.receive_errors = 0

    def start(self) -> bool:
        """Start the UDP transport"""
        if self.running:
            return True

        try:
            # Create UDP socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # Bind to local address
            self.socket.bind((self.bind_address, self.bind_port))

            # Set socket timeout for non-blocking receive
            self.socket.settimeout(0.1)  # 100ms timeout

            self.running = True

            # Start receive thread
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()

            print(f"UDP transport started on {self.bind_address}:{self.bind_port}")
            return True

        except Exception as e:
            print(f"Failed to start UDP transport: {e}")
            return False

    def stop(self):
        """Stop the UDP transport"""
        if not self.running:
            return

        self.running = False

        # Wait for receive thread to finish
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)

        # Close socket
        if self.socket:
            self.socket.close()
            self.socket = None

        print("UDP transport stopped")

    def send(self, data: bytes) -> bool:
        """
        Send data to peer

        Args:
            data: Data to send

        Returns:
            True if send successful, False otherwise
        """
        if not self.running or not self.socket:
            return False

        try:
            self.socket.sendto(data, (self.peer_address, self.peer_port))
            self.messages_sent += 1
            self.bytes_sent += len(data)
            return True

        except Exception as e:
            print(f"Send error: {e}")
            self.send_errors += 1
            return False

    def receive(self, timeout: float = 0.0) -> Optional[Tuple[bytes, Tuple[str, int]]]:
        """
        Receive data from peer (blocking with timeout)

        Args:
            timeout: Timeout in seconds (0 for non-blocking)

        Returns:
            Tuple of (data, (address, port)) or None if timeout
        """
        if not self.running or not self.socket:
            return None

        try:
            # Set timeout
            self.socket.settimeout(timeout)

            # Receive data
            data, addr = self.socket.recvfrom(65535)  # Max UDP packet size

            self.messages_received += 1
            self.bytes_received += len(data)

            return data, addr

        except socket.timeout:
            return None
        except Exception as e:
            print(f"Receive error: {e}")
            self.receive_errors += 1
            return None

    def set_message_callback(self, callback: Callable[[bytes, Tuple[str, int]], None]):
        """
        Set callback for received messages

        Args:
            callback: Function to call when message received (data, address)
        """
        self.message_callback = callback

    def get_queued_messages(self) -> list:
        """
        Get all queued messages from receive thread

        Returns:
            List of (data, address) tuples
        """
        messages = []
        while not self.message_queue.empty():
            try:
                messages.append(self.message_queue.get_nowait())
            except queue.Empty:
                break
        return messages

    def get_statistics(self) -> dict:
        """
        Get transport statistics

        Returns:
            Dictionary with statistics
        """
        return {
            'messages_sent': self.messages_sent,
            'messages_received': self.messages_received,
            'bytes_sent': self.bytes_sent,
            'bytes_received': self.bytes_received,
            'send_errors': self.send_errors,
            'receive_errors': self.receive_errors,
            'running': self.running
        }

    def reset_statistics(self):
        """Reset statistics counters"""
        self.messages_sent = 0
        self.messages_received = 0
        self.bytes_sent = 0
        self.bytes_received = 0
        self.send_errors = 0
        self.receive_errors = 0

    def _receive_loop(self):
        """Receive loop running in separate thread"""
        while self.running:
            try:
                # Receive data with timeout
                result = self.receive(timeout=0.1)
                if result is None:
                    continue
                data, addr = result

                if data:
                    # Call callback if set
                    if self.message_callback:
                        self.message_callback(data, addr)

                    # Also queue the message
                    self.message_queue.put((data, addr))

            except Exception as e:
                if self.running:
                    print(f"Receive loop error: {e}")
                break


class HilTransportBidirectional:
    """Bidirectional UDP transport for HIL communication"""

    def __init__(self, local_address: str = "0.0.0.0", local_port: int = 5555,
                 remote_address: str = "127.0.0.1", remote_port: int = 5556):
        """
        Initialize bidirectional UDP transport

        Args:
            local_address: Local address to bind to
            local_port: Local port to bind to
            remote_address: Remote address to send to
            remote_port: Remote port to send to
        """
        self.tx_transport = HilTransportUDP(
            bind_address=local_address,
            bind_port=local_port,
            peer_address=remote_address,
            peer_port=remote_port
        )

        self.rx_transport = HilTransportUDP(
            bind_address=local_address,
            bind_port=local_port + 1,  # Use different port for RX
            peer_address=remote_address,
            peer_port=remote_port + 1  # Expect responses on different port
        )

    def start(self) -> bool:
        """Start both transports"""
        tx_ok = self.tx_transport.start()
        rx_ok = self.rx_transport.start()
        return tx_ok and rx_ok

    def stop(self):
        """Stop both transports"""
        self.tx_transport.stop()
        self.rx_transport.stop()

    def send(self, data: bytes) -> bool:
        """Send data using TX transport"""
        return self.tx_transport.send(data)

    def receive(self, timeout: float = 0.0) -> Optional[Tuple[bytes, Tuple[str, int]]]:
        """Receive data using RX transport"""
        return self.rx_transport.receive(timeout)

    def set_message_callback(self, callback: Callable[[bytes, Tuple[str, int]], None]):
        """Set callback for received messages"""
        self.rx_transport.set_message_callback(callback)

    def get_statistics(self) -> dict:
        """Get combined statistics"""
        tx_stats = self.tx_transport.get_statistics()
        rx_stats = self.rx_transport.get_statistics()

        return {
            'tx': tx_stats,
            'rx': rx_stats,
            'total_messages_sent': tx_stats['messages_sent'],
            'total_messages_received': rx_stats['messages_received'],
            'total_bytes_sent': tx_stats['bytes_sent'],
            'total_bytes_received': rx_stats['bytes_received'],
            'total_errors': tx_stats['send_errors'] + rx_stats['receive_errors']
        }

    def reset_statistics(self):
        """Reset statistics for both transports"""
        self.tx_transport.reset_statistics()
        self.rx_transport.reset_statistics()


# Convenience function for creating transport
def create_udp_transport(local_port: int = 5555, remote_address: str = "127.0.0.1",
                       remote_port: int = 5556) -> HilTransportUDP:
    """
    Create UDP transport with default settings

    Args:
        local_port: Local port to bind to
        remote_address: Remote address to send to
        remote_port: Remote port to send to

    Returns:
        HilTransportUDP instance
    """
    return HilTransportUDP(
        bind_address="0.0.0.0",
        bind_port=local_port,
        peer_address=remote_address,
        peer_port=remote_port
    )