"""
HIL Transport Layer for ZeroMQ Communication

This module provides ZeroMQ PUB/SUB transport for HIL communication between
Basilisk and the BeagleBone Black.
"""

import queue
import threading
from typing import Optional, Callable, Tuple


class HilTransportZMQ:
    """ZeroMQ PUB/SUB transport for HIL communication."""

    def __init__(
        self,
        bind_address: str = "0.0.0.0",
        bind_port: int = 5556,
        peer_address: str = "127.0.0.1",
        peer_port: int = 5555,
        topic: bytes = b"",
    ):
        """
        Initialize ZeroMQ transport.

        Args:
            bind_address: Local address to bind the PUB socket
            bind_port: Local port to bind the PUB socket
            peer_address: Peer address to connect the SUB socket
            peer_port: Peer port to connect the SUB socket
            topic: Subscription topic (empty means all messages)
        """
        self.bind_address = bind_address
        self.bind_port = bind_port
        self.peer_address = peer_address
        self.peer_port = peer_port
        self.topic = topic

        self._zmq = None
        self._context = None
        self._pub_socket = None
        self._sub_socket = None

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
        """Start the ZeroMQ transport."""
        if self.running:
            return True

        try:
            import zmq  # pylint: disable=import-error

            self._zmq = zmq
            self._context = zmq.Context.instance()

            self._pub_socket = self._context.socket(zmq.PUB)
            self._pub_socket.setsockopt(zmq.LINGER, 0)
            self._pub_socket.bind(f"tcp://{self.bind_address}:{self.bind_port}")

            self._sub_socket = self._context.socket(zmq.SUB)
            self._sub_socket.setsockopt(zmq.LINGER, 0)
            self._sub_socket.setsockopt(zmq.SUBSCRIBE, self.topic)
            self._sub_socket.connect(f"tcp://{self.peer_address}:{self.peer_port}")

            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()

            print(
                f"ZeroMQ transport started (PUB tcp://{self.bind_address}:{self.bind_port}, "
                f"SUB tcp://{self.peer_address}:{self.peer_port})"
            )
            return True

        except Exception as exc:
            print(f"Failed to start ZeroMQ transport: {exc}")
            return False

    def stop(self):
        """Stop the ZeroMQ transport."""
        if not self.running:
            return

        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)

        if self._sub_socket is not None:
            self._sub_socket.close(0)
            self._sub_socket = None

        if self._pub_socket is not None:
            self._pub_socket.close(0)
            self._pub_socket = None

        print("ZeroMQ transport stopped")

    def send(self, data: bytes) -> bool:
        """Send data to peer."""
        if not self.running or self._pub_socket is None:
            return False

        try:
            self._pub_socket.send(data)
            self.messages_sent += 1
            self.bytes_sent += len(data)
            return True

        except Exception as exc:
            print(f"ZeroMQ send error: {exc}")
            self.send_errors += 1
            return False

    def set_message_callback(self, callback: Callable[[bytes, Tuple[str, int]], None]):
        """Set callback for received messages."""
        self.message_callback = callback

    def get_queued_messages(self) -> list:
        """Get all queued messages from the receive thread."""
        messages = []
        while not self.message_queue.empty():
            try:
                messages.append(self.message_queue.get_nowait())
            except queue.Empty:
                break
        return messages

    def get_statistics(self) -> dict:
        """Get transport statistics."""
        return {
            "messages_sent": self.messages_sent,
            "messages_received": self.messages_received,
            "bytes_sent": self.bytes_sent,
            "bytes_received": self.bytes_received,
            "send_errors": self.send_errors,
            "receive_errors": self.receive_errors,
            "running": self.running,
        }

    def reset_statistics(self):
        """Reset statistics counters."""
        self.messages_sent = 0
        self.messages_received = 0
        self.bytes_sent = 0
        self.bytes_received = 0
        self.send_errors = 0
        self.receive_errors = 0

    def _receive_loop(self):
        """Receive loop running in a separate thread."""
        if self._zmq is None or self._sub_socket is None:
            return

        poller = self._zmq.Poller()
        poller.register(self._sub_socket, self._zmq.POLLIN)

        while self.running:
            try:
                events = dict(poller.poll(timeout=100))
                if self._sub_socket in events:
                    data = self._sub_socket.recv(flags=self._zmq.NOBLOCK)
                    self.messages_received += 1
                    self.bytes_received += len(data)

                    addr = (self.peer_address, self.peer_port)
                    if self.message_callback:
                        self.message_callback(data, addr)
                    self.message_queue.put((data, addr))
            except Exception as exc:
                if self.running:
                    print(f"ZeroMQ receive error: {exc}")
                    self.receive_errors += 1
                break


def create_zmq_transport(
    local_port: int = 5556,
    remote_address: str = "127.0.0.1",
    remote_port: int = 5555,
) -> HilTransportZMQ:
    """
    Create ZeroMQ transport with default settings.

    Args:
        local_port: Local port to bind the PUB socket
        remote_address: Remote address to connect the SUB socket
        remote_port: Remote port to connect the SUB socket

    Returns:
        HilTransportZMQ instance
    """
    return HilTransportZMQ(
        bind_address="0.0.0.0",
        bind_port=local_port,
        peer_address=remote_address,
        peer_port=remote_port,
    )
