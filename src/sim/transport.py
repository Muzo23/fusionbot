"""
FusionBot ZeroMQ lockstep transport (step 5, protocol 2.0, see sim.protocol).

State channel:   CoppeliaSim PUB tcp://127.0.0.1:23110 -> Python SUB
Command channel: Python PUB -> CoppeliaSim SUB tcp://127.0.0.1:23111
"""

from __future__ import annotations

import time

import zmq

from sim.protocol import (  # noqa: F401  (re-exported)
    _COMMAND_STRUCT,
    COMMAND_ENDPOINT,
    PROTOCOL_VERSION,
    STATE_ENDPOINT,
    SimState,
    decode_state,
    encode_command,
)


class LockstepSession:
    """
    Owns the sockets and the duplicate-state handshake.

        with LockstepSession() as link:
            sim.startSimulation()
            while ...:
                state = link.next_state()          # new state k (dups handled)
                ...compute command...
                link.send(state.state_sequence, l, r)
    """

    def __init__(
        self,
        *,
        state_endpoint: str = STATE_ENDPOINT,
        command_endpoint: str = COMMAND_ENDPOINT,
    ) -> None:
        self.context = zmq.Context()
        self.state_sub = self.context.socket(zmq.SUB)
        self.state_sub.setsockopt(zmq.SUBSCRIBE, b"")
        self.state_sub.setsockopt(zmq.LINGER, 0)
        self.state_sub.connect(state_endpoint)

        self.command_pub = self.context.socket(zmq.PUB)
        self.command_pub.setsockopt(zmq.LINGER, 0)
        self.command_pub.connect(command_endpoint)

        self.poller = zmq.Poller()
        self.poller.register(self.state_sub, zmq.POLLIN)

        self.command_sequence = 0
        self.last_state_sequence: int | None = None
        self._last_payload: bytes | None = None
        self.duplicate_states = 0
        self.skipped_states = 0

    def close(self) -> None:
        try:
            self.poller.unregister(self.state_sub)
        except Exception:
            pass
        self.state_sub.close(0)
        self.command_pub.close(0)
        self.context.term()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def send(self, source_state_sequence: int, left: float, right: float) -> None:
        self.command_sequence += 1
        payload = encode_command(
            self.command_sequence, source_state_sequence, left, right
        )
        self._last_payload = payload
        self.command_pub.send(payload)

    def release(self) -> None:
        """Tell a lockstep bridge to stop waiting (zero commands from now on).
        Must be sent before ANY Remote API call such as stopSimulation()."""
        seq = self.last_state_sequence or 0
        payload = _COMMAND_STRUCT.pack(PROTOCOL_VERSION, -1.0, float(seq), 0.0, 0.0)
        for _ in range(5):  # PUB is fire-and-forget; repeat a few times
            self.command_pub.send(payload)
            time.sleep(0.01)

    def next_state(self, timeout_s: float = 5.0) -> SimState:
        deadline = time.perf_counter() + timeout_s
        while True:
            remaining = deadline - time.perf_counter()
            if remaining <= 0.0:
                raise TimeoutError(
                    f"No new FusionBot state for {timeout_s:.1f} s "
                    f"(last sequence {self.last_state_sequence})."
                )
            events = dict(self.poller.poll(int(min(remaining, 0.1) * 1000) + 1))
            if self.state_sub not in events:
                continue
            state = decode_state(self.state_sub.recv())
            seq = state.state_sequence
            last = self.last_state_sequence
            if last is not None and seq <= last:
                # Simulator re-published a state we already answered:
                # our command was lost -> re-send the identical payload.
                self.duplicate_states += 1
                if seq == last and self._last_payload is not None:
                    self.command_pub.send(self._last_payload)
                continue
            if last is not None and seq > last + 1:
                self.skipped_states += seq - last - 1
            self.last_state_sequence = seq
            return state
