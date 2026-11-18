"""Helpers shared by the integration tests (fake server) and the robot tests (real Navis)."""

import time


def wait_for(condition, timeout: float = 5.0, period: float = 0.05) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(period)
    return bool(condition())


class Harness:
    """Calls services and collects topics through a test client node.

    ``fake`` is the FakeNavis server for the integration tests, or None against a real robot.
    """

    def __init__(self, fake, client):
        self.fake = fake
        self.client = client
        self._clients = {}

    def call_async(self, srv_type, name: str, request=None):
        key = (srv_type, name)
        if key not in self._clients:
            cli = self.client.create_client(srv_type, name)
            assert cli.wait_for_service(timeout_sec=5.0), f"service {name} not available"
            self._clients[key] = cli
        return self._clients[key].call_async(request if request is not None else srv_type.Request())

    def call(self, srv_type, name: str, request=None, timeout: float = 10.0):
        future = self.call_async(srv_type, name, request)
        assert wait_for(future.done, timeout), f"{name} did not respond"
        return future.result()

    def collect(self, msg_type, topic: str):
        """Subscribe on the test client; returns (subscription, list of received messages)."""
        received = []
        sub = self.client.create_subscription(msg_type, topic, received.append, 10)
        return sub, received
