import signal

from tp_a_slam_aruco import graph_slam_node


class _FakeGraphSlamNode:
    def __init__(self):
        self.save_calls = 0
        self.handler_during_save = None

    def save_trajectory(self):
        self.save_calls += 1
        self.handler_during_save = signal.getsignal(signal.SIGINT)


def test_final_trajectory_save_ignores_sigint_and_restores_handler():
    node = _FakeGraphSlamNode()
    original_handler = signal.getsignal(signal.SIGINT)

    graph_slam_node._save_trajectory_ignoring_sigint(node)

    assert node.save_calls == 1
    assert node.handler_during_save == signal.SIG_IGN
    assert signal.getsignal(signal.SIGINT) == original_handler
