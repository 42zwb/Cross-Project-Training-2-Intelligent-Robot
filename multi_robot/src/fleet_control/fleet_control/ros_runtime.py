"""Signals request a stop; ROS contexts close only after callbacks have drained."""
import signal
import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.signals import SignalHandlerOptions


class RosRuntime:
    def __init__(self):
        self.stopping = False
        self.previous_handlers = {}

    def request_stop(self, signum, frame):
        # Raising KeyboardInterrupt in a TF/DDS Python binding can turn an
        # ordinary signal into SystemError. Never invalidate a live callback.
        self.stopping = True

    def __enter__(self):
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            self.previous_handlers[sig] = signal.signal(sig, self.request_stop)
        try:
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        except BaseException:
            self.restore_handlers()
            raise
        return self

    def ok(self):
        return not self.stopping and rclpy.ok()

    def spin(self, node, executor=None):
        node._fleet_stopping = lambda: self.stopping
        executor = executor or SingleThreadedExecutor()
        executor.add_node(node)
        try:
            while self.ok():
                executor.spin_once(timeout_sec=.1)
        except ExternalShutdownException:
            pass
        finally:
            executor.shutdown()

    def restore_handlers(self):
        for sig, handler in self.previous_handlers.items():
            signal.signal(sig, handler)

    def __exit__(self, exc_type, exc, traceback):
        try:
            rclpy.try_shutdown()
        finally:
            self.restore_handlers()
