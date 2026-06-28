"""Full-stack test harness for pingdsp_driver.

Analogous to serial_ping_pkg's harness, but the sonar speaks TCP rather than a
serial line, so instead of socat we stand up a small in-process ``FakeSonar``
TCP server that streams synthetic ``DxHeader``/``DxData`` frames (see
``dx_frame_factory``). The real ``tdss_driver`` is launched as a subprocess and
pointed at the fake server; a ``RosProbe`` observes the published topics.

    FakeSonar (TCP, this process)  <--bytes-->  tdss_driver (subprocess)
                                                      |
                                                   ROS topics
                                                      |
                                                  RosProbe (this process)

Integration helpers are gated on ``DRIVER_AVAILABLE`` (the built executable),
so the pure-logic tests still run in an unbuilt checkout.
"""

import os
import shutil
import socket
import subprocess
import threading
import time

from dx_frame_factory import build_dx_frame


def exe(name, package='pingdsp_driver'):
    """Resolve an installed console-script path, or None if not built."""
    path = shutil.which(name)
    if path:
        return path
    prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    for base in prefix.split(os.pathsep):
        candidate = os.path.join(base, 'lib', package, name)
        if os.path.isfile(candidate):
            return candidate
    return None


DRIVER_EXE = exe('tdss_driver')
DRIVER_AVAILABLE = DRIVER_EXE is not None

_SPAWNED = []


def _track(proc):
    _SPAWNED.append(proc)
    return proc


def reap_spawned():
    """Terminate every subprocess spawned by the harness (autouse fixture)."""
    while _SPAWNED:
        proc = _SPAWNED.pop()
        try:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        except Exception:
            pass


class FakeSonar:
    """A TCP server that streams synthetic DX frames to one client."""

    def __init__(self, frames=None, loop=True, interval=0.05):
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(('127.0.0.1', 0))
        self._server.listen(1)
        self.port = self._server.getsockname()[1]
        self.frames = frames if frames is not None else [build_dx_frame()]
        self.loop = loop
        self.interval = interval
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self.client_connected = threading.Event()

    def start(self):
        self._thread.start()
        return self

    def _serve(self):
        self._server.settimeout(1.0)
        while not self._stop.is_set():
            try:
                client, _ = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.client_connected.set()
            try:
                while not self._stop.is_set():
                    for frame in self.frames:
                        if self._stop.is_set():
                            break
                        client.sendall(frame)
                        time.sleep(self.interval)
                    if not self.loop:
                        break
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                try:
                    client.close()
                except OSError:
                    pass
            if not self.loop:
                break

    def stop(self):
        self._stop.set()
        try:
            self._server.close()
        except OSError:
            pass


def spawn_driver(sonar_port, extra_params=None, env=None):
    """Launch tdss_driver pointed at 127.0.0.1:sonar_port."""
    args = [
        DRIVER_EXE, '--ros-args',
        '-p', 'sonar_host:=127.0.0.1',
        '-p', f'sonar_port:={sonar_port}',
    ]
    for kv in (extra_params or []):
        args += ['-p', kv]
    return _track(subprocess.Popen(
        args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        env=env))


def spawn_node(exe_path, extra_args=None, env=None):
    """Launch any pingdsp_driver node executable."""
    return _track(subprocess.Popen(
        [exe_path, '--ros-args'] + (extra_args or []),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env))


_DOMAIN_SEQ = [0]
_DOMAIN_BASE = 91
_DOMAIN_SPAN = 8


def _next_domain():
    _DOMAIN_SEQ[0] += 1
    return _DOMAIN_BASE + (_DOMAIN_SEQ[0] % _DOMAIN_SPAN)


class RosProbe:
    """An rclpy node spun up in the test process to observe topics."""

    def __init__(self, name=None):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        self._rclpy = rclpy
        self._owns_init = not rclpy.ok()
        if self._owns_init:
            rclpy.init()
        self.node = Node(name or 'pingdsp_probe')
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._spin = threading.Thread(target=self._exec.spin, daemon=True)
        self._spin.start()

    def collect(self, msg_type, topic, depth=10):
        """Subscribe and return a list that grows as messages arrive."""
        from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=depth)
        received = []
        self.node.create_subscription(
            msg_type, topic, lambda m: received.append(m), qos)
        return received

    def set_param(self, node_name, name, value):
        """Set a remote parameter via the set_parameters service."""
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        cli = self.node.create_client(
            SetParameters, f'/{node_name}/set_parameters')
        if not cli.wait_for_service(timeout_sec=5.0):
            return False
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = value
        elif isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = value
        else:
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = str(value)
        req = SetParameters.Request(
            parameters=[Parameter(name=name, value=pv)])
        future = cli.call_async(req)
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        return future.done()

    def shutdown(self):
        try:
            self._exec.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if self._owns_init and self._rclpy.ok():
            self._rclpy.shutdown()


class Stack:
    """Wires a FakeSonar + driver subprocess + in-process RosProbe together."""

    def __init__(self, frames=None, loop=True, interval=0.05):
        self.domain = _next_domain()
        self._env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain))
        self._saved_domain = os.environ.get('ROS_DOMAIN_ID')
        os.environ['ROS_DOMAIN_ID'] = str(self.domain)
        self.sonar = FakeSonar(frames=frames, loop=loop, interval=interval)
        self.driver = None
        self.nodes = []
        self.probe = None

    def start_sonar(self):
        self.sonar.start()
        return self

    def start_driver(self, extra_params=None):
        self.driver = spawn_driver(
            self.sonar.port, extra_params=extra_params, env=self._env)
        return self

    def start_node(self, exe_path, extra_args=None):
        proc = spawn_node(exe_path, extra_args=extra_args, env=self._env)
        self.nodes.append(proc)
        return proc

    def start_probe(self):
        self.probe = RosProbe()
        return self.probe

    def stop(self):
        if self.probe is not None:
            self.probe.shutdown()
        self.sonar.stop()
        reap_spawned()
        if self._saved_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = self._saved_domain


def wait_until(predicate, timeout=10.0, poll=0.05):
    """Block until predicate() is truthy or timeout elapses."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(poll)
    return False
