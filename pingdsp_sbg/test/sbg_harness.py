"""Test harness for pingdsp_sbg integration tests.

Spawns the real ``sbg_to_odom_initializer`` / ``sbg_to_odom`` executables and
drives them with synthetic SBG (NED) messages published from an in-process node,
then observes the resulting ENU ``Odometry``. The live SBG driver and UDP are
not involved - we inject ``SbgEkfNav``/``SbgEkfQuat``/``SbgImuData`` directly.

Integration helpers are gated on ``SBG_AVAILABLE`` (built executables) and on
``sbg_driver`` messages being importable.
"""

import os
import shutil
import subprocess
import threading
import time

try:
    import sbg_driver.msg  # noqa: F401
    SBG_MSGS_AVAILABLE = True
except Exception:
    SBG_MSGS_AVAILABLE = False


def exe(name, package='pingdsp_sbg'):
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


SBG_AVAILABLE = (exe('sbg_to_odom') is not None
                 and exe('sbg_to_odom_initializer') is not None)

_SPAWNED = []


def reap_spawned():
    """Terminate every subprocess spawned by the harness."""
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


def spawn(exe_path, params=None, env=None):
    """Launch a node executable with -p params."""
    args = [exe_path, '--ros-args']
    for kv in (params or []):
        args += ['-p', kv]
    proc = subprocess.Popen(
        args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        env=env)
    _SPAWNED.append(proc)
    return proc


_DOMAIN_SEQ = [0]


def next_domain():
    _DOMAIN_SEQ[0] += 1
    return 81 + (_DOMAIN_SEQ[0] % 8)


class RosProbe:
    """An rclpy node in the test process to publish and observe topics."""

    def __init__(self, name='sbg_probe'):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        self._rclpy = rclpy
        self._owns_init = not rclpy.ok()
        if self._owns_init:
            rclpy.init()
        self.node = Node(name)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._spin = threading.Thread(target=self._exec.spin, daemon=True)
        self._spin.start()

    def collect(self, msg_type, topic, depth=10):
        received = []
        self.node.create_subscription(
            msg_type, topic, lambda m: received.append(m), depth)
        return received

    def publisher(self, msg_type, topic, depth=10):
        return self.node.create_publisher(msg_type, topic, depth)

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


def make_nav(lat, lon, solution_mode=4, altitude=0.0):
    """Build an SbgEkfNav in NED with a given solution mode."""
    from sbg_driver.msg import SbgEkfNav
    msg = SbgEkfNav()
    msg.latitude = float(lat)
    msg.longitude = float(lon)
    msg.altitude = float(altitude)
    msg.velocity.x = 0.0
    msg.velocity.y = 0.0
    msg.velocity.z = 0.0
    msg.status.solution_mode = int(solution_mode)
    return msg


def make_quat(qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    """Build an SbgEkfQuat (NED attitude)."""
    from sbg_driver.msg import SbgEkfQuat
    msg = SbgEkfQuat()
    msg.quaternion.x = float(qx)
    msg.quaternion.y = float(qy)
    msg.quaternion.z = float(qz)
    msg.quaternion.w = float(qw)
    return msg


def make_euler(roll=0.0, pitch=0.0, yaw=0.0):
    """Build an SbgEkfEuler (NED attitude as roll/pitch/yaw [rad])."""
    from sbg_driver.msg import SbgEkfEuler
    msg = SbgEkfEuler()
    msg.angle.x = float(roll)
    msg.angle.y = float(pitch)
    msg.angle.z = float(yaw)
    return msg


def make_imu(gx=0.0, gy=0.0, gz=0.0):
    """Build an SbgImuData with body angular rates."""
    from sbg_driver.msg import SbgImuData
    msg = SbgImuData()
    msg.gyro.x = float(gx)
    msg.gyro.y = float(gy)
    msg.gyro.z = float(gz)
    return msg


def make_imu_short(gx=0.0, gy=0.0, gz=0.0):
    """Build an SbgImuShort with body angular rates (delta_angle, rad/s)."""
    from sbg_driver.msg import SbgImuShort
    msg = SbgImuShort()
    msg.delta_angle.x = float(gx)
    msg.delta_angle.y = float(gy)
    msg.delta_angle.z = float(gz)
    return msg


def wait_until(predicate, timeout=15.0, poll=0.05):
    """Block until predicate() is truthy or timeout elapses."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(poll)
    return False


class SbgStack:
    """Wires the initializer + sbg_to_odom + an in-process probe together."""

    def __init__(self):
        self.domain = next_domain()
        self._env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain))
        self._saved = os.environ.get('ROS_DOMAIN_ID')
        os.environ['ROS_DOMAIN_ID'] = str(self.domain)
        self.probe = None

    def start_nodes(self, params=None):
        spawn(exe('sbg_to_odom_initializer'), params=params, env=self._env)
        spawn(exe('sbg_to_odom'), params=params, env=self._env)

    def start_probe(self):
        self.probe = RosProbe()
        return self.probe

    def stop(self):
        if self.probe is not None:
            self.probe.shutdown()
        reap_spawned()
        if self._saved is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = self._saved
