"""Choose real or simulated IMU inputs for the gap_follow launch (no ROS)."""

import subprocess

# Direct console script or Python's script argument, anchored at argv[0].
# A waiting shell merely mentioning the bridge must not select simulated IMU.
GYM_BRIDGE_PROCESS = (
    r'^([^ ]*python[0-9.]* +)?/[^ ]*/lib/racerbot_sim/gym_bridge_node( |$)')


def gym_bridge_running():
    """Use the same process probe as racerbot_launch/scripts/map_course."""
    return subprocess.run(['pgrep', '-f', GYM_BRIDGE_PROCESS],
                          capture_output=True).returncode == 0


def select_imu_hardware(requested, probe=None):
    """Return (hardware launch-argument text, reason) for auto/true/false.

    Explicit selections do not query processes. Tests can inject the same
    boolean-returning probe that auto uses on a running system.
    """
    text = str(requested).strip().lower()
    if text in ('true', 'false'):
        return text, f'imu_hardware:={text}, explicit selection'
    if text != 'auto':
        raise RuntimeError(f'imu_hardware:={requested!r}: expected auto, true or false')
    running = (probe if probe is not None else gym_bridge_running)()
    if running:
        return 'false', 'imu_hardware:=auto, simulator detected'
    return 'true', 'imu_hardware:=auto, simulator not detected'
