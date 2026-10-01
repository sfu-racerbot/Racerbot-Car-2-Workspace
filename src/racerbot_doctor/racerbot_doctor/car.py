"""What car 2's hardware is, and what healthy looks like.

Every number cites where it comes from, so a hardware change has an
obvious place to update. Keep in step with the files named.
"""

# --- VESC: /etc/udev/rules.d/99-vesc.rules, f1tenth_stack config/vesc.yaml
VESC_USB = ('0483', '5740')
VESC_SYMLINK = 'sensors/vesc'               # under /dev
VESC_KERNEL_DRIVER = 'cdc_acm'
VESC_CORE_MIN_HZ = 25.0                     # driver polls at 50 Hz, vesc_driver.cpp:107

# --- Battery: 3S LiPo (team, 2026-10-01). Per-cell: 3.4 V fail, 3.6 V warn,
# 4.25 V is above a full 3S charge.
BATTERY_CELLS = 3
BATTERY_ABSENT_V = 5.0                      # below this: VESC alive on USB only
BATTERY_FAIL_V = 3.4 * BATTERY_CELLS        # 10.2
BATTERY_WARN_V = 3.6 * BATTERY_CELLS        # 10.8
BATTERY_HIGH_V = 4.25 * BATTERY_CELLS       # 12.75

# --- LiDAR: Hokuyo UST-10LX over Ethernet, f1tenth_stack config/sensors.yaml
# (ip_address/ip_port) and the `hokuyo` NetworkManager profile on enP8p1s0.
LIDAR_IFACE = 'enP8p1s0'
LIDAR_HOST_IP = '192.168.0.15/24'
LIDAR_IP = '192.168.0.10'
LIDAR_PORT = 10940
SCAN_MIN_HZ = 30.0                          # UST-10LX scans at 40 Hz
SCAN_MIN_FINITE_FRACTION = 0.5

# --- Joystick: Logitech F710, joy_teleop.yaml. XInput mode is c21f; the
# D (DirectInput) position on the back switch enumerates as c219, and this
# Jetson then creates no /dev/input/js* at all (docs/troubleshooting.md).
JOY_VID = '046d'
JOY_XINPUT_PID = 'c21f'
JOY_DIRECTINPUT_PID = 'c219'
JOY_KERNEL_DRIVER = 'xpad'
JOY_MIN_HZ = 5.0                            # autorepeat_rate: 20.0

# --- Camera: RealSense D435i, racerbot_launch realsense_camera_launch.py
# (424x240 @ 15 fps). Optional: never fails the run (team, 2026-10-01).
CAMERA_USB = ('8086', '0b3a')
CAMERA_KERNEL_DRIVER = 'uvcvideo'
CAMERA_MIN_SPEED_MBPS = 5000.0              # USB 3; 480 = USB 2
CAMERA_TOPIC = '/camera/camera/color/image_raw'
CAMERA_MIN_HZ = 8.0

# --- Bringup: f1tenth_stack bringup_launch.py
ODOM_MIN_HZ = 25.0                          # vesc_to_odom publishes per /sensors/core
LASER_TF = (0.26, 0.0, 0.11)                # base_link->laser, docs/hardware-reference.md
LASER_TF_TOL_M = 0.001

# Executable basenames, as they appear in /proc/<pid>/cmdline.
EXE_VESC_DRIVER = 'vesc_driver_node'
EXE_URG = 'urg_node_driver'
EXE_JOY = 'joy_node'
EXE_MUX = 'ackermann_mux'
EXE_ACK_TO_VESC = 'ackermann_to_vesc_node'
EXE_VESC_TO_ODOM = 'vesc_to_odom_node'
EXE_REALSENSE = 'realsense2_camera_node'
BRINGUP_EXES = (EXE_VESC_DRIVER, EXE_URG, EXE_JOY, EXE_MUX,
                EXE_ACK_TO_VESC, EXE_VESC_TO_ODOM)
# racerbot_sim forges /joy and publishes a second /scan and /odom.
SIM_EXES = ('gym_bridge_node', 'sim_joy_node')

TEST_DOMAIN_ID = '79'                       # CLAUDE.md: tests only
