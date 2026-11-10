import math
import os
import re
import signal
import subprocess
import threading
import time

import genomix


# Vehicle and mission geometry
MASS = 2.91
ARM_LENGTH = 0.38998
ROTOR_CF = 9.9016e-4
ROTOR_CT = 1.9e-5

SHOULDER_Z = -0.15
LINK_2 = 0.15
LINK_3 = 0.195
ARM_HOME = (0.0, 0.0, 0.0)

CUBE_FALL_LIMIT = 0.14
TABLE_TOP = 0.20
PEDESTAL_X = -1.0
PEDESTAL_Y = 0.0

CRUISE_Z = 0.75
WORK_Z = 0.55
LIFT_Z = 0.68
PICK_X = 0.87
PICK_Y = 0.0
STANDOFF = 0.24
PICK_REACH = 0.21

GRIPPER_OPEN = -0.02
GRIPPER_CLOSED = 0.03
POSE_TOPIC = "/world/hexa_phynt/pose/info"

ARM_DT = 0.01
ARM_KP = 0.9
ARM_SPEED = 0.28
SERVO_TARGET_SLEW = 0.003
APPROACH_HEIGHT = 0.18
TOP_GRASP_OFFSET = 0.016
APPROACH_TIMEOUT = 10.0
DESCENT_TIME = 2.0
GRASP_XY_TOL = 0.018
GRASP_Z_TOL = 0.015
GRASP_STABLE_TIME = 0.15


g = genomix.connect()
g.rpath(os.path.join(os.environ["HOME"], "openrobots/lib/genom/pocolibs/plugins"))

optitrack = g.load("optitrack")
rotorcraft = g.load("rotorcraft")
pom = g.load("pom")
maneuver = g.load("maneuver")
nhfc = g.load("nhfc")
dynamixel = g.load("dynamixel")


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_error(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def yaw_from_quaternion(q):
    x = q.get("x", 0.0)
    y = q.get("y", 0.0)
    z = q.get("z", 0.0)
    w = q.get("w", 1.0)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def parse_pose_block(block):
    name = re.search(r'name:\s*"([^"]+)"', block)
    position = re.search(r"position\s*\{([^}]*)\}", block, re.DOTALL)
    orientation = re.search(r"orientation\s*\{([^}]*)\}", block, re.DOTALL)
    if not name or not position:
        return None

    def values(text):
        return {
            key: float(value)
            for key, value in re.findall(r"\b([xyzw]):\s*([-+0-9.eE]+)", text)
        }

    xyz = values(position.group(1))
    quaternion = values(orientation.group(1)) if orientation else {}
    return name.group(1), (
        xyz.get("x", 0.0),
        xyz.get("y", 0.0),
        xyz.get("z", 0.0),
        yaw_from_quaternion(quaternion),
    )


def parse_poses(raw):
    poses = {}
    for block in re.split(r"\bpose\s*\{", raw)[1:]:
        parsed = parse_pose_block(block)
        if parsed is not None:
            name, pose = parsed
            poses[name] = pose
    return poses


def gazebo_poses():
    raw = subprocess.check_output(
        ["gz", "topic", "-e", "-n", "1", "-t", POSE_TOPIC],
        stderr=subprocess.DEVNULL,
        text=True,
        timeout=4.0,
    )
    poses = parse_poses(raw)
    if not poses:
        raise RuntimeError(f"nessuna posa letta da {POSE_TOPIC}")
    return poses


class PoseStream:
    def __init__(self):
        self.process = None
        self.thread = None
        self.lock = threading.Lock()
        self.poses = {}
        self.updated = {}

    def start(self):
        self.process = subprocess.Popen(
            ["gz", "topic", "-e", "-t", POSE_TOPIC],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()
        self.snapshot(timeout=4.0)
        return self

    def _read(self):
        block = []
        depth = 0
        for line in self.process.stdout:
            if depth == 0:
                if not re.search(r"\bpose\s*\{", line):
                    continue
                block = [line]
                depth = line.count("{") - line.count("}")
            else:
                block.append(line)
                depth += line.count("{") - line.count("}")

            if block and depth == 0:
                parsed = parse_pose_block("".join(block))
                block = []
                if parsed is None:
                    continue
                name, pose = parsed
                with self.lock:
                    self.poses[name] = pose
                    self.updated[name] = time.monotonic()

    def snapshot(self, timeout=0.25):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                poses = dict(self.poses)
                updated = dict(self.updated)
            try:
                cube_name = pose_name(poses, "cube")
                drone_name = pose_name(poses, "fahexa")
            except RuntimeError:
                time.sleep(0.002)
                continue
            now = time.monotonic()
            if now - updated[cube_name] < 0.20 and now - updated[drone_name] < 0.20:
                return poses
            time.sleep(0.002)
        raise RuntimeError("stream pose Gazebo non aggiornato")

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()


def pose_name(poses, name):
    if name in poses:
        return name
    for candidate in poses:
        if candidate.endswith("::" + name):
            return candidate
    raise RuntimeError(f"posa Gazebo non trovata: {name}")


def named_pose(poses, name):
    return poses[pose_name(poses, name)]


def gripper_center(poses):
    try:
        left = named_pose(poses, "left_finger")
        right = named_pose(poses, "right_finger")
        return tuple((left[i] + right[i]) / 2.0 for i in range(3))
    except RuntimeError:
        drone = named_pose(poses, "fahexa")
        joints = arm_position()
        return arm_fk_world(joints, drone)


def publish_gripper(position, repeats):
    position = clamp(position, GRIPPER_OPEN, GRIPPER_CLOSED)
    base = ["gz", "topic", "-m", "gz.msgs.Double", "-p", f"data: {position}"]
    topics = (
        "/model/fahexa/joint/left_finger_joint/cmd_pos",
        "/model/fahexa/joint/right_finger_joint/cmd_pos",
    )
    for index in range(repeats):
        processes = [
            subprocess.Popen(
                base + ["-t", topic],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            for topic in topics
        ]
        for process in processes:
            process.wait()
        if index + 1 < repeats:
            time.sleep(0.1)


def open_gripper():
    publish_gripper(GRIPPER_OPEN, repeats=4)


def close_gripper():
    publish_gripper(GRIPPER_CLOSED, repeats=6)


def arm_position():
    values = dynamixel.motors().get("motors", {}).get("position", [])
    if len(values) < 3:
        raise RuntimeError("posizione Dynamixel non disponibile")
    return tuple(float(value) for value in values[:3])


def set_arm_velocity(values):
    command = [0.0] * 8
    command[:3] = values
    dynamixel.set_velocity({"velocity": command})


def arm_step(target, max_speed=ARM_SPEED):
    current = arm_position()
    errors = (
        angle_error(target[0], current[0]),
        target[1] - current[1],
        target[2] - current[2],
    )
    velocity = [clamp(ARM_KP * error, -max_speed, max_speed) for error in errors]
    set_arm_velocity(velocity)
    return current, max(abs(error) for error in errors)


def slew_joints(current, goal, max_step=SERVO_TARGET_SLEW):
    result = list(current)
    result[0] += clamp(angle_error(goal[0], result[0]), -max_step, max_step)
    for index in (1, 2):
        result[index] += clamp(goal[index] - result[index], -max_step, max_step)
    return tuple(result)


def hold_arm(target, duration, max_speed=ARM_SPEED):
    end = time.monotonic() + duration
    try:
        while time.monotonic() < end:
            arm_step(target, max_speed=max_speed)
            time.sleep(ARM_DT)
    finally:
        set_arm_velocity((0.0, 0.0, 0.0))


def world_to_body(target, drone):
    dx = target[0] - drone[0]
    dy = target[1] - drone[1]
    c = math.cos(drone[3])
    s = math.sin(drone[3])
    return c * dx + s * dy, -s * dx + c * dy, target[2] - drone[2]


def arm_ik(target_world, drone_pose, current, from_above=False):
    x, y, z = world_to_body(target_world, drone_pose)
    yaw = math.atan2(y, x)
    radius = math.hypot(x, y)
    height = z - SHOULDER_Z
    distance = math.hypot(radius, height)

    if not abs(LINK_3 - LINK_2) < distance < LINK_2 + LINK_3:
        raise RuntimeError(f"target fuori workspace: distanza {distance:.3f} m")

    cosine = (
        distance * distance - LINK_2 * LINK_2 - LINK_3 * LINK_3
    ) / (2.0 * LINK_2 * LINK_3)
    elbow = math.acos(clamp(cosine, -1.0, 1.0))

    candidates = []
    for elbow_standard in (-elbow, elbow):
        k1 = LINK_2 + LINK_3 * math.cos(elbow_standard)
        k2 = LINK_3 * math.sin(elbow_standard)
        shoulder_standard = math.atan2(height, radius) - math.atan2(k2, k1)
        joints = (yaw, -shoulder_standard, -elbow_standard)
        if not (-math.pi <= joints[0] <= math.pi):
            continue
        shoulder_min = -1.45 if from_above else 0.0
        if not (shoulder_min <= joints[1] <= 1.45):
            continue
        if from_above and joints[1] > 0.02:
            continue
        if not (-2.30 <= joints[2] <= 2.30):
            continue
        change = sum(angle_error(joints[i], current[i]) ** 2 for i in range(3))
        candidates.append((change, joints))

    if not candidates:
        if from_above:
            raise RuntimeError(
                f"nessuna configurazione IK sicura dall'alto "
                f"r={radius:.3f} h={height:.3f}"
            )
        raise RuntimeError("nessuna configurazione IK sicura")
    return min(candidates, key=lambda candidate: candidate[0])[1]


def arm_fk_world(joints, drone):
    yaw, shoulder, elbow = joints
    radius = (
        LINK_2 * math.cos(shoulder)
        + LINK_3 * math.cos(shoulder + elbow)
    )
    z_body = SHOULDER_Z - (
        LINK_2 * math.sin(shoulder)
        + LINK_3 * math.sin(shoulder + elbow)
    )
    x_body = radius * math.cos(yaw)
    y_body = radius * math.sin(yaw)
    c = math.cos(drone[3])
    s = math.sin(drone[3])
    return (
        drone[0] + c * x_body - s * y_body,
        drone[1] + s * x_body + c * y_body,
        drone[2] + z_body,
    )


def goto(x, y, z, duration, arm_target, yaw=0.0, settle=2.0):
    maneuver.set_current_state()
    maneuver.goto({"x": x, "y": y, "z": z, "yaw": yaw, "duration": duration})
    hold_arm(arm_target, duration + settle)


def move_gripper_to(target=None, object_name=None, corrections=2):
    joints = arm_position()
    for _ in range(corrections):
        poses = gazebo_poses()
        drone = named_pose(poses, "fahexa")
        if object_name is not None:
            target = named_pose(poses, object_name)[:3]
        if target is None:
            raise ValueError("target della pinza non specificato")

        joints = arm_ik(target, drone, joints)
        hold_arm(joints, duration=3.0, max_speed=0.22)

        poses = gazebo_poses()
        if object_name is not None:
            target = named_pose(poses, object_name)[:3]
        center = gripper_center(poses)
        error_xy = math.hypot(center[0] - target[0], center[1] - target[1])
        error_z = abs(center[2] - target[2])
        print(f"[GRIP ALIGN] errore xy={error_xy:.3f} z={error_z:.3f}")
        if error_xy < 0.018 and error_z < 0.018:
            return joints
    raise RuntimeError("pinza non allineata al cubo")


def cube_between_fingers(poses, joints):
    cube = named_pose(poses, "cube")
    drone = named_pose(poses, "fahexa")
    center = gripper_center(poses)
    if center is None:
        center = arm_fk_world(joints, drone)
    error_xy = math.hypot(center[0] - cube[0], center[1] - cube[1])
    error_z = abs(center[2] - (cube[2] + TOP_GRASP_OFFSET))
    return error_xy < GRASP_XY_TOL and error_z < GRASP_Z_TOL, error_xy, error_z


class CubeFallen(RuntimeError):
    pass


def live_cube_pose(stream):
    poses = stream.snapshot(timeout=1.0)
    cube = named_pose(poses, "cube")
    if cube[2] < CUBE_FALL_LIMIT:
        raise CubeFallen(f"cubo caduto a z={cube[2]:.3f}")
    return cube


def pick_drone_xy(cube):
    return cube[0] - PICK_REACH, cube[1]


def recenter_on_cube(stream, arm_target, settle=1.5):
    cube = live_cube_pose(stream)
    x, y = pick_drone_xy(cube)
    print(
        f"--- RICENTRAGGIO SU CUBO "
        f"cube=({cube[0]:.3f}, {cube[1]:.3f}, {cube[2]:.3f}) "
        f"drone=({x:.3f}, {y:.3f}, {WORK_Z:.3f}) ---"
    )
    goto(x, y, WORK_Z, 2.5, arm_target, settle=settle)
    return x, y


def servo_grasp(stream):
    open_gripper()
    joints = arm_position()
    target_joints = joints
    phase = "approach"
    phase_started = time.monotonic()
    aligned_since = None
    debug_at = 0.0

    try:
        while True:
            loop_started = time.monotonic()
            poses = stream.snapshot()
            cube = named_pose(poses, "cube")
            drone = named_pose(poses, "fahexa")
            if cube[2] < CUBE_FALL_LIMIT:
                raise CubeFallen(f"cubo caduto a z={cube[2]:.3f}")

            elapsed = loop_started - phase_started
            if phase == "approach":
                height = APPROACH_HEIGHT
            else:
                alpha = min(1.0, elapsed / DESCENT_TIME)
                height = APPROACH_HEIGHT + alpha * (TOP_GRASP_OFFSET - APPROACH_HEIGHT)

            target = (cube[0], cube[1], cube[2] + height)
            goal = arm_ik(target, drone, target_joints, from_above=True)
            target_joints = slew_joints(target_joints, goal)
            joints, joint_error = arm_step(target_joints, max_speed=0.32)

            center = gripper_center(poses)
            target_xy = math.hypot(center[0] - target[0], center[1] - target[1])
            target_z = abs(center[2] - target[2])

            if phase == "approach":
                if target_xy < 0.020 and target_z < 0.020 and joint_error < 0.05:
                    if aligned_since is None:
                        aligned_since = loop_started
                    elif loop_started - aligned_since > 0.20:
                        phase = "descent"
                        phase_started = loop_started
                        aligned_since = None
                        print("[GRIP] allineata sopra il cubo, avvicinamento dall'alto")
                else:
                    aligned_since = None
                if elapsed > APPROACH_TIMEOUT:
                    raise RuntimeError("allineamento sopra il cubo non riuscito")
            else:
                between, error_xy, error_z = cube_between_fingers(poses, joints)
                if between and height < TOP_GRASP_OFFSET + 0.004:
                    if aligned_since is None:
                        aligned_since = loop_started
                    elif loop_started - aligned_since >= GRASP_STABLE_TIME:
                        set_arm_velocity((0.0, 0.0, 0.0))
                        close_gripper()
                        hold_arm(target_joints, duration=1.0, max_speed=0.16)
                        print("[GRIP] cubo centrato, pinza chiusa")
                        return target_joints
                else:
                    aligned_since = None
                if elapsed > DESCENT_TIME + 3.0:
                    raise RuntimeError("cubo non centrato tra le pinze")

            if loop_started >= debug_at:
                print(
                    f"[GRIP {phase}] offset={height:.3f} "
                    f"target_xy={target_xy:.3f} target_z={target_z:.3f}"
                )
                debug_at = loop_started + 0.5

            remaining = ARM_DT - (time.monotonic() - loop_started)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        set_arm_velocity((0.0, 0.0, 0.0))


def verify_grasp(stream, joints):
    poses = stream.snapshot(timeout=1.0)
    cube = named_pose(poses, "cube")
    center = gripper_center(poses)
    return cube[2] > 0.32 and math.dist(cube[:3], center) < 0.050


def place(grasp_joints):
    transfer_x = PEDESTAL_X - STANDOFF
    goto(transfer_x, PEDESTAL_Y, CRUISE_Z, 6.5, grasp_joints, settle=1.5)
    goto(transfer_x, PEDESTAL_Y, WORK_Z, 3.5, grasp_joints, settle=2.0)

    target = (PEDESTAL_X, PEDESTAL_Y, TABLE_TOP + 0.025 + 0.006)
    place_joints = move_gripper_to(target)
    open_gripper()
    hold_arm(place_joints, duration=1.0)

    goto(transfer_x, PEDESTAL_Y, LIFT_Z, 3.0, place_joints, settle=1.0)
    hold_arm(ARM_HOME, duration=3.0)


def setup():
    optitrack.connect({
        "host": "localhost",
        "host_port": "1509",
        "mcast": "",
        "mcast_port": "0",
    })

    rotorcraft.connect({"serial": "/tmp/pty-fahexa", "baud": 0})
    rotorcraft.set_sensor_rate({
        "rate": {"imu": 1000, "mag": 0, "motor": 30, "battery": 1}
    })
    rotorcraft.set_imu_filter({
        "gfc": [18, 18, 18],
        "afc": [4, 4, 4],
        "mfc": [20, 20, 20],
    })

    pom.set_prediction_model("::pom::constant_acceleration")
    pom.set_process_noise({"max_jerk": 100, "max_dw": 50})
    pom.set_history_length({"history_length": 0.25})
    pom.connect_port({"local": "measure/imu", "remote": "rotorcraft/imu"})
    pom.add_measurement("imu")
    pom.connect_port({"local": "measure/mocap", "remote": "optitrack/bodies/FH"})
    pom.add_measurement("mocap")

    maneuver.connect_port({"local": "state", "remote": "pom/frame/robot"})
    maneuver.set_bounds({
        "xmin": -5,
        "xmax": 5,
        "ymin": -5,
        "ymax": 5,
        "zmin": 0,
        "zmax": 3,
        "yawmin": -math.pi,
        "yawmax": math.pi,
    })
    maneuver.set_velocity_limit({"v": 0.55, "w": 0.50})
    maneuver.set_acceleration_limit({"a": 0.55, "dw": 0.70})
    maneuver.set_jerk_limit({"j": 1.0, "ddw": 1.2})
    maneuver.set_snap_limit({"s": 2.5, "dddw": 3.0})

    nhfc.set_gtmrp_geom({
        "rotors": 6,
        "cx": 0,
        "cy": 0,
        "cz": 0,
        "armlen": ARM_LENGTH,
        "mass": MASS,
        "mbodyw": 0.1354,
        "mbodyh": 0.05,
        "mmotor": 0.07,
        "rx": -20,
        "ry": 0,
        "rz": -1,
        "cf": ROTOR_CF,
        "ct": ROTOR_CT,
    })
    nhfc.set_mass({"mass": MASS})
    nhfc.set_wlimit({"wmin": 16.0, "wmax": 120.0})
    nhfc.set_saturation({"sat": {"x": 1, "v": 1, "ix": 1}})
    nhfc.set_servo_gain({"gain": {
        "Kpxy": 11.0,
        "Kpz": 10.0,
        "Kvxy": 20.0,
        "Kvz": 16.0,
        "Kixy": 0.25,
        "Kiz": 0.8,
        "Kqxy": 8.0,
        "Kqz": 1.0,
        "Kwxy": 3.0,
        "Kwz": 0.8,
    }})
    nhfc.set_control_mode({"att_mode": "::nhfc::tilt_prioritized"})
    nhfc.connect_port({"local": "state", "remote": "pom/frame/robot"})
    nhfc.connect_port({"local": "reference", "remote": "maneuver/desired"})
    nhfc.connect_port({
        "local": "rotor_measure",
        "remote": "rotorcraft/rotor_measure",
    })
    rotorcraft.connect_port({
        "local": "rotor_input",
        "remote": "nhfc/rotor_input",
    })

    serial = os.environ.get("DYNAMIXEL_SERIAL", "/tmp/pty-fahexa-arm")
    dynamixel.connect({"serial": serial, "baud": 1000000})


def start():
    rotorcraft.start()
    time.sleep(0.3)
    maneuver.set_current_state()
    nhfc.set_current_position()
    time.sleep(0.2)
    nhfc.servo(ack=True)
    rotorcraft.servo(ack=True)
    dynamixel.servo(ack=True)


def stop():
    for function in (dynamixel.stop, rotorcraft.stop, nhfc.stop, maneuver.stop):
        try:
            function()
        except Exception as error:
            print(f"[STOP] {error}")


def return_and_land(arm_target):
    print("--- RIENTRO E ATTERRAGGIO ---")
    goto(PICK_X, PICK_Y, CRUISE_Z, 3.5, arm_target, settle=1.0)
    hold_arm(ARM_HOME, duration=3.0)
    goto(0.0, 0.0, CRUISE_Z, 5.0, ARM_HOME, settle=1.5)
    goto(0.0, 0.0, 0.05, 5.0, ARM_HOME, settle=1.0)


def mission():
    open_gripper()
    hold_arm(ARM_HOME, duration=1.0)

    print("--- DECOLLO E HOVER A 0.75 M ---")
    maneuver.set_current_state()
    maneuver.take_off({"height": CRUISE_Z, "duration": 4.0})
    hold_arm(ARM_HOME, duration=6.0)

    print("--- VOLO SOPRA LA PIATTAFORMA ---")
    goto(PICK_X, PICK_Y, CRUISE_Z, 4.5, ARM_HOME, settle=3.0)

    stream = PoseStream().start()
    grasp_joints = None
    try:
        cube = live_cube_pose(stream)
        x, y = pick_drone_xy(cube)
        print(f"--- DISCESA A ({x:.2f}, {y:.2f}, {WORK_Z:.2f}), BRACCIO TESO ---")
        goto(x, y, WORK_Z, 3.5, ARM_HOME, settle=3.0)

        while grasp_joints is None:
            try:
                x, y = recenter_on_cube(stream, ARM_HOME)
                grasp_joints = servo_grasp(stream)
                goto(x, y, LIFT_Z, 3.0, grasp_joints, settle=1.0)
                if not verify_grasp(stream, grasp_joints):
                    print("[PICK RETRY] presa non confermata, riprovo")
                    open_gripper()
                    hold_arm(ARM_HOME, duration=2.0)
                    goto(x, y, WORK_Z, 2.5, ARM_HOME, settle=1.0)
                    grasp_joints = None
            except CubeFallen:
                raise
            except RuntimeError as error:
                print(f"[PICK RETRY] {error}; riposiziono il drone")
                open_gripper()
                hold_arm(ARM_HOME, duration=2.0)
    except CubeFallen as error:
        print(f"[ABORT PICK] {error}")
        current = arm_position()
        stream.stop()
        return_and_land(current)
        return
    finally:
        stream.stop()

    place(grasp_joints)
    return_and_land(ARM_HOME)


def main():
    def abort(signum, _frame):
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, abort)
    signal.signal(signal.SIGTERM, abort)

    try:
        setup()
        start()
        mission()
    finally:
        stop()


if __name__ == "__main__":
    main()
