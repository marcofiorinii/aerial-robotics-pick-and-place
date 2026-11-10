import json
import math
import os
import re
import signal
import shutil
import subprocess
import threading
import time

import genomix

# Componenti GenoM usati dalla missione.
g = genomix.connect()
g.rpath(os.environ["HOME"] + "/openrobots/lib/genom/pocolibs/plugins")

optitrack = g.load("optitrack")
rotorcraft = g.load("rotorcraft")
pom       = g.load("pom")
maneuver  = g.load("maneuver")
nhfc      = g.load("nhfc")
phynt     = g.load("phynt")
dynamixel = g.load("dynamixel")

# I path dei log GenoM sono limitati a 64 caratteri.
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
LOG_DIR = os.path.join(BASE_DIR, 'logs', 'last')
if os.path.isdir(LOG_DIR):
    shutil.rmtree(LOG_DIR)
os.makedirs(LOG_DIR, exist_ok=True)
print(f"[LOG] directory: {LOG_DIR}")

ROBOT_SERIAL = "/tmp/pty-fahexa"
MASS = 2.91
ARMLEN = 0.38998
CF = 9.9016e-4
CT = 1.9e-5

# Le dita prismatiche usano direttamente i topic Gazebo.
GRIPPER_OPEN = -0.02
# Il limite fisico SDF delle dita e' 0.03 m. Comandare oltre quel valore non
# aumenta la corsa, ma mantiene il position controller in spinta sul cubo.
GRIPPER_CLOSED = 0.085
GRIP_REFRESH_PERIOD = 0.06
GRIP_STABLE_SECS = 0.0
MAX_GRASP_ATTEMPTS = 7
GRASP_DEBUG_ENABLED = True
GRASP_DEBUG_HZ = 25.0


def _reap_processes(processes):
    for process in processes:
        try:
            process.wait()
        except Exception:
            pass


def _publish_gripper_once(pos):
    cmd = ["gz", "topic", "-m", "gz.msgs.Double", "-p", f"data: {pos}"]
    processes = [
        subprocess.Popen(
            cmd + ["-t", "/model/fahexa/joint/left_finger_joint/cmd_pos"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
        subprocess.Popen(
            cmd + ["-t", "/model/fahexa/joint/right_finger_joint/cmd_pos"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
    ]
    threading.Thread(target=_reap_processes, args=(processes,), daemon=True).start()


def set_gripper(pos, n_repeat=2, dt=0.04):
    """Accoda comandi pinza senza bloccare la missione.

    `gz topic -p` puo' impiegare anche ~1 s prima di terminare. Aspettarlo qui
    bloccava la presa: nella run del 28/06 close_begin->close_done durava 12 s.
    I processi vengono quindi lanciati fire-and-forget e raccolti da un reaper.
    """
    pos = max(GRIPPER_OPEN, min(GRIPPER_CLOSED, pos))

    def worker():
        for i in range(n_repeat):
            _publish_gripper_once(pos)
            if i < n_repeat - 1:
                time.sleep(dt)

    threading.Thread(target=worker, daemon=True).start()


def open_gripper(n_repeat=6, dt=0.035):
    _grasp_debug_log("gripper_open", phase="gripper", gripper_cmd=GRIPPER_OPEN, force=True)
    set_gripper(GRIPPER_OPEN, n_repeat=n_repeat, dt=dt)


def close_gripper():
    _grasp_debug_log("close_begin", phase="gripper", gripper_cmd=GRIPPER_CLOSED, force=True)
    set_gripper(GRIPPER_CLOSED, n_repeat=20, dt=0.030)
    _grasp_debug_log("close_queued", phase="gripper", gripper_cmd=GRIPPER_CLOSED, force=True)


def refresh_gripper_closed():
    """Rinforza la chiusura senza bloccare il loop del braccio."""
    set_gripper(GRIPPER_CLOSED, n_repeat=1, dt=0.0)


def _get_cube_pose():
    """Legge la posizione world del cubo da Gazebo, con fallback testuale."""
    topic_args = ["-e", "-n", "1", "-t", "/world/hexa_phynt/pose/info"]

    try:
        raw_json = subprocess.check_output(
            ["gz", "topic", "--json"] + topic_args,
            stderr=subprocess.DEVNULL, timeout=5.0
        ).decode('utf-8', errors='replace')
        data = json.loads(raw_json)
        for pose in data.get("pose", []):
            if pose.get("name") == "cube":
                pos = pose.get("position", {})
                return pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)
    except Exception:
        pass

    try:
        raw = subprocess.check_output(
            ["gz", "topic"] + topic_args,
            stderr=subprocess.DEVNULL, timeout=5.0
        ).decode('utf-8', errors='replace')
    except Exception:
        return None

    chunks = re.split(r'\bpose\s*\{', raw)
    for chunk in chunks:
        m_name = re.search(r'name:\s*"([^"]*)"', chunk)
        if not m_name or m_name.group(1) != "cube":
            continue
        pm = re.search(r'position\s*\{([^}]*)\}', chunk, re.DOTALL)
        if not pm:
            continue
        p = pm.group(1)
        xm = re.search(r'x:\s*([-\d.eE+]+)', p)
        ym = re.search(r'y:\s*([-\d.eE+]+)', p)
        zm = re.search(r'z:\s*([-\d.eE+]+)', p)
        x = float(xm.group(1)) if xm else 0.0
        y = float(ym.group(1)) if ym else 0.0
        z = float(zm.group(1)) if zm else 0.0
        return x, y, z

    return None


def _parse_cube_pose_block(block):
    name = re.search(r'name:\s*"([^"]*)"', block)
    if not name or name.group(1) != "cube":
        return None
    position = re.search(r'position\s*\{([^}]*)\}', block, re.DOTALL)
    if not position:
        return None
    values = {
        key: float(value)
        for key, value in re.findall(r'\b([xyz]):\s*([-+0-9.eE]+)', position.group(1))
    }
    return values.get("x", 0.0), values.get("y", 0.0), values.get("z", 0.0)


def _update_cube_cache(pose):
    with _cube_lock:
        global _cube_cache
        _cube_cache = (time.time(), pose)



# La posa del cubo viene letta fuori dal loop del braccio per evitare jitter.
CUBE_READ_HZ = 80.0
CUBE_CACHE_MAX_AGE = 0.12

_cube_cache = None
_cube_lock = threading.Lock()
_cube_stop = threading.Event()
_cube_thread = None


def _cube_reader_loop(period):
    while not _cube_stop.is_set():
        process = None
        try:
            process = subprocess.Popen(
                ["gz", "topic", "-e", "-t", "/world/hexa_phynt/pose/info"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
            block = []
            depth = 0
            while not _cube_stop.is_set() and process.poll() is None:
                line = process.stdout.readline()
                if not line:
                    time.sleep(0.001)
                    continue
                if depth == 0:
                    if not re.search(r"\bpose\s*\{", line):
                        continue
                    block = [line]
                    depth = line.count("{") - line.count("}")
                else:
                    block.append(line)
                    depth += line.count("{") - line.count("}")

                if block and depth == 0:
                    pose = _parse_cube_pose_block("".join(block))
                    block = []
                    if pose is not None:
                        _update_cube_cache(pose)
        except Exception:
            pose = _get_cube_pose()
            if pose is not None:
                _update_cube_cache(pose)
            time.sleep(period)
        finally:
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    process.kill()
        if not _cube_stop.is_set():
            time.sleep(0.05)


def start_cube_reader(rate_hz=CUBE_READ_HZ):
    global _cube_thread, _cube_cache
    if _cube_thread is not None and _cube_thread.is_alive():
        if not _cube_stop.is_set():
            return
        _cube_thread.join(timeout=0.25)
        if _cube_thread.is_alive():
            return
    with _cube_lock:
        _cube_cache = None
    _cube_stop.clear()
    period = 1.0 / max(1.0, rate_hz)
    _cube_thread = threading.Thread(target=_cube_reader_loop, args=(period,), daemon=True)
    _cube_thread.start()


def stop_cube_reader():
    _cube_stop.set()


def get_cube_pose_cached(max_age=CUBE_CACHE_MAX_AGE):
    with _cube_lock:
        cached = _cube_cache
    if cached is None:
        return None
    ts, pose = cached
    if time.time() - ts > max_age:
        return None
    return pose


# Geometria e tolleranze della missione.
CUBE_X, CUBE_Y = 1.08, 0.0
PEDESTAL2_X, PEDESTAL2_Y = -1.0, 0.0
CRUISE_ALT = 1.00
PICKUP_ALT = 0.62
PLACE_WORK_ALT = 0.68
ARM_REACH = 0.27
ARM_STRETCHED = (0.0, 0.0, 0.0)
ARM_TUCKED = (0.0, math.pi / 3, math.pi / 4)
CUBE_CENTER_Z_NOMINAL = 0.225
CUBE_SIZE = 0.05
PEDESTAL_TOP_Z = 0.20
POST_GRASP_CHECK_ALT = CRUISE_ALT
POST_GRASP_HOVER_TIME = 0.6
GRASP_FALLEN_Z = PEDESTAL_TOP_Z - 0.005
GRASP_ON_PEDESTAL_Z = CUBE_CENTER_Z_NOMINAL + 0.035
PLACE_SURFACE_CLEARANCE = 0.025
PLACE_RELEASE_CLEARANCE = 0.050
PLACE_ARM_REACH = 0.29
PLACE_CUBE_RELEASE_Z = PEDESTAL_TOP_Z + CUBE_SIZE / 2.0 + PLACE_RELEASE_CLEARANCE
PLACE_CUBE_APPROACH_Z = PLACE_CUBE_RELEASE_Z + 0.110
PLACE_GRIPPER_RELEASE_Z = PLACE_CUBE_RELEASE_Z
PLACE_GRIPPER_APPROACH_Z = PLACE_CUBE_APPROACH_Z
PLACE_APPROACH_BACKOFF = -0.035
PLACE_APPROACH_Z = 0.105
PLACE_TARGET_XY_TOL = 0.035
PLACE_TARGET_Z_TOL = 0.030
PLACE_ARM_ERROR_TOL = 0.100
PLACE_RELEASE_XY_TOL = 0.045
PLACE_RELEASE_SERVO_XY_TOL = 0.085
PLACE_RELEASE_FINAL_XY_TOL = 0.085
PLACE_RELEASE_Z_MIN = PLACE_CUBE_RELEASE_Z - 0.018
PLACE_RELEASE_Z_MAX = PLACE_CUBE_RELEASE_Z + 0.022
PLACE_RECENTER_XY_TOL = 0.035
PLACE_RECENTER_MAX_STEP = 0.040
PLACE_SERVO_CLAMP_XY = 0.160
PLACE_SERVO_CLAMP_Z = 0.350
PLACE_JOINT_SLEW = 1.35
VERIFY_DXY_TOL = 0.045
VERIFY_Z_TOL = 0.024
FINGER_Z_SIZE = 0.020
MIN_GRIP_Z_OVERLAP = 0.002
GRIPPER_CAPTURE_X_HALF = 0.060
GRIPPER_CAPTURE_Y_HALF = 0.055
GRIPPER_CAPTURE_Z_HALF = 0.045
PRE_GRASP_SETTLE_TIME = 1.6
GRASP_SCAN_OFFSETS = [
    (0.000, 0.000, 0.000),
    (0.000, 0.012, 0.000),
    (0.000, -0.012, 0.000),
    (0.012, 0.000, 0.000),
    (-0.012, 0.000, 0.000),
    (0.000, 0.000, 0.008),
    (0.000, 0.000, -0.006),
]
IK_APPROACH_Z = 0.095       # EE sopra il cubo: approccio dall'alto
IK_APPROACH_BACKOFF = -0.035 # approccio obliquo da dietro, non discesa verticale
IK_GRASP_Z_OFFSET = 0.004   # target basso: la pinza deve entrare davvero sul cubo

# Geometria del braccio e limiti software anti-collisione.
ARM_LIMITS = (
    (-math.pi, math.pi),
    (-1.40, 1.45),
    (-2.35, 2.35),
)
SAFE_SHOULDER_MIN = -0.10
_Z_J2 = -0.15
_L2 = 0.15
_L3 = 0.20
_J3_LIMIT = 2.35

def _arm_joints_within_limits(joints):
    return all(lo <= value <= hi for value, (lo, hi) in zip(joints, ARM_LIMITS))


def _limit_arm_joints(joints, label=None):
    j1, j2, j3 = joints
    limited = tuple(_clamp(value, lo, hi) for value, (lo, hi) in zip(joints, ARM_LIMITS))
    if limited != tuple(joints) and label:
        print(f"[SAFETY] {label}: target braccio limitato "
              f"({j1:.3f},{j2:.3f},{j3:.3f}) -> "
              f"({limited[0]:.3f},{limited[1]:.3f},{limited[2]:.3f})")
    return limited


def _set_arm_target(j1, j2, j3, label=None):
    j1, j2, j3 = _limit_arm_joints((j1, j2, j3), label=label)
    with _arm_lock:
        _arm_target[:] = (j1, j2, j3)
    return j1, j2, j3


def _drone_ik(px_b, py_b, pz_b, elbow_up=None, strict=False):
    """
    IK analitica 2-link planare per fah_arm, frame body drone.

    Di default valuta entrambe le soluzioni del gomito e sceglie la prima che
    rispetta i limiti software anti-compenetrazione. Questo evita casi come il
    deposito, dove la soluzione matematica elbow-down porta j2 molto negativo e
    fa salire il primo link dentro il body del drone.
    """
    j1 = math.atan2(py_b, px_b)
    R = math.hypot(px_b, py_b)
    h = pz_b - _Z_J2
    D = math.hypot(R, h)
    d_max = _L2 + _L3 - 1e-4
    d_min = math.sqrt(_L2**2 + _L3**2 + 2.0 * _L2 * _L3 * math.cos(_J3_LIMIT)) - 1e-4
    if D > d_max:
        raise ValueError(f"troppo lontano D={D:.3f} > {d_max:.3f}")
    if D < d_min:
        raise ValueError(f"troppo vicino D={D:.3f} < {d_min:.3f}")

    cos_j3 = (D**2 - _L2**2 - _L3**2) / (2.0 * _L2 * _L3)
    cos_j3 = max(-1.0, min(1.0, cos_j3))
    j3_geo = math.acos(cos_j3)

    def solution(use_elbow_up):
        j3_std = j3_geo if use_elbow_up else -j3_geo
        k1 = _L2 + _L3 * math.cos(j3_std)
        k2 = _L3 * math.sin(j3_std)
        j2_std = math.atan2(h, R) - math.atan2(k2, k1)
        return j1, -j2_std, -j3_std

    if elbow_up is None:
        raw_candidates = [solution(False), solution(True)]
        safe = [joints for joints in raw_candidates if joints[1] >= SAFE_SHOULDER_MIN]
        high = [joints for joints in raw_candidates if joints[1] < SAFE_SHOULDER_MIN]
        candidates = safe + high
    else:
        preferred = solution(bool(elbow_up))
        alternate = solution(not bool(elbow_up))
        candidates = [preferred] if strict else [preferred, alternate]

    for joints in candidates:
        if _arm_joints_within_limits(joints):
            return joints

    parts = []
    for index, joints in enumerate(candidates, start=1):
        limited = _limit_arm_joints(joints)
        parts.append(
            f"sol{index}=({joints[0]:.3f},{joints[1]:.3f},{joints[2]:.3f}) "
            f"lim=({limited[0]:.3f},{limited[1]:.3f},{limited[2]:.3f})"
        )
    raise ValueError("IK richiede braccio fuori limiti: " + " ".join(parts))


def _fk_arm(j1, j2, j3):
    """Cinematica diretta: angoli Gazebo → posizione EE in frame body drone."""
    phi2 = j2
    phi3 = j2 + j3
    R  = _L2 * math.cos(phi2) + _L3 * math.cos(phi3)
    dz = -(_L2 * math.sin(phi2) + _L3 * math.sin(phi3))
    return R * math.cos(j1), R * math.sin(j1), _Z_J2 + dz


def _world_to_body(dx_w, dy_w, yaw):
    """Ruota un offset orizzontale dal frame world al frame body (rotazione yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx_w + s * dy_w, -s * dx_w + c * dy_w


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))

# I port POM omettono campi opzionali: la posa viene letta dal log di stato.
def _read_drone_pose(n_avg=20):
    """
    Legge (x, y, z, yaw) dal log di pom come media degli ultimi n_avg campioni.
    Restituisce None se il log non è disponibile o z < 0.3 m (drone a terra).
    """
    try:
        pom_log = os.path.join(LOG_DIR, "pom.log")
        with open(pom_log, 'rb') as f:
            f.seek(0, 2)
            size  = f.tell()
            chunk = min(size, 4096)  # 4 KB basta per ~20 righe a 200 byte/riga
            f.seek(-chunk, 2)
            tail  = f.read().decode('utf-8', errors='replace')
        xs, ys, zs, yaws = [], [], [], []
        for line in reversed(tail.splitlines()):
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('ts'):
                continue
            try:
                cols = line.split()
                z = float(cols[9])
                if z < 0.3:   # drone a terra o dato non valido
                    continue
                xs.append(float(cols[7]))
                ys.append(float(cols[8]))
                zs.append(z)
                yaws.append(float(cols[12]))
            except (IndexError, ValueError):
                continue
            if len(xs) >= n_avg:
                break
        if not xs:
            return None
        return sum(xs)/len(xs), sum(ys)/len(ys), sum(zs)/len(zs), sum(yaws)/len(yaws)
    except Exception:
        return None

# Le letture da disco avvengono in background; i controller usano la cache RAM.
_pom_cache = None
_pom_lock  = threading.Lock()
_pom_stop  = threading.Event()

def _pom_reader_loop():
    while not _pom_stop.is_set():
        pose = _read_drone_pose(n_avg=30)
        with _pom_lock:
            global _pom_cache
            _pom_cache = pose
        time.sleep(0.004)   # 250 Hz reader, POM resta mediato su ~30 ms

def start_pom_reader():
    _pom_stop.clear()
    threading.Thread(target=_pom_reader_loop, daemon=True).start()

def stop_pom_reader():
    _pom_stop.set()

def get_pom_pose():
    with _pom_lock:
        return _pom_cache


def _average_pom_pose(duration=0.8, period=0.02):
    """Media breve della posa POM dalla cache, con yaw mediato su seno/coseno."""
    samples = []
    t_end = time.time() + duration
    while time.time() < t_end:
        pose = get_pom_pose()
        if pose is not None:
            samples.append(pose)
        time.sleep(period)
    if not samples:
        return None
    x = sum(p[0] for p in samples) / len(samples)
    y = sum(p[1] for p in samples) / len(samples)
    z = sum(p[2] for p in samples) / len(samples)
    sy = sum(math.sin(p[3]) for p in samples) / len(samples)
    cy = sum(math.cos(p[3]) for p in samples) / len(samples)
    yaw = math.atan2(sy, cy)
    return x, y, z, yaw

# I target di posizione Dynamixel vengono sovrascritti: usiamo un P in velocita'.
KP_ARM = 0.80           # guadagno P [rad/s / rad]
MAX_VEL = 0.52          # velocità max nominale [rad/s]
ARM_GRASP_MAX_VEL = 0.40
ARM_CTRL_DT = 0.010     # 100 Hz: correzione rapida durante la presa
ARM_ACCEL_LIMIT = 0.75  # rad/s^2, limita gli impulsi di reazione sul drone
ARM_GRASP_ACCEL_LIMIT = 0.65

_arm_target = [0.0, 0.0, 0.0]   # target corrente [j1, j2, j3], aggiornato dal tracking
_arm_cmd_vel = [0.0, 0.0, 0.0]  # rampa velocita' inviata ai Dynamixel
_arm_lock = threading.Lock()    # protegge _arm_target da accessi concorrenti


def _read_arm_pos():
    state = dynamixel.motors()
    return state.get("motors", {}).get("position", [0.0] * 8)


def _step_arm_controller(max_vel=MAX_VEL, accel_limit=ARM_ACCEL_LIMIT, dt=ARM_CTRL_DT):
    """Un passo del controller P in velocità con limitazione di accelerazione."""
    pos = _read_arm_pos()
    vel = [0.0] * 8
    max_err = 0.0
    with _arm_lock:
        raw_target = tuple(_arm_target)
    limited_target = _limit_arm_joints(raw_target)
    if limited_target != raw_target:
        print(f"[SAFETY] loop: target braccio limitato "
              f"({raw_target[0]:.3f},{raw_target[1]:.3f},{raw_target[2]:.3f}) -> "
              f"({limited_target[0]:.3f},{limited_target[1]:.3f},{limited_target[2]:.3f})")
        with _arm_lock:
            _arm_target[0], _arm_target[1], _arm_target[2] = limited_target
    target = list(limited_target)

    dv_max = max(0.02, accel_limit * max(dt, 1e-3))
    for i in range(3):
        e = target[i] - pos[i]
        max_err = max(max_err, abs(e))
        desired = _clamp(KP_ARM * e, -max_vel, max_vel)
        _arm_cmd_vel[i] += _clamp(desired - _arm_cmd_vel[i], -dv_max, dv_max)
        vel[i] = _arm_cmd_vel[i]

    dynamixel.set_velocity({"velocity": vel})
    return max_err, pos


def _run_arm_controller(duration, grip=False, max_vel=MAX_VEL,
                        accel_limit=ARM_ACCEL_LIMIT, dt=ARM_CTRL_DT):
    """Esegue il loop P; grip=True reinvia la chiusura periodicamente."""
    t_end = time.time() + duration
    last_grip_t = 0.0
    while time.time() < t_end:
        _step_arm_controller(max_vel=max_vel, accel_limit=accel_limit, dt=dt)
        if grip:
            now = time.time()
            if now - last_grip_t >= GRIP_REFRESH_PERIOD:
                refresh_gripper_closed()
                last_grip_t = now
        time.sleep(dt)


def move_arm_to(j1=0.0, j2=0.0, j3=0.0, duration=3.0, grip=False,
                max_vel=MAX_VEL, accel_limit=ARM_ACCEL_LIMIT):
    """Porta il braccio agli angoli dati e li mantiene per duration secondi."""
    _set_arm_target(j1, j2, j3, label="move_arm_to")
    _run_arm_controller(duration, grip=grip, max_vel=max_vel, accel_limit=accel_limit)

# SETUP — configurazione componenti GenoM
def setup():
    optitrack.connect({
        "host": "localhost",
        "host_port": "1509",
        "mcast": "",
        "mcast_port": "0",
    })

    rotorcraft.connect({"serial": ROBOT_SERIAL, "baud": 0})
    rotorcraft.set_sensor_rate({"rate": {"imu": 1000, "mag": 0, "motor": 30, "battery": 1}})
    rotorcraft.set_imu_filter({"gfc": [18, 18, 18], "afc": [4, 4, 4], "mfc": [20, 20, 20]})
    try:
        rotorcraft.set_cmd_filter({"fc": 35.0})
    except Exception as e:
        print(f"[WARN] rotorcraft.set_cmd_filter non disponibile: {e}")
    try:
        rotorcraft.set_ramp({"ramp": 0.25})
    except Exception as e:
        print(f"[WARN] rotorcraft.set_ramp non disponibile: {e}")

    pom.set_prediction_model("::pom::constant_acceleration")
    pom.set_process_noise({"max_jerk": 100, "max_dw": 50})
    pom.set_history_length({"history_length": 0.25})
    pom.set_mag_field({"magdir": {"x": 23.8e-06, "y": -0.4e-06, "z": -39.8e-06}})
    pom.connect_port({"local": "measure/imu",   "remote": "rotorcraft/imu"})
    pom.add_measurement("imu")
    pom.connect_port({"local": "measure/mag",   "remote": "rotorcraft/mag"})
    pom.add_measurement("mag")
    pom.connect_port({"local": "measure/mocap", "remote": "optitrack/bodies/FH"})
    pom.add_measurement("mocap")

    maneuver.connect_port({"local": "state", "remote": "pom/frame/robot"})
    maneuver.set_bounds({
        "xmin": -5, "xmax": 5, "ymin": -5, "ymax": 5,
        "zmin": 0,  "zmax": 5, "yawmin": -3.14, "yawmax": 3.14,
    })
    _set_maneuver_profile("cruise")

    phynt.set_mass({"mass": MASS})

    phynt.set_wo_fc({"fc": [5.0, 5.0, 5.0, 2.0, 2.0, 2.0]})

    phynt.set_wo_thresh({"thresh": [0.3, 0.3, 0.5, 0.05, 0.05, 0.1]})

    phynt.set_af_parameters({
        "mass": 1.5,   # ridotto da 2.0: risposta più rapida alle forze del braccio
        "B":    [22.0, 22.0, 70.0, 1.0, 1.0, 0.6],  # B_z alto per stabilità con K_z=500
        "K":    [50.0, 50.0, 500.0, 2.0, 2.0, 1.0], # K_z molto alto: AF non alza il drone
        "J":    [0.1, 0.0, 0.0,
                 0.0, 0.1, 0.0,
                 0.0, 0.0, 0.05],
    })

    phynt.enable({"enable": {"wo": True, "af": False}})
    phynt.connect_port({"local": "reference",      "remote": "maneuver/desired"})
    phynt.connect_port({"local": "state",          "remote": "pom/frame/robot"})
    phynt.connect_port({"local": "wrench_measure", "remote": "nhfc/wrench_measure"})

    nhfc.set_gtmrp_geom({
        "rotors": 6, "cx": 0, "cy": 0, "cz": 0,
        "armlen": ARMLEN, "mass": MASS,
        "mbodyw": 0.1354, "mbodyh": 0.05, "mmotor": 0.07,
        "rx": -20, "ry": 0, "rz": -1,
        "cf": CF, "ct": CT,
    })
    nhfc.set_mass({"mass": MASS})
    nhfc.set_wlimit({"wmin": 16.0, "wmax": 120.0})
    nhfc.set_saturation({"sat": {"x": 1, "v": 1, "ix": 1}})
    nhfc.set_servo_gain({"gain": dict(_FLIGHT_GAINS)})
    nhfc.set_control_mode({"att_mode": "::nhfc::tilt_prioritized"})
    nhfc.set_emerg({"emerg": {"descent": 0.1, "dx": 0.10, "dq": 20, "dv": 0.3, "dw": 20}})
    nhfc.connect_port({"local": "state",         "remote": "pom/frame/robot"})
    nhfc.connect_port({"local": "reference",     "remote": "phynt/desired"})
    nhfc.connect_port({"local": "rotor_measure", "remote": "rotorcraft/rotor_measure"})
    rotorcraft.connect_port({"local": "rotor_input", "remote": "nhfc/rotor_input"})

    DYN_SERIAL = os.environ.get("DYNAMIXEL_SERIAL", "/tmp/pty-fahexa-arm")
    dynamixel.connect({"serial": DYN_SERIAL, "baud": 1000000})

def start():
    pom.log_state(os.path.join(LOG_DIR, "pom.log"))
    pom.log_measurements(os.path.join(LOG_DIR, "pom_measurements.log"))
    optitrack.set_logfile(os.path.join(LOG_DIR, "opti.log"))
    rotorcraft.log(os.path.join(LOG_DIR, "rotorcraft.log"))
    nhfc.log(os.path.join(LOG_DIR, "nhfc.log"))
    maneuver.log(os.path.join(LOG_DIR, "maneuver.log"))
    dynamixel.log(os.path.join(LOG_DIR, "dynamixel.log"))

    rotorcraft.start()
    time.sleep(0.2)

    start_pom_reader()

    nhfc.set_current_position()
    time.sleep(0.1)
    maneuver.set_current_state()
    time.sleep(0.1)
    phynt.set_current_position()
    time.sleep(0.1)

    phynt.servo(ack=True)
    nhfc.servo(ack=True)
    rotorcraft.servo(ack=True)

    try:
        phynt.log({"path": os.path.join(LOG_DIR, "phynt.log"), "decimation": 1})
    except Exception as e:
        print(f"[WARN] phynt.log fallito: {e}")

    try:
        dynamixel.servo(ack=True)
    except Exception:
        pass


def _try_call(fn):
    try:
        fn()
    except Exception as e:
        name = getattr(fn, "__qualname__", repr(fn))
        print(f"[WARN] {name}: {e}")


def stop():
    _grasp_debug_close()
    for component in (dynamixel, rotorcraft, nhfc, phynt, maneuver):
        _try_call(component.stop)
        _try_call(component.log_stop)
    stop_cube_reader()
    stop_pom_reader()
    _try_call(pom.log_stop)
    _try_call(optitrack.unset_logfile)


# dict() e' necessario perche' genomix consuma le mappe durante la serializzazione.
_FLIGHT_GAINS = {
    # Leggermente meno rigido del vecchio set, ma piu' smorzato. In crociera
    # riduce overshoot e oscillazioni dopo i goto senza perdere autorita' Z.
    "Kpxy": 7.5,  "Kpz": 7.0,
    "Kvxy": 13.5, "Kvz": 10.0,
    "Kixy": 0.8,  "Kiz": 1.0,
    "Kqxy": 7.5,  "Kqz": 0.8,
    "Kwxy": 2.4,  "Kwz": 0.55,
}

_PICK_GAINS = {
    # Manipolazione: rigidezza moderata + damping alto. Evita di inseguire
    # aggressivamente gli errori del braccio, ma smorza velocemente il residuo.
    "Kpxy": 10.5, "Kpz": 9.0,
    "Kvxy": 20.0, "Kvz": 15.0,
    # Integrale XY basso: durante la presa il braccio applica bias quasi costanti.
    # Lasciare Kixy alto accumula windup e produce overshoot proprio prima del grip.
    # Kiz resta a 1.0 per non cambiare la spinta verticale gia' integrata in hover.
    "Kixy": 0.18, "Kiz": 1.0,
    "Kqxy": 9.5,  "Kqz": 1.0,
    "Kwxy": 3.4,  "Kwz": 0.85,
}

def _set_pick_gains():
    """Gains nhfc molto smorzati per pick/place."""
    print("  [GAINS] Modalità PICK: Kpxy=10.5, Kvxy=20, Kixy=0.18, Kqxy=9.5")
    nhfc.set_servo_gain({"gain": dict(_PICK_GAINS)})
    time.sleep(0.3)   # un ciclo per l'applicazione dei nuovi gains

def _restore_flight_gains():
    """Ripristina gains nominali per il volo di trasferimento."""
    print("  [GAINS] Modalità VOLO: Kpxy=7.5, Kvxy=13.5, Kixy=0.8, Kqxy=7.5")
    nhfc.set_servo_gain({"gain": dict(_FLIGHT_GAINS)})   # copia — vedi _set_pick_gains
    time.sleep(0.3)


def _enable_af():
    """Azzera bias WO e abilita admittance filter. Richiede drone fermo."""
    print("  [AF] Azzeramento bias wrench observer (1.5 s)...")
    try:
        phynt.set_wo_zero({"duration": 1.5})
    except Exception as e:
        print(f"  [AF] set_wo_zero non supportato, uso set_wo_bias: {e}")
        phynt.set_wo_bias({"bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})
    time.sleep(1.8)   # attende completamento calibrazione
    print("  [AF] Admittance filter ON (wo=True, af=True)")
    phynt.enable({"enable": {"wo": True, "af": True}})
    time.sleep(1.0)   # l'AF ha bisogno di almeno 2-3x tau_AF
    try:
        phynt.log_stop()
    except Exception:
        pass
    try:
        phynt.log({"path": os.path.join(LOG_DIR, "phynt.log"), "decimation": 1})
        print("  [AF] phynt.log riavviato dopo set_wo_zero")
    except Exception as e:
        print(f"  [AF] phynt.log restart fallito: {e}")


def _disable_af():
    """Disabilita admittance filter (mantiene WO attivo per il logging)."""
    print("  [AF] Admittance filter OFF (wo=True, af=False)")
    phynt.enable({"enable": {"wo": True, "af": False}})
    time.sleep(0.3)


# SEQUENZA DI SIMULAZIONE
_MANEUVER_PROFILES = {
    # v/w, a/dw, jerk/ddw, snap/dddw. Valori conservativi: il planner genera
    # riferimenti piu' morbidi e nhfc non deve inseguire discontinuita' inutili.
    "takeoff": {"v": 0.45, "w": 0.55, "a": 0.55, "dw": 0.7, "j": 1.1, "ddw": 1.2, "s": 3.0, "dddw": 3.0},
    "cruise":  {"v": 0.55, "w": 0.55, "a": 0.55, "dw": 0.7, "j": 1.0, "ddw": 1.2, "s": 2.5, "dddw": 3.0},
    "carry":   {"v": 0.32, "w": 0.34, "a": 0.26, "dw": 0.32, "j": 0.45, "ddw": 0.55, "s": 1.4, "dddw": 1.6},
    "approach":{"v": 0.38, "w": 0.45, "a": 0.45, "dw": 0.60, "j": 0.90, "ddw": 1.0, "s": 2.5, "dddw": 2.8},
    "lift":    {"v": 0.34, "w": 0.40, "a": 0.40, "dw": 0.55, "j": 0.80, "ddw": 0.9, "s": 2.2, "dddw": 2.5},
    "landing": {"v": 0.24, "w": 0.30, "a": 0.28, "dw": 0.35, "j": 0.45, "ddw": 0.6, "s": 1.2, "dddw": 1.5},
    "recovery":{"v": 0.70, "w": 0.80, "a": 0.90, "dw": 1.00, "j": 1.80, "ddw": 1.8, "s": 5.0, "dddw": 5.0},
}
_active_maneuver_profile = None


def _set_maneuver_profile(name):
    """Applica limiti dinamici al planner maneuver, se supportati dal componente."""
    global _active_maneuver_profile
    if name == _active_maneuver_profile:
        return
    cfg = _MANEUVER_PROFILES[name]
    try:
        maneuver.set_velocity_limit({"v": cfg["v"], "w": cfg["w"]})
        maneuver.set_acceleration_limit({"a": cfg["a"], "dw": cfg["dw"]})
        maneuver.set_jerk_limit({"j": cfg["j"], "ddw": cfg["ddw"]})
        maneuver.set_snap_limit({"s": cfg["s"], "dddw": cfg["dddw"]})
        _active_maneuver_profile = name
        print(f"  [MANEUVER] profilo {name}: v={cfg['v']:.2f} a={cfg['a']:.2f} j={cfg['j']:.2f}")
    except Exception as e:
        print(f"  [MANEUVER] limiti profilo {name} non applicati: {e}")


def _sync_maneuver_to_current_state():
    try:
        maneuver.set_current_state()
    except Exception as e:
        print(f"  [MANEUVER] set_current_state fallito, continuo: {e}")


def _hold_arm(target=ARM_STRETCHED, duration=1.0, grip=False,
              max_vel=MAX_VEL, accel_limit=ARM_ACCEL_LIMIT):
    """Mantiene il braccio; target=None conserva il target corrente."""
    if target is not None:
        _set_arm_target(*target, label="_hold_arm")
    _run_arm_controller(duration, grip=grip, max_vel=max_vel, accel_limit=accel_limit)


def _wait_hover_stable(label, target=None, stable_time=1.0, timeout=6.0,
                       arm=None, grip=False, xy_tol=0.055, z_tol=0.160,
                       span_xy_tol=0.030, span_z_tol=0.020):
    """Aspetta che POM mostri un hovering fermo, continuando a comandare il braccio."""
    if arm is not None:
        _set_arm_target(*arm, label="_wait_hover_stable")

    print(f"  [HOVER] {label}: attendo stabilita' POM...")
    samples = []
    stable_since = None
    t0 = time.time()
    last_grip_t = 0.0
    last_dbg = 0.0

    while time.time() - t0 < timeout:
        now = time.time()
        pose = get_pom_pose()
        if pose is not None:
            samples.append((now, pose))
            while samples and now - samples[0][0] > 0.8:
                samples.pop(0)

        _step_arm_controller(max_vel=ARM_GRASP_MAX_VEL if grip else MAX_VEL,
                             accel_limit=ARM_GRASP_ACCEL_LIMIT if grip else ARM_ACCEL_LIMIT)
        if grip and now - last_grip_t >= GRIP_REFRESH_PERIOD:
            refresh_gripper_closed()
            last_grip_t = now

        stable = False
        desc = "POM non disponibile"
        if len(samples) >= 6:
            xs = [p[1][0] for p in samples]
            ys = [p[1][1] for p in samples]
            zs = [p[1][2] for p in samples]
            last = samples[-1][1]
            span_xy = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            span_z = max(zs) - min(zs)
            if target is not None:
                exy = math.hypot(last[0] - target[0], last[1] - target[1])
                ez = abs(last[2] - target[2])
            else:
                exy = 0.0
                ez = 0.0
            stable = (exy <= xy_tol and ez <= z_tol and
                      span_xy <= span_xy_tol and span_z <= span_z_tol)
            desc = f"err_xy={exy:.3f} err_z={ez:.3f} span_xy={span_xy:.3f} span_z={span_z:.3f}"

        if stable:
            if stable_since is None:
                stable_since = now
            elif now - stable_since >= stable_time:
                print(f"  [HOVER] stabile: {desc}")
                return True
        else:
            stable_since = None

        if now - last_dbg > 1.0:
            print(f"  [HOVER] {desc}")
            last_dbg = now
        time.sleep(ARM_CTRL_DT)

    print(f"  [HOVER] timeout stabilita': {desc}")
    return False


def _takeoff_request(height, duration):
    """Takeoff con retry: dopo modifiche al modello la quota iniziale puo' cambiare."""
    attempts = [
        ("takeoff", duration),
        ("takeoff", max(duration * 1.55, duration + 1.8)),
        ("recovery", max(duration * 2.0, duration + 3.2)),
    ]
    last_error = None
    for prof, dur in attempts:
        _set_maneuver_profile(prof)
        _sync_maneuver_to_current_state()
        try:
            maneuver.take_off({"height": height, "duration": dur})
            if prof != "takeoff" or abs(dur - duration) > 1e-6:
                print(f"  [MANEUVER] takeoff retry OK: profilo={prof}, duration={dur:.1f}s")
            return dur
        except Exception as e:
            last_error = e
            if "e_limits" in str(e):
                print(f"  [MANEUVER] takeoff rifiutato da {prof} duration={dur:.1f}s: e_limits")
                continue
            raise
    raise last_error


def _goto_request(x, y, z, yaw, duration, profile):
    """Invia goto con retry se il planner rifiuta i limiti cinematici."""
    if profile == "carry":
        attempts = [
            ("carry", duration),
            ("carry", max(duration * 1.35, duration + 2.2)),
            ("carry", max(duration * 1.75, duration + 4.0)),
            ("recovery", max(duration * 2.10, duration + 6.0)),
        ]
    else:
        attempts = [
            (profile, duration),
            (profile, max(duration * 1.45, duration + 1.0)),
            ("recovery", max(duration * 1.70, duration + 1.8)),
        ]
    last_error = None
    for prof, dur in attempts:
        _set_maneuver_profile(prof)
        _sync_maneuver_to_current_state()
        try:
            maneuver.goto({"x": x, "y": y, "z": z, "yaw": yaw, "duration": dur})
            if prof != profile or abs(dur - duration) > 1e-6:
                print(f"  [MANEUVER] goto retry OK: profilo={prof}, duration={dur:.1f}s")
            return dur
        except Exception as e:
            last_error = e
            if "e_limits" in str(e):
                print(f"  [MANEUVER] goto rifiutato da {prof} duration={dur:.1f}s: e_limits")
                continue
            raise
    raise last_error


def _goto(x, y, z, yaw=0.0, duration=5.0, wait=None, arm=ARM_STRETCHED,
          grip=False, profile="cruise", settle=True,
          arm_max_vel=None, arm_accel_limit=None):
    """Esegue un goto mantenendo il braccio e confermando l'hover prima di proseguire."""
    used_duration = _goto_request(x, y, z, yaw, duration, profile)
    hold_time = used_duration + 1.0 if wait is None else max(wait, used_duration + 0.8)
    if arm_max_vel is None:
        arm_max_vel = ARM_GRASP_MAX_VEL if grip else MAX_VEL
    if arm_accel_limit is None:
        arm_accel_limit = ARM_GRASP_ACCEL_LIMIT if grip else ARM_ACCEL_LIMIT
    _hold_arm(arm, hold_time, grip=grip,
              max_vel=arm_max_vel, accel_limit=arm_accel_limit)
    if settle:
        _wait_hover_stable(f"goto ({x:.2f},{y:.2f},{z:.2f})", target=(x, y, z, yaw),
                           timeout=6.0, arm=arm, grip=grip)


def _end_effector_world():
    pose = get_pom_pose()
    if pose is None:
        return None
    pos_j = _read_arm_pos()
    return _ee_from_pose_and_joints(pose, pos_j)


def _read_cube_pose_or_nominal():
    cube = get_cube_pose_cached() or _get_cube_pose()
    if cube is not None:
        return cube, True
    return (CUBE_X, CUBE_Y, CUBE_CENTER_Z_NOMINAL), False


def _median(values):
    values = sorted(values)
    n = len(values)
    if n == 0:
        return None
    mid = n // 2
    if n % 2:
        return values[mid]
    return 0.5 * (values[mid - 1] + values[mid])


def _read_cube_pose_for_post_grasp_check(duration=0.55, period=0.05):
    samples = []
    deadline = time.time() + duration
    while time.time() < deadline:
        pose = get_cube_pose_cached(max_age=0.25)
        if pose is not None:
            samples.append(pose)
        time.sleep(period)
    if not samples:
        return _get_cube_pose()
    return tuple(_median([p[i] for p in samples]) for i in range(3))


def _classify_cube_after_grasp_lift(cube_pose):
    if cube_pose is None:
        return "missing", "posa cubo non disponibile dopo la risalita"
    z = cube_pose[2]
    if z < GRASP_FALLEN_Z:
        return "fallen", f"cubo sotto il piedistallo: z={z:.3f} < {GRASP_FALLEN_Z:.3f}"
    if z <= GRASP_ON_PEDESTAL_Z:
        return "pedestal", f"cubo ancora sul piedistallo: z={z:.3f} <= {GRASP_ON_PEDESTAL_Z:.3f}"
    return "lifted", f"cubo sopra il piedistallo: z={z:.3f} > {GRASP_ON_PEDESTAL_Z:.3f}"


def _verify_cube_carried(label, duration=0.55):
    cube = _read_cube_pose_for_post_grasp_check(duration=duration, period=0.05)
    state, desc = _classify_cube_after_grasp_lift(cube)
    print(f"  [{label}] {desc}")
    _grasp_debug_log(
        "carry_check", phase=label, cube_pose=cube,
        gripper_cmd=GRIPPER_CLOSED, force=True,
    )
    if state == "lifted":
        return True
    print(f"  [{label}] cubo non piu' in presa: non apro e interrompo il place")
    return False


def _cube_between_current_gripper(cube_pose=None, require_lift=False, min_lift_z=0.35,
                                  z_offset=0.0):
    if cube_pose is None:
        cube_pose = get_cube_pose_cached() or _get_cube_pose()
    ee = _end_effector_world()
    if cube_pose is None or ee is None:
        return False, "posa cubo o EE non disponibile"
    target_z = cube_pose[2] + z_offset
    d_xy = math.hypot(cube_pose[0] - ee[0], cube_pose[1] - ee[1])
    d_z = abs(target_z - ee[2])
    z_overlap = CUBE_SIZE / 2.0 + FINGER_Z_SIZE / 2.0 - abs(ee[2] - cube_pose[2])
    lifted = (not require_lift) or cube_pose[2] > min_lift_z
    ok = (
        d_xy < VERIFY_DXY_TOL
        and d_z < VERIFY_Z_TOL
        and z_overlap >= MIN_GRIP_Z_OVERLAP
        and lifted
    )
    desc = (f"cube=({cube_pose[0]:.3f},{cube_pose[1]:.3f},{cube_pose[2]:.3f}) "
            f"EE=({ee[0]:.3f},{ee[1]:.3f},{ee[2]:.3f}) "
            f"target_z={target_z:.3f} "
            f"d_xy={d_xy:.3f}/{VERIFY_DXY_TOL:.3f} "
            f"d_z={d_z:.3f}/{VERIFY_Z_TOL:.3f} "
            f"overlap_z={z_overlap:.3f}/{MIN_GRIP_Z_OVERLAP:.3f} "
            f"lift={'OK' if lifted else 'NO'}")
    return ok, desc


def _gripper_local_cube(cube_pose=None, drone_pose=None, joints=None):
    if cube_pose is None:
        cube_pose = get_cube_pose_cached() or _get_cube_pose()
    if drone_pose is None:
        drone_pose = get_pom_pose()
    if joints is None:
        joints = _read_arm_pos()
    if cube_pose is None or drone_pose is None or joints is None or len(joints) < 3:
        return None

    j1, j2, j3 = float(joints[0]), float(joints[1]), float(joints[2])
    ee_b = _fk_arm(j1, j2, j3)
    cube_x_b, cube_y_b = _world_to_body(
        cube_pose[0] - drone_pose[0], cube_pose[1] - drone_pose[1], drone_pose[3])
    cube_z_b = cube_pose[2] - drone_pose[2]
    rel = (cube_x_b - ee_b[0], cube_y_b - ee_b[1], cube_z_b - ee_b[2])

    phi3 = j2 + j3
    c1, s1 = math.cos(j1), math.sin(j1)
    c3, s3 = math.cos(phi3), math.sin(phi3)
    x_axis = (c1 * c3, s1 * c3, -s3)
    y_axis = (-s1, c1, 0.0)
    z_axis = (c1 * s3, s1 * s3, c3)

    return (
        rel[0] * x_axis[0] + rel[1] * x_axis[1] + rel[2] * x_axis[2],
        rel[0] * y_axis[0] + rel[1] * y_axis[1] + rel[2] * y_axis[2],
        rel[0] * z_axis[0] + rel[1] * z_axis[1] + rel[2] * z_axis[2],
    )


def _cube_in_gripper_volume(cube_pose=None, drone_pose=None, joints=None):
    local = _gripper_local_cube(cube_pose, drone_pose, joints)
    if local is None:
        return False, "volume pinza non valutabile", None
    lx, ly, lz = local
    inside = (
        abs(lx) <= GRIPPER_CAPTURE_X_HALF
        and abs(ly) <= GRIPPER_CAPTURE_Y_HALF
        and abs(lz) <= GRIPPER_CAPTURE_Z_HALF
    )
    desc = (f"grip_local=({lx:.3f},{ly:.3f},{lz:.3f}) "
            f"lim=({GRIPPER_CAPTURE_X_HALF:.3f},{GRIPPER_CAPTURE_Y_HALF:.3f},"
            f"{GRIPPER_CAPTURE_Z_HALF:.3f})")
    return inside, desc, local




_GRASP_DEBUG_HEADER = [
    "t", "dt", "phase", "event",
    "cube_x", "cube_y", "cube_z",
    "drone_x", "drone_y", "drone_z", "drone_yaw",
    "ee_x", "ee_y", "ee_z",
    "j1", "j2", "j3",
    "target_x", "target_y", "target_z",
    "drone_cmd_x", "drone_cmd_y", "drone_cmd_z",
    "arm_goal_j1", "arm_goal_j2", "arm_goal_j3",
    "arm_target_j1", "arm_target_j2", "arm_target_j3",
    "arm_error", "gripper_cmd",
    "d_xy", "d_z", "overlap_z", "lifted", "between",
    "grip_local_x", "grip_local_y", "grip_local_z", "in_gripper_volume",
]
_grasp_debug_lock = threading.Lock()
_grasp_debug_file = None
_grasp_debug_t0 = None
_grasp_debug_last_sample = {}


def _csv_value(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def _grasp_debug_open():
    global _grasp_debug_file, _grasp_debug_t0
    if not GRASP_DEBUG_ENABLED:
        return None
    with _grasp_debug_lock:
        if _grasp_debug_file is None:
            path = os.path.join(LOG_DIR, "grasp_debug.csv")
            _grasp_debug_file = open(path, "w", encoding="utf-8", buffering=1)
            _grasp_debug_file.write(",".join(_GRASP_DEBUG_HEADER) + "\n")
            _grasp_debug_t0 = time.time()
            print(f"[LOG] grasp debug: {path}")
        return _grasp_debug_file


def _grasp_debug_close():
    global _grasp_debug_file
    with _grasp_debug_lock:
        if _grasp_debug_file is not None:
            _grasp_debug_file.close()
            _grasp_debug_file = None


def _ee_from_pose_and_joints(drone_pose, joints):
    if drone_pose is None or joints is None or len(joints) < 3:
        return None
    x_d, y_d, z_d, yaw_d = drone_pose
    ax, ay, az = _fk_arm(float(joints[0]), float(joints[1]), float(joints[2]))
    c, s = math.cos(yaw_d), math.sin(yaw_d)
    return x_d + ax * c - ay * s, y_d + ax * s + ay * c, z_d + az


def _grasp_debug_log(event, phase="", cube_pose=None, drone_pose=None, joints=None,
                     target_xyz=None, drone_cmd=None, arm_goal=None, arm_target=None,
                     arm_error=None, gripper_cmd=None, z_offset=0.0, force=False):
    if not GRASP_DEBUG_ENABLED:
        return
    now = time.time()
    if not force:
        key = phase or event
        last = _grasp_debug_last_sample.get(key, 0.0)
        if now - last < 1.0 / GRASP_DEBUG_HZ:
            return
        _grasp_debug_last_sample[key] = now

    if cube_pose is None:
        cube_pose = get_cube_pose_cached()
    if drone_pose is None:
        drone_pose = get_pom_pose()
    if joints is None:
        try:
            joints = _read_arm_pos()
        except Exception:
            joints = None
    if arm_target is None:
        with _arm_lock:
            arm_target = tuple(_arm_target)

    ee = _ee_from_pose_and_joints(drone_pose, joints)
    d_xy = d_z = overlap_z = None
    lifted = between = None
    grip_local = None
    in_gripper_volume = None
    if cube_pose is not None and ee is not None:
        target_z = cube_pose[2] + z_offset
        d_xy = math.hypot(cube_pose[0] - ee[0], cube_pose[1] - ee[1])
        d_z = abs(target_z - ee[2])
        overlap_z = CUBE_SIZE / 2.0 + FINGER_Z_SIZE / 2.0 - abs(ee[2] - cube_pose[2])
        lifted = cube_pose[2] > CUBE_CENTER_Z_NOMINAL + 0.12
        between = d_xy < VERIFY_DXY_TOL and d_z < VERIFY_Z_TOL and overlap_z >= MIN_GRIP_Z_OVERLAP
        in_gripper_volume, _, grip_local = _cube_in_gripper_volume(cube_pose, drone_pose, joints)

    fp = _grasp_debug_open()
    if fp is None:
        return
    t0 = _grasp_debug_t0 or now
    row = [
        now, now - t0, phase, event,
        *(cube_pose if cube_pose is not None else (None, None, None)),
        *(drone_pose if drone_pose is not None else (None, None, None, None)),
        *(ee if ee is not None else (None, None, None)),
        *(joints[:3] if joints is not None and len(joints) >= 3 else (None, None, None)),
        *(target_xyz if target_xyz is not None else (None, None, None)),
        *(drone_cmd if drone_cmd is not None else (None, None, None)),
        *(arm_goal if arm_goal is not None else (None, None, None)),
        *(arm_target if arm_target is not None else (None, None, None)),
        arm_error, gripper_cmd,
        d_xy, d_z, overlap_z, lifted, between,
        *(grip_local if grip_local is not None else (None, None, None)),
        in_gripper_volume,
    ]
    with _grasp_debug_lock:
        fp.write(",".join(_csv_value(v) for v in row) + "\n")


def _ik_joints_from_current_pose(x_obj, y_obj, z_obj, label, avg_time=0.6,
                                from_above=False):
    pose = _average_pom_pose(duration=avg_time)
    if pose is None:
        print(f"  [IK {label}] POM non disponibile")
        return None

    x_d, y_d, z_d, yaw_d = pose
    dx_b, dy_b = _world_to_body(x_obj - x_d, y_obj - y_d, yaw_d)
    dz_b = z_obj - z_d
    try:
        joints = _drone_ik(
            dx_b, dy_b, dz_b,
            elbow_up=False if from_above else None,
            strict=from_above,
        )
    except ValueError as e:
        print(f"  [IK {label}] fuori workspace: {e} "
              f"target=({x_obj:.3f},{y_obj:.3f},{z_obj:.3f}) "
              f"drone=({x_d:.3f},{y_d:.3f},{z_d:.3f})")
        return None

    D = math.hypot(math.hypot(dx_b, dy_b), dz_b - _Z_J2)
    print(f"  [IK {label}] drone=({x_d:.3f},{y_d:.3f},{z_d:.3f}) "
          f"j=({math.degrees(joints[0]):.1f}°, "
          f"{math.degrees(joints[1]):.1f}°, {math.degrees(joints[2]):.1f}°) "
          f"D={D:.3f} m")
    return joints


LIVE_IK_UPDATE_HZ = 60.0
LIVE_IK_DRONE_TAU = 0.12
LIVE_IK_CUBE_TAU = 0.04
LIVE_IK_JOINT_SLEW = 0.85      # rad/s sul target IK, prima del P velocity loop
LIVE_IK_DRONE_CLAMP_XY = 0.060 # compensa bias, non insegue oscillazioni larghe
LIVE_IK_DRONE_CLAMP_Z = 0.220 # usa quasi tutta la quota POM reale durante la presa


def _angle_lerp(current, target, alpha):
    return current + alpha * math.atan2(math.sin(target - current), math.cos(target - current))


def _pose_lerp(current, target, alpha):
    if current is None:
        return target
    return (
        current[0] + alpha * (target[0] - current[0]),
        current[1] + alpha * (target[1] - current[1]),
        current[2] + alpha * (target[2] - current[2]),
        _angle_lerp(current[3], target[3], alpha),
    )


def _xyz_lerp(current, target, alpha):
    if current is None:
        return target
    return (
        current[0] + alpha * (target[0] - current[0]),
        current[1] + alpha * (target[1] - current[1]),
        current[2] + alpha * (target[2] - current[2]),
    )


def _slew_joint_target(current, goal, max_step):
    out = list(current)
    # j1 e' angolare: usa differenza wrappata. j2/j3 restano differenze lineari.
    dj1 = math.atan2(math.sin(goal[0] - out[0]), math.cos(goal[0] - out[0]))
    out[0] += _clamp(dj1, -max_step, max_step)
    for i in (1, 2):
        out[i] += _clamp(goal[i] - out[i], -max_step, max_step)
    return list(_limit_arm_joints(out))


def _live_ik_from_filtered(target_xyz, drone_pose, x_drone_cmd, y_drone_cmd, z_drone_cmd,
                           from_above=False, clamp_xy=LIVE_IK_DRONE_CLAMP_XY,
                           clamp_z=LIVE_IK_DRONE_CLAMP_Z):
    x_d, y_d, z_d, yaw_d = drone_pose
    x_eff = x_drone_cmd + _clamp(x_d - x_drone_cmd, -clamp_xy, clamp_xy)
    y_eff = y_drone_cmd + _clamp(y_d - y_drone_cmd, -clamp_xy, clamp_xy)
    z_eff = z_drone_cmd + _clamp(z_d - z_drone_cmd, -clamp_z, clamp_z)
    dx_b, dy_b = _world_to_body(target_xyz[0] - x_eff, target_xyz[1] - y_eff, yaw_d)
    dz_b = target_xyz[2] - z_eff
    return _drone_ik(
        dx_b, dy_b, dz_b,
        elbow_up=False if from_above else None,
        strict=from_above,
    )


def _servo_arm_to_dynamic_cube(x_drone_cmd, y_drone_cmd, z_drone_cmd, offset_xyz,
                               label, duration, stable_check=False,
                               min_duration=1.0, from_above=False,
                               stable_z_offset=0.0):
    """Visual-servo IK filtrato: legge cubo spesso, muove il braccio lentamente."""
    start_cube_reader()
    pos0 = _read_arm_pos()
    target_j = [float(pos0[0]), float(pos0[1]), float(pos0[2])]
    target_j = list(_set_arm_target(*target_j))

    dt_ik = 1.0 / LIVE_IK_UPDATE_HZ
    next_ik = 0.0
    t0 = time.time()
    t_end = t0 + duration
    cube_f = None
    drone_f = None
    last_goal = None
    stable_reached = False
    last_desc = "non verificato"
    ready_since = None
    dbg_t = 0.0
    target_xyz = None
    failures = 0

    while time.time() < t_end:
        now = time.time()
        if now >= next_ik:
            next_ik = now + dt_ik
            cube = get_cube_pose_cached()
            if cube is None:
                cube = cube_f if cube_f is not None else (CUBE_X, CUBE_Y, CUBE_CENTER_Z_NOMINAL)

            pose = get_pom_pose()
            if pose is None:
                pose = (x_drone_cmd, y_drone_cmd, z_drone_cmd, 0.0)

            alpha_cube = _clamp(dt_ik / LIVE_IK_CUBE_TAU, 0.0, 1.0)
            alpha_drone = _clamp(dt_ik / LIVE_IK_DRONE_TAU, 0.0, 1.0)
            cube_f = _xyz_lerp(cube_f, cube, alpha_cube)
            drone_f = _pose_lerp(drone_f, pose, alpha_drone)
            target_xyz = (
                cube_f[0] + offset_xyz[0],
                cube_f[1] + offset_xyz[1],
                cube_f[2] + offset_xyz[2],
            )

            try:
                goal_j = _live_ik_from_filtered(
                    target_xyz, drone_f, x_drone_cmd, y_drone_cmd, z_drone_cmd,
                    from_above=from_above)
                target_j = _slew_joint_target(target_j, goal_j, LIVE_IK_JOINT_SLEW * dt_ik)
                target_j = list(_set_arm_target(*target_j))
                last_goal = goal_j
            except ValueError as e:
                failures += 1
                if failures % 10 == 1:
                    print(f"  [IK {label}] fuori workspace durante servo: {e}")

        arm_err, arm_pos = _step_arm_controller(
            max_vel=ARM_GRASP_MAX_VEL, accel_limit=ARM_GRASP_ACCEL_LIMIT)
        _grasp_debug_log(
            "sample", phase=label, cube_pose=cube_f, drone_pose=drone_f,
            joints=arm_pos, target_xyz=target_xyz,
            drone_cmd=(x_drone_cmd, y_drone_cmd, z_drone_cmd),
            arm_goal=last_goal, arm_target=tuple(target_j), arm_error=arm_err,
            z_offset=stable_z_offset if stable_check else offset_xyz[2],
        )

        if stable_check and now - t0 >= min_duration:
            cube_now = get_cube_pose_cached() or cube_f
            drone_now = get_pom_pose() or drone_f
            ok, last_desc, _ = _cube_in_gripper_volume(cube_now, drone_now, arm_pos)
            if ok:
                print(f"  [IK {label}] cubo nel volume pinza: {last_desc}")
                _grasp_debug_log(
                    "cube_inside_gripper_volume", phase=label, cube_pose=cube_now,
                    drone_pose=drone_now, joints=arm_pos, target_xyz=target_xyz,
                    drone_cmd=(x_drone_cmd, y_drone_cmd, z_drone_cmd),
                    arm_goal=last_goal, arm_target=tuple(target_j), arm_error=arm_err,
                    z_offset=stable_z_offset, force=True,
                )
                stable_reached = True
                break

        if now - dbg_t > 0.5:
            if cube_f is not None and drone_f is not None and last_goal is not None:
                print(f"  [IK {label}] cube=({cube_f[0]:.3f},{cube_f[1]:.3f},{cube_f[2]:.3f}) "
                      f"j_tgt=({target_j[0]:.2f},{target_j[1]:.2f},{target_j[2]:.2f})")
            dbg_t = now
        time.sleep(ARM_CTRL_DT)

    if stable_check:
        print(f"  [IK {label}] verifica finale: {last_desc}")
        if not stable_reached:
            _grasp_debug_log(
                "servo_timeout", phase=label, cube_pose=cube_f, drone_pose=drone_f,
                target_xyz=target_xyz, drone_cmd=(x_drone_cmd, y_drone_cmd, z_drone_cmd),
                arm_goal=last_goal, arm_target=tuple(target_j), z_offset=stable_z_offset,
                force=True,
            )
            return None
    _grasp_debug_log(
        "servo_done", phase=label, cube_pose=cube_f, drone_pose=drone_f,
        target_xyz=target_xyz, drone_cmd=(x_drone_cmd, y_drone_cmd, z_drone_cmd),
        arm_goal=last_goal, arm_target=tuple(target_j),
        z_offset=stable_z_offset if stable_check else offset_xyz[2], force=True,
    )
    return tuple(target_j) if last_goal is not None else None


def _servo_arm_to_world_target(x_drone_cmd, y_drone_cmd, z_drone_cmd, yaw_cmd,
                               target_xyz, label, duration, grip=False,
                               min_duration=0.6,
                               xy_tol=PLACE_TARGET_XY_TOL,
                               z_tol=PLACE_TARGET_Z_TOL,
                               arm_tol=PLACE_ARM_ERROR_TOL,
                               max_vel=ARM_GRASP_MAX_VEL,
                               accel_limit=ARM_GRASP_ACCEL_LIMIT,
                               joint_slew=LIVE_IK_JOINT_SLEW,
                               drone_clamp_xy=LIVE_IK_DRONE_CLAMP_XY,
                               drone_clamp_z=LIVE_IK_DRONE_CLAMP_Z,
                               from_above=False,
                               require_arm_converged=True):
    """Servo IK su un target world fisso, usando la posa POM reale del drone."""
    pos0 = _read_arm_pos()
    target_j = [float(pos0[0]), float(pos0[1]), float(pos0[2])]
    target_j = list(_set_arm_target(*target_j))

    dt_ik = 1.0 / LIVE_IK_UPDATE_HZ
    next_ik = 0.0
    t0 = time.time()
    t_end = t0 + duration
    drone_f = None
    last_goal = None
    last_desc = "non verificato"
    dbg_t = 0.0
    failures = 0
    last_grip_t = 0.0

    while time.time() < t_end:
        now = time.time()
        if now >= next_ik:
            next_ik = now + dt_ik
            pose = get_pom_pose()
            if pose is None:
                pose = (x_drone_cmd, y_drone_cmd, z_drone_cmd, yaw_cmd)

            alpha_drone = _clamp(dt_ik / LIVE_IK_DRONE_TAU, 0.0, 1.0)
            drone_f = _pose_lerp(drone_f, pose, alpha_drone)

            try:
                goal_j = _live_ik_from_filtered(
                    target_xyz, drone_f, x_drone_cmd, y_drone_cmd, z_drone_cmd,
                    from_above=from_above,
                    clamp_xy=drone_clamp_xy, clamp_z=drone_clamp_z)
                target_j = _slew_joint_target(
                    target_j, goal_j, joint_slew * dt_ik)
                target_j = list(_set_arm_target(*target_j))
                last_goal = goal_j
            except ValueError as e:
                failures += 1
                if failures % 10 == 1:
                    print(f"  [IK {label}] fuori workspace durante servo: {e}")

        arm_err, arm_pos = _step_arm_controller(
            max_vel=max_vel, accel_limit=accel_limit)
        if grip and now - last_grip_t >= GRIP_REFRESH_PERIOD:
            refresh_gripper_closed()
            last_grip_t = now

        ee = _ee_from_pose_and_joints(drone_f, arm_pos)
        if ee is not None:
            d_xy = math.hypot(target_xyz[0] - ee[0], target_xyz[1] - ee[1])
            d_z = abs(target_xyz[2] - ee[2])
            last_desc = (f"EE=({ee[0]:.3f},{ee[1]:.3f},{ee[2]:.3f}) "
                         f"target=({target_xyz[0]:.3f},{target_xyz[1]:.3f},"
                         f"{target_xyz[2]:.3f}) d_xy={d_xy:.3f}/{xy_tol:.3f} "
                         f"d_z={d_z:.3f}/{z_tol:.3f} arm_err={arm_err:.3f}/{arm_tol:.3f}")
            arm_ok = arm_err <= arm_tol
            position_ok = d_xy <= xy_tol and d_z <= z_tol
            if (now - t0 >= min_duration and last_goal is not None
                    and position_ok and (arm_ok or not require_arm_converged)):
                if arm_ok:
                    print(f"  [IK {label}] target deposito raggiunto: {last_desc}")
                else:
                    print(f"  [IK {label}] posizione pinza raggiunta, ignoro arm_err: {last_desc}")
                _grasp_debug_log(
                    "place_target_reached", phase=label, drone_pose=drone_f,
                    joints=arm_pos, target_xyz=target_xyz,
                    drone_cmd=(x_drone_cmd, y_drone_cmd, z_drone_cmd),
                    arm_goal=last_goal, arm_target=tuple(target_j),
                    arm_error=arm_err, gripper_cmd=GRIPPER_CLOSED if grip else None,
                    force=True,
                )
                return tuple(target_j)

        _grasp_debug_log(
            "sample", phase=label, drone_pose=drone_f, joints=arm_pos,
            target_xyz=target_xyz, drone_cmd=(x_drone_cmd, y_drone_cmd, z_drone_cmd),
            arm_goal=last_goal, arm_target=tuple(target_j), arm_error=arm_err,
            gripper_cmd=GRIPPER_CLOSED if grip else None,
        )

        if now - dbg_t > 0.5:
            if drone_f is not None and last_goal is not None:
                print(f"  [IK {label}] {last_desc}")
            dbg_t = now
        time.sleep(ARM_CTRL_DT)

    if last_goal is None:
        print(f"  [IK {label}] target deposito non raggiungibile")
        _grasp_debug_log(
            "place_servo_failed", phase=label, drone_pose=drone_f,
            target_xyz=target_xyz, drone_cmd=(x_drone_cmd, y_drone_cmd, z_drone_cmd),
            arm_target=tuple(target_j), force=True,
        )
        return None

    print(f"  [IK {label}] timeout servo deposito: {last_desc}")
    _grasp_debug_log(
        "place_servo_timeout", phase=label, drone_pose=drone_f,
        target_xyz=target_xyz, drone_cmd=(x_drone_cmd, y_drone_cmd, z_drone_cmd),
        arm_goal=last_goal, arm_target=tuple(target_j), force=True,
    )
    return None


def _held_cube_place_state(cube_pose, target_xy, target_z, xy_tol=PLACE_RELEASE_XY_TOL,
                           z_min=None, z_max=None):
    if cube_pose is None:
        return False, "posa cubo non disponibile"
    if z_min is None:
        z_min = target_z - PLACE_TARGET_Z_TOL
    if z_max is None:
        z_max = target_z + PLACE_TARGET_Z_TOL
    d_xy = math.hypot(cube_pose[0] - target_xy[0], cube_pose[1] - target_xy[1])
    z_ok = z_min <= cube_pose[2] <= z_max
    xy_ok = d_xy <= xy_tol
    ok = xy_ok and z_ok
    desc = (
        f"cube=({cube_pose[0]:.3f},{cube_pose[1]:.3f},{cube_pose[2]:.3f}) "
        f"target=({target_xy[0]:.3f},{target_xy[1]:.3f},{target_z:.3f}) "
        f"d_xy={d_xy:.3f}/{xy_tol:.3f} "
        f"z={cube_pose[2]:.3f} in [{z_min:.3f},{z_max:.3f}] "
        f"xy={'OK' if xy_ok else 'NO'} z={'OK' if z_ok else 'NO'}"
    )
    return ok, desc


def _servo_held_cube_to_world_target(x_drone_cmd, y_drone_cmd, z_drone_cmd, yaw_cmd,
                                     cube_target_xyz, label, duration, grip=True,
                                     min_duration=0.8,
                                     xy_tol=PLACE_RELEASE_XY_TOL,
                                     z_tol=PLACE_TARGET_Z_TOL,
                                     arm_tol=PLACE_ARM_ERROR_TOL,
                                     max_vel=MAX_VEL,
                                     accel_limit=ARM_ACCEL_LIMIT,
                                     joint_slew=PLACE_JOINT_SLEW,
                                     drone_clamp_xy=PLACE_SERVO_CLAMP_XY,
                                     drone_clamp_z=PLACE_SERVO_CLAMP_Z,
                                     from_above=False,
                                     require_arm_converged=True):
    """Servo di deposito: il target da verificare e' la posa reale del cubo, non l'EE."""
    pos0 = _read_arm_pos()
    target_j = [float(pos0[0]), float(pos0[1]), float(pos0[2])]
    target_j = list(_set_arm_target(*target_j))

    dt_ik = 1.0 / LIVE_IK_UPDATE_HZ
    next_ik = 0.0
    t0 = time.time()
    t_end = t0 + duration
    drone_f = None
    cube_f = None
    ee_minus_cube = None
    last_goal = None
    last_ee_target = None
    last_desc = "non verificato"
    dbg_t = 0.0
    failures = 0
    last_grip_t = 0.0
    target_xy = (cube_target_xyz[0], cube_target_xyz[1])

    while time.time() < t_end:
        now = time.time()
        if now >= next_ik:
            next_ik = now + dt_ik
            pose = get_pom_pose()
            if pose is None:
                pose = (x_drone_cmd, y_drone_cmd, z_drone_cmd, yaw_cmd)

            alpha_drone = _clamp(dt_ik / LIVE_IK_DRONE_TAU, 0.0, 1.0)
            drone_f = _pose_lerp(drone_f, pose, alpha_drone)

            cube_pose = get_cube_pose_cached(max_age=0.25)
            if cube_pose is None and int((now - t0) * 10) % 10 == 0:
                cube_pose = _get_cube_pose()
            if cube_pose is not None:
                alpha_cube = _clamp(dt_ik / LIVE_IK_CUBE_TAU, 0.0, 1.0)
                cube_f = _xyz_lerp(cube_f, cube_pose, alpha_cube)

            arm_for_offset = _read_arm_pos()
            ee_now = _ee_from_pose_and_joints(drone_f, arm_for_offset)
            if cube_f is not None and ee_now is not None:
                offset = (
                    ee_now[0] - cube_f[0],
                    ee_now[1] - cube_f[1],
                    ee_now[2] - cube_f[2],
                )
                if math.hypot(offset[0], offset[1]) < 0.30 and abs(offset[2]) < 0.45:
                    ee_minus_cube = _xyz_lerp(ee_minus_cube, offset, 0.20)

            if ee_minus_cube is None:
                ee_target = cube_target_xyz
            else:
                ee_target = (
                    cube_target_xyz[0] + ee_minus_cube[0],
                    cube_target_xyz[1] + ee_minus_cube[1],
                    cube_target_xyz[2] + ee_minus_cube[2],
                )
            last_ee_target = ee_target

            try:
                goal_j = _live_ik_from_filtered(
                    ee_target, drone_f, x_drone_cmd, y_drone_cmd, z_drone_cmd,
                    from_above=from_above,
                    clamp_xy=drone_clamp_xy, clamp_z=drone_clamp_z)
                target_j = _slew_joint_target(target_j, goal_j, joint_slew * dt_ik)
                target_j = list(_set_arm_target(*target_j))
                last_goal = goal_j
            except ValueError as e:
                failures += 1
                if failures % 10 == 1:
                    print(f"  [IK {label}] fuori workspace durante servo cubo: {e}")

        arm_err, arm_pos = _step_arm_controller(max_vel=max_vel, accel_limit=accel_limit)
        if grip and now - last_grip_t >= GRIP_REFRESH_PERIOD:
            refresh_gripper_closed()
            last_grip_t = now

        cube_check = get_cube_pose_cached(max_age=0.25) or cube_f
        is_release = label.startswith("PLACE_RELEASE")
        z_min = PLACE_RELEASE_Z_MIN if is_release else cube_target_xyz[2] - z_tol
        z_max = PLACE_RELEASE_Z_MAX if is_release else cube_target_xyz[2] + z_tol
        ok, desc = _held_cube_place_state(
            cube_check, target_xy, cube_target_xyz[2], xy_tol=xy_tol,
            z_min=z_min, z_max=z_max,
        )
        last_desc = f"{desc} arm_err={arm_err:.3f}/{arm_tol:.3f}"
        if last_ee_target is not None:
            last_desc += (f" ee_tgt=({last_ee_target[0]:.3f},"
                          f"{last_ee_target[1]:.3f},{last_ee_target[2]:.3f})")

        _grasp_debug_log(
            "sample", phase=label, cube_pose=cube_check, drone_pose=drone_f,
            joints=arm_pos, target_xyz=cube_target_xyz,
            drone_cmd=(x_drone_cmd, y_drone_cmd, z_drone_cmd),
            arm_goal=last_goal, arm_target=tuple(target_j), arm_error=arm_err,
            gripper_cmd=GRIPPER_CLOSED if grip else None,
        )

        arm_ok = arm_err <= arm_tol
        if (now - t0 >= min_duration and last_goal is not None
                and ok and (arm_ok or not require_arm_converged)):
            if arm_ok:
                print(f"  [IK {label}] cubo in volume deposito: {last_desc}")
            else:
                print(f"  [IK {label}] cubo in posa deposito, ignoro arm_err: {last_desc}")
            _grasp_debug_log(
                "place_cube_target_reached", phase=label, cube_pose=cube_check,
                drone_pose=drone_f, joints=arm_pos, target_xyz=cube_target_xyz,
                drone_cmd=(x_drone_cmd, y_drone_cmd, z_drone_cmd),
                arm_goal=last_goal, arm_target=tuple(target_j), arm_error=arm_err,
                gripper_cmd=GRIPPER_CLOSED if grip else None, force=True,
            )
            return tuple(target_j)

        if now - dbg_t > 0.5:
            print(f"  [IK {label}] {last_desc}")
            dbg_t = now
        time.sleep(ARM_CTRL_DT)

    print(f"  [IK {label}] timeout servo cubo deposito: {last_desc}")
    _grasp_debug_log(
        "place_cube_servo_timeout", phase=label, cube_pose=cube_check,
        drone_pose=drone_f, target_xyz=cube_target_xyz,
        drone_cmd=(x_drone_cmd, y_drone_cmd, z_drone_cmd),
        arm_goal=last_goal, arm_target=tuple(target_j), force=True,
    )
    return None


def _retreat_after_failed_grasp(approach_j, duration):
    move_arm_to(*approach_j, duration=duration, max_vel=ARM_GRASP_MAX_VEL,
                accel_limit=ARM_GRASP_ACCEL_LIMIT)
    move_arm_to(*ARM_TUCKED, duration=2.0)
    _hold_arm(ARM_STRETCHED, duration=1.2)


def _grasp_until_confirmed(x_drone_cmd, y_drone_cmd):
    """Tenta la presa visual-servo finche' il sollevamento e' confermato."""
    start_cube_reader()
    _grasp_debug_log("pick_start", phase="pick", force=True)
    time.sleep(0.15)
    x_cmd, y_cmd = x_drone_cmd, y_drone_cmd
    try:
        for attempt in range(1, MAX_GRASP_ATTEMPTS + 1):
            cube, measured = _read_cube_pose_or_nominal()
            source = "Gazebo/cache" if measured else "nominale"
            if measured and cube[2] < CUBE_CENTER_Z_NOMINAL - 0.08:
                print(f"  [IK] CUBO CADUTO z={cube[2]:.3f} m: presa abortita")
                _grasp_debug_log("cube_fallen_abort", phase="pick", cube_pose=cube, force=True)
                return None

            _grasp_debug_log(
                "attempt_start", phase=f"attempt_{attempt}", cube_pose=cube,
                drone_cmd=(x_cmd, y_cmd, PICKUP_ALT), force=True,
            )
            desired_x = cube[0] - ARM_REACH
            desired_y = cube[1]
            if math.hypot(desired_x - x_cmd, desired_y - y_cmd) > 0.012:
                print(
                    f"  [PICK XY] ricentro drone su cubo: "
                    f"({x_cmd:.3f},{y_cmd:.3f}) -> ({desired_x:.3f},{desired_y:.3f})"
                )
                _grasp_debug_log(
                    "recenter_start", phase=f"attempt_{attempt}", cube_pose=cube,
                    drone_cmd=(desired_x, desired_y, PICKUP_ALT), force=True,
                )
                _goto(
                    desired_x, desired_y, PICKUP_ALT, yaw=0.0,
                    duration=1.8, wait=2.4, arm=ARM_STRETCHED,
                    profile="approach",
                )
                x_cmd, y_cmd = desired_x, desired_y
                _grasp_debug_log(
                    "recenter_done", phase=f"attempt_{attempt}", cube_pose=cube,
                    drone_cmd=(x_cmd, y_cmd, PICKUP_ALT), force=True,
                )

            ox, oy, oz = GRASP_SCAN_OFFSETS[(attempt - 1) % len(GRASP_SCAN_OFFSETS)]
            print(f"--- TENTATIVO PRESA IK #{attempt} ({source}) ---")
            print(f"  [CUBE] ({cube[0]:.3f},{cube[1]:.3f},{cube[2]:.3f}) "
                  f"offset=({ox:.3f},{oy:.3f},{oz:.3f})")

            open_gripper()
            _hold_arm(None, duration=0.25, max_vel=ARM_GRASP_MAX_VEL,
                      accel_limit=ARM_GRASP_ACCEL_LIMIT)
            _wait_hover_stable(
                "pre-presa", target=(x_cmd, y_cmd, PICKUP_ALT, 0.0),
                stable_time=0.25, timeout=max(1.6, PRE_GRASP_SETTLE_TIME), arm=None,
            )

            print(f"  [F1] approccio obliquo dall'alto, dietro al cubo (+{IK_APPROACH_Z:.3f} m)")
            approach_j = _servo_arm_to_dynamic_cube(
                x_cmd, y_cmd, PICKUP_ALT,
                (ox + IK_APPROACH_BACKOFF, oy, oz + IK_APPROACH_Z),
                label="TOP", duration=1.8, stable_check=False,
                from_above=True,
            )
            if approach_j is None:
                print("  [F1] IK dall'alto non valida: ritento")
                _grasp_debug_log("approach_failed", phase=f"attempt_{attempt}", cube_pose=cube, force=True)
                time.sleep(0.25)
                continue

            # La chiusura parte al primo campione in cui il cubo risulta nel
            # volume locale della pinza. Il volume z e' stretto per evitare il
            # falso ingresso sul bordo superiore visto nelle ultime run.
            print(f"  [F2] avvicinamento dall'alto alla parte alta del cubo (+{IK_GRASP_Z_OFFSET:.3f} m)")
            grasp_j = _servo_arm_to_dynamic_cube(
                x_cmd, y_cmd, PICKUP_ALT,
                (ox, oy, oz + IK_GRASP_Z_OFFSET),
                label="TOP_GRASP", duration=6.0, stable_check=True,
                min_duration=0.0, from_above=True,
                stable_z_offset=IK_GRASP_Z_OFFSET,
            )
            if grasp_j is None:
                print("  [F2] non ancora allineato: continuo il tracking senza ritirare")
                _grasp_debug_log("grasp_not_aligned", phase=f"attempt_{attempt}", cube_pose=cube, force=True)
                time.sleep(0.10)
                continue

            cube_now = get_cube_pose_cached() or _get_cube_pose() or cube
            print("  [GRIP] cubo nel volume pinza: chiudo e risalgo subito alto")
            _grasp_debug_log("grip_trigger", phase=f"attempt_{attempt}", cube_pose=cube_now, gripper_cmd=GRIPPER_CLOSED, force=True)
            close_gripper()
            _hold_arm(grasp_j, duration=0.25, grip=True,
                      max_vel=ARM_GRASP_MAX_VEL, accel_limit=ARM_GRASP_ACCEL_LIMIT)
            _grasp_debug_log(
                "lift_start", phase=f"attempt_{attempt}", cube_pose=cube_now,
                drone_cmd=(x_cmd, y_cmd, POST_GRASP_CHECK_ALT), gripper_cmd=GRIPPER_CLOSED,
                force=True,
            )
            _goto(
                x_cmd, y_cmd, POST_GRASP_CHECK_ALT, yaw=0.0,
                duration=3.8, wait=4.6, arm=grasp_j, grip=True, profile="lift",
                arm_max_vel=ARM_GRASP_MAX_VEL, arm_accel_limit=ARM_GRASP_ACCEL_LIMIT,
                settle=False,
            )

            print("  [GRASP CHECK] drone alto: stendo il braccio, hover e leggo z cubo")
            _set_arm_target(*ARM_STRETCHED, label="post_grip_stretch_high")
            _wait_hover_stable(
                "post-presa alta", target=(x_cmd, y_cmd, POST_GRASP_CHECK_ALT, 0.0),
                stable_time=POST_GRASP_HOVER_TIME, timeout=2.6,
                arm=ARM_STRETCHED, grip=True, xy_tol=0.090, z_tol=0.180,
                span_xy_tol=0.050, span_z_tol=0.050,
            )
            cube_after = _read_cube_pose_for_post_grasp_check()
            _grasp_debug_log(
                "lift_done", phase=f"attempt_{attempt}", cube_pose=cube_after,
                drone_cmd=(x_cmd, y_cmd, POST_GRASP_CHECK_ALT), gripper_cmd=GRIPPER_CLOSED,
                force=True,
            )
            grasp_state, grasp_desc = _classify_cube_after_grasp_lift(cube_after)
            print(f"  [GRASP CHECK] {grasp_desc}")

            if grasp_state == "lifted":
                print("  [GRASP CHECK] PRESA CONFERMATA: il cubo e' sopra il piedistallo")
                _grasp_debug_log("grasp_confirmed", phase=f"attempt_{attempt}", cube_pose=cube_after, force=True)
                print("  [GRASP CHECK] rinforzo chiusura prima del trasferimento")
                close_gripper()
                _hold_arm(ARM_STRETCHED, duration=0.70, grip=True,
                          max_vel=0.24, accel_limit=0.28)
                return cube_after, ARM_STRETCHED

            if grasp_state == "fallen":
                print("  [GRASP CHECK] CUBO CADUTO SOTTO IL PIEDESTALLO: abort missione")
                _grasp_debug_log("cube_fallen_abort", phase=f"attempt_{attempt}", cube_pose=cube_after, force=True)
                return None

            if grasp_state == "missing":
                print("  [GRASP CHECK] posizione cubo non leggibile: abort senza riaprire la pinza")
                _grasp_debug_log("cube_pose_missing_abort", phase=f"attempt_{attempt}", cube_pose=cube_after, force=True)
                return None

            print("  [GRASP CHECK] cubo ancora alla quota del piedistallo: apro e ritento")
            _grasp_debug_log("grasp_on_pedestal_retry", phase=f"attempt_{attempt}", cube_pose=cube_after, gripper_cmd=GRIPPER_OPEN, force=True)
            open_gripper()
            print("  [RETRY SAFE] drone ancora alto: stendo completamente il braccio prima di ridiscendere")
            _set_arm_target(*ARM_STRETCHED, label="retry_safe_stretch_high")
            _hold_arm(
                ARM_STRETCHED, duration=1.4, grip=False,
                max_vel=MAX_VEL, accel_limit=ARM_ACCEL_LIMIT,
            )
            _grasp_debug_log(
                "retry_safe_stretched", phase=f"attempt_{attempt}", cube_pose=cube_after,
                drone_cmd=(x_cmd, y_cmd, POST_GRASP_CHECK_ALT), gripper_cmd=GRIPPER_OPEN,
                force=True,
            )
            _goto(
                x_cmd, y_cmd, PICKUP_ALT, yaw=0.0,
                duration=3.2, wait=4.0, arm=ARM_STRETCHED, grip=False,
                profile="approach", settle=False,
            )
            time.sleep(0.20)

        print(f"  [IK] Raggiunto MAX_GRASP_ATTEMPTS={MAX_GRASP_ATTEMPTS}: presa abortita")
        return None
    finally:
        stop_cube_reader()

def _place_nominal_drone_xy(yaw, reach=PLACE_ARM_REACH):
    return (
        PEDESTAL2_X - math.cos(yaw) * reach,
        PEDESTAL2_Y - math.sin(yaw) * reach,
    )


def _correct_place_standoff(x_cmd, y_cmd, z_cmd, yaw, arm, label="PLACE STANDOFF"):
    """Corregge il comando del drone usando POM, cosi' il target dall'alto resta nel workspace."""
    desired_x, desired_y = _place_nominal_drone_xy(yaw)
    for idx in range(2):
        pose = _average_pom_pose(duration=0.35) or get_pom_pose()
        if pose is None:
            print(f"  [{label}] POM non disponibile: uso comando nominale")
            return x_cmd, y_cmd

        dx_b, dy_b = _world_to_body(PEDESTAL2_X - pose[0], PEDESTAL2_Y - pose[1], yaw)
        err_x = desired_x - pose[0]
        err_y = desired_y - pose[1]
        err_xy = math.hypot(err_x, err_y)
        print(
            f"  [{label}] pose=({pose[0]:.3f},{pose[1]:.3f}) "
            f"target_body=({dx_b:.3f},{dy_b:.3f}) err_xy={err_xy:.3f}"
        )

        if err_xy <= 0.045 and 0.17 <= dx_b <= 0.33 and abs(dy_b) <= 0.070:
            return x_cmd, y_cmd

        step_x = _clamp(err_x, -0.22, 0.22)
        step_y = _clamp(err_y, -0.22, 0.22)
        x_cmd += step_x
        y_cmd += step_y
        print(f"  [{label}] correggo comando drone a ({x_cmd:.3f},{y_cmd:.3f})")
        _goto(
            x_cmd, y_cmd, z_cmd, yaw=yaw,
            duration=1.9, wait=2.6, arm=arm, grip=True,
            profile="approach", settle=False,
        )
    return x_cmd, y_cmd


def _place_with_arm_motion(carry_arm=None):
    print("--- DEPOSITO SPECULARE ALLA PRESA ---")
    start_cube_reader()
    carry_arm = carry_arm or ARM_STRETCHED
    print("  [CARRY] braccio teso durante il trasferimento, pinza sempre serrata")
    close_gripper()
    _hold_arm(carry_arm, duration=1.0, grip=True,
              max_vel=0.24, accel_limit=0.28)

    # Speculare alla presa con yaw ruotato di 180 gradi: nel pick il drone
    # sta a x_c - ARM_REACH e guarda il cubo; nel place sta a x_c + PLACE_ARM_REACH
    # e guarda il secondo piedistallo dopo la rotazione di pi.
    place_yaw = math.pi
    x_place = PEDESTAL2_X + PLACE_ARM_REACH
    y_place = PEDESTAL2_Y
    approach_target = (
        PEDESTAL2_X + math.cos(place_yaw) * PLACE_APPROACH_BACKOFF,
        PEDESTAL2_Y + math.sin(place_yaw) * PLACE_APPROACH_BACKOFF,
        PLACE_CUBE_APPROACH_Z,
    )
    release_target = (PEDESTAL2_X, PEDESTAL2_Y, PLACE_CUBE_RELEASE_Z)

    try:
        print(
            f"  [PLACE TRANSFER] hover speculare al pick: "
            f"drone=({x_place:.3f},{y_place:.3f},{CRUISE_ALT:.3f}) yaw=pi"
        )
        _goto(x_place, y_place, CRUISE_ALT, yaw=place_yaw,
              duration=12.0, wait=13.2, arm=carry_arm, grip=True,
              profile="carry", arm_max_vel=0.22, arm_accel_limit=0.25)
        if not _verify_cube_carried("CARRY CHECK transfer"):
            return False

        _set_pick_gains()
        print(f"  [PLACE] scendo a quota lavoro alta z={PLACE_WORK_ALT:.2f} con gripper chiuso")
        close_gripper()
        _goto(x_place, y_place, PLACE_WORK_ALT, yaw=place_yaw,
              duration=5.2, wait=6.1, arm=carry_arm, grip=True,
              profile="carry", arm_max_vel=0.22, arm_accel_limit=0.25)
        _wait_hover_stable(
            "pre-place", target=(x_place, y_place, PLACE_WORK_ALT, place_yaw),
            stable_time=0.25, timeout=1.8, arm=carry_arm, grip=True,
            xy_tol=0.110, z_tol=0.180, span_xy_tol=0.060, span_z_tol=0.050,
        )
        cube_low = _read_cube_pose_for_post_grasp_check(duration=0.30, period=0.05)
        if cube_low is None:
            print("  [PLACE CHECK] posa cubo non disponibile a quota place: procedo serrato")
        elif cube_low[2] < PEDESTAL_TOP_Z - 0.030:
            print(f"  [PLACE CHECK] cubo caduto sotto il piedistallo: z={cube_low[2]:.3f}; interrompo")
            return False
        else:
            print(
                f"  [PLACE CHECK] cubo a z={cube_low[2]:.3f}: "
                "compatibile con quota deposito, procedo al servo place"
            )

        x_place, y_place = _correct_place_standoff(
            x_place, y_place, PLACE_WORK_ALT, place_yaw, carry_arm,
            label="PLACE STANDOFF SPEC",
        )

        print(
            f"  [PLACE] servo continuo del cubo sopra il centro del piedistallo, "
            f"quota rilascio={PLACE_CUBE_RELEASE_Z:.3f} m"
        )
        place_j = None
        cube_release = None
        release_ok = False
        for place_attempt in range(1, 6):
            print(f"  [PLACE LOOP] tentativo {place_attempt}: servo cubo -> volume rilascio")
            place_j = _servo_held_cube_to_world_target(
                x_place, y_place, PLACE_WORK_ALT, place_yaw,
                release_target,
                label=f"PLACE_RELEASE_{place_attempt}", duration=5.5, grip=True,
                min_duration=0.5, xy_tol=PLACE_RELEASE_SERVO_XY_TOL, z_tol=0.060,
                arm_tol=0.300, max_vel=MAX_VEL, accel_limit=ARM_ACCEL_LIMIT,
                joint_slew=PLACE_JOINT_SLEW, drone_clamp_xy=PLACE_SERVO_CLAMP_XY,
                drone_clamp_z=PLACE_SERVO_CLAMP_Z, from_above=True,
                require_arm_converged=False,
            )
            cube_release = get_cube_pose_cached() or _get_cube_pose()
            ok, desc = _held_cube_place_state(
                cube_release, (PEDESTAL2_X, PEDESTAL2_Y), PLACE_CUBE_RELEASE_Z,
                xy_tol=PLACE_RELEASE_FINAL_XY_TOL,
                z_min=PLACE_RELEASE_Z_MIN, z_max=PLACE_RELEASE_Z_MAX,
            )
            print(f"  [PLACE RELEASE] verifica tentativo {place_attempt}: {desc}")
            if ok:
                release_ok = True
                if place_j is None:
                    place_j = tuple(_read_arm_pos())
                break
            if place_j is not None:
                print("  [PLACE LOOP] IK arrivata ma cubo non nel volume valido: continuo")
                place_j = None

            if cube_release is None:
                print("  [PLACE LOOP] posa cubo non disponibile: continuo il servo senza ricentrare")
                continue

            dx = PEDESTAL2_X - cube_release[0]
            dy = PEDESTAL2_Y - cube_release[1]
            dz_ok = PLACE_RELEASE_Z_MIN <= cube_release[2] <= PLACE_RELEASE_Z_MAX
            dxy = math.hypot(dx, dy)
            if dz_ok and dxy <= PLACE_RELEASE_FINAL_XY_TOL:
                print("  [PLACE LOOP] cubo nel volume valido: apro")
                release_ok = True
                place_j = tuple(_read_arm_pos())
                break

            step_x = _clamp(dx, -PLACE_RECENTER_MAX_STEP, PLACE_RECENTER_MAX_STEP)
            step_y = _clamp(dy, -PLACE_RECENTER_MAX_STEP, PLACE_RECENTER_MAX_STEP)
            x_place += step_x
            y_place += step_y
            print(
                f"  [PLACE LOOP] ricentro base sul cubo: err=({dx:.3f},{dy:.3f}) "
                f"cmd->({x_place:.3f},{y_place:.3f})"
            )
            _goto(
                x_place, y_place, PLACE_WORK_ALT, yaw=place_yaw,
                duration=2.0, wait=2.8, arm=tuple(_read_arm_pos()), grip=True,
                profile="approach", settle=False,
            )

        if place_j is None or not release_ok:
            print("  [PLACE] cubo non arrivato sopra il piedistallo dopo servo continuo: NON apro")
            _set_arm_target(*carry_arm, label="place_recover_no_release")
            _hold_arm(None, duration=1.0, grip=True)
            _restore_flight_gains()
            return False

        cube_release = get_cube_pose_cached() or _get_cube_pose() or cube_release
        ok, desc = _held_cube_place_state(
            cube_release, (PEDESTAL2_X, PEDESTAL2_Y), PLACE_CUBE_RELEASE_Z,
            xy_tol=PLACE_RELEASE_FINAL_XY_TOL,
            z_min=PLACE_RELEASE_Z_MIN, z_max=PLACE_RELEASE_Z_MAX,
        )
        print(f"  [PLACE RELEASE] verifica apertura: {desc}")
        if not ok:
            print("  [PLACE RELEASE] ultima lettura non valida: NON apro")
            _set_arm_target(*carry_arm, label="place_recover_bad_final")
            _hold_arm(None, duration=1.0, grip=True)
            _restore_flight_gains()
            return False

        print("  [PLACE RELEASE] cubo sopra il piedistallo: apro subito le pinze")
        _grasp_debug_log(
            "place_release_open", phase="PLACE_RELEASE", cube_pose=cube_release,
            target_xyz=release_target, gripper_cmd=GRIPPER_OPEN, force=True,
        )
        open_gripper(n_repeat=24, dt=0.045)
        _hold_arm(place_j, duration=0.65)

        print("  [PLACE EXIT] risalgo dall'alto, poi stendo il braccio")
        _goto(x_place, y_place, CRUISE_ALT, yaw=place_yaw,
              duration=3.2, wait=3.9, arm=place_j, profile="lift",
              settle=False)
        _hold_arm(ARM_STRETCHED, duration=1.0)
        _restore_flight_gains()
        return True
    finally:
        stop_cube_reader()

def _run_mission():
    print("--- BRACCIO TESO + PINZA APERTA ---")
    set_gripper(GRIPPER_OPEN, n_repeat=2, dt=0.05)
    _hold_arm(ARM_STRETCHED, duration=0.1)

    print("--- DECOLLO CON BRACCIO TESO ---")
    takeoff_duration = _takeoff_request(CRUISE_ALT, duration=3.4)
    _hold_arm(ARM_STRETCHED, duration=takeoff_duration + 0.8)
    _wait_hover_stable(
        "post-decollo", target=(0.0, 0.0, CRUISE_ALT, 0.0),
        stable_time=1.0, timeout=6.0, arm=ARM_STRETCHED,
    )

    cube, measured = _read_cube_pose_or_nominal()
    source = "Gazebo" if measured else "nominale"
    print(f"--- VOLO VERSO CUBO ({source}: {cube[0]:.3f},{cube[1]:.3f},{cube[2]:.3f}) ---")
    x_pick = cube[0] - ARM_REACH
    y_pick = cube[1]
    _goto(x_pick, y_pick, CRUISE_ALT, duration=4.2, wait=4.8, profile="cruise")

    print("--- MODALITA' PICK ---")
    _set_pick_gains()
    _enable_af()
    print(f"--- DISCESA A PICKUP_ALT={PICKUP_ALT:.2f} m ---")
    _goto(x_pick, y_pick, PICKUP_ALT, duration=3.0, wait=3.5, profile="approach")
    grasp_result = _grasp_until_confirmed(x_pick, y_pick)
    _disable_af()
    _restore_flight_gains()

    if grasp_result is None:
        print("--- PRESA FALLITA: CUBO IRRAGGIUNGIBILE - RIENTRO ---")
    else:
        _, carry_arm = grasp_result
        print("--- PRESA CONFERMATA: PASSO AL TRASFERIMENTO ---")
        _place_with_arm_motion(carry_arm)

    print("--- RIENTRO ---")
    _goto(0.0, 0.0, CRUISE_ALT, duration=5.0, wait=5.8, profile="cruise")
    _hold_arm(ARM_STRETCHED, duration=1.0)

    print("--- ATTERRAGGIO ---")
    _goto(
        0.0, 0.0, 0.05, duration=4.8, wait=6.2,
        arm=ARM_STRETCHED, profile="landing", settle=False,
    )
    print("--- OPERAZIONE COMPLETATA ---")


def simulation():
    print("--- INIZIALIZZAZIONE ---")
    setup()

    def _abort(sig, _frame):
        print(f"\n[ABORT] Segnale {sig} ricevuto")
        raise SystemExit(128 + sig)

    signal.signal(signal.SIGINT, _abort)
    signal.signal(signal.SIGTERM, _abort)

    try:
        print("--- AVVIO MOTORI E CONTROLLORI ---")
        start()
        _run_mission()
    finally:
        stop()


if __name__ == "__main__":
    simulation()
