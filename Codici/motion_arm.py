import genomix
import math
import os
import subprocess
import time

DYN_SERIAL = os.environ.get("DYNAMIXEL_SERIAL", "/tmp/pty-ground-arm")
DYN_BAUD = 1000000
LOG_DIR = "/tmp/arm_logs"

KP      = 4.0   # rad/s per rad of error
MAX_VEL = 2.0   # rad/s clamp

# Arm geometry in model-local frame (origin at ground, Z up)    
#   J1 (yaw)   at z = 0.15 m
#   J2 (pitch) at z = 0.30 m   ← shoulder
#   Upper arm (J2→J3): L2 = 0.30 m
#   Forearm   (J3→EE): L3 = 0.25 m
#   Home EE position: (0.55, 0, 0.30)
Z_J2 = 0.30
L2   = 0.30
L3   = 0.25

g = genomix.connect()
g.rpath("/opt/openrobots/lib/genom/pocolibs/plugins")
dynamixel = g.load("dynamixel")

target = [0.0, 0.0, 0.0]   # current joint targets [j1, j2, j3]


# ---------------------------------------------------------------------------
# Inverse kinematics
# ---------------------------------------------------------------------------

def ik(px, py, pz, elbow_up=False):
    """
    Analytical IK for the 3-DOF arm.

    Input:  (px, py, pz) desired end-effector position in model-local frame [m].
    Output: (j1, j2, j3) Gazebo joint angles [rad].

    Model-local frame: origin at arm base (ground level), +X = arm default
    forward direction, +Z = up.
    Home EE: (0.55, 0, 0.30) → (j1=0, j2=0, j3=0).

    Raises ValueError if the target is outside the reachable workspace.
    """
    j1 = math.atan2(py, px)
    R  = math.hypot(px, py)     # horizontal reach from base axis
    h  = pz - Z_J2              # height above shoulder joint (signed)
    D  = math.hypot(R, h)       # distance shoulder → target

    d_max = L2 + L3 - 1e-4
    d_min = abs(L2 - L3) + 1e-4
    if D > d_max:
        raise ValueError(f"Target too far: {D:.3f} m > max {d_max:.3f} m")
    if D < d_min:
        raise ValueError(f"Target too close: {D:.3f} m < min {d_min:.3f} m")

    # Elbow angle via law of cosines
    cos_j3 = (D**2 - L2**2 - L3**2) / (2.0 * L2 * L3)
    cos_j3 = max(-1.0, min(1.0, cos_j3))
    j3_geo  = math.acos(cos_j3)                          # ∈ [0, π]
    j3_std  = j3_geo if elbow_up else -j3_geo             # standard convention

    # Shoulder angle
    k1 = L2 + L3 * math.cos(j3_std)
    k2 = L3 * math.sin(j3_std)
    j2_std = math.atan2(h, R) - math.atan2(k2, k1)

    # Convert standard (positive=up) to Gazebo convention (positive=down)
    return j1, -j2_std, -j3_std


def fk(j1, j2, j3):
    """Forward kinematics: joint angles → end-effector position [m]."""
    # In Gazebo convention: R_y(theta) maps (1,0,0) to (cos,-sin_perp,0)
    # Absolute angle of forearm from horizontal
    phi2 = j2          # angle of upper arm  (positive = down in Gazebo)
    phi3 = j2 + j3     # angle of forearm    (cumulative)
    R  = L2 * math.cos(phi2) + L3 * math.cos(phi3)
    dz = -(L2 * math.sin(phi2) + L3 * math.sin(phi3))   # negate: Gazebo positive=down
    pz = Z_J2 + dz
    px = R * math.cos(j1)
    py = R * math.sin(j1)
    return px, py, pz


# ---------------------------------------------------------------------------
# Controller helpers
# ---------------------------------------------------------------------------

def read_pos():
    state = dynamixel.motors()
    return state.get("motors", {}).get("position", [0.0] * 8)


def run_controller(duration):
    """Run P velocity controller for `duration` seconds toward current target."""
    t_end = time.time() + duration
    while time.time() < t_end:
        pos = read_pos()
        vel = [0.0] * 8
        for i in range(3):
            v = KP * (target[i] - pos[i])
            vel[i] = max(-MAX_VEL, min(MAX_VEL, v))
        dynamixel.set_velocity({"velocity": vel})
        time.sleep(0.02)   # 50 Hz


def move_to(j1=0.0, j2=0.0, j3=0.0, duration=3.0):
    """Move to joint angles [rad] and hold for duration [s]."""
    target[0] = j1
    target[1] = j2
    target[2] = j3
    run_controller(duration)
    pos = read_pos()
    ex, ey, ez = fk(pos[0], pos[1], pos[2])
    print(f"  joints: j1={pos[0]:.3f}  j2={pos[1]:.3f}  j3={pos[2]:.3f}"
          f"   EE≈({ex:.3f}, {ey:.3f}, {ez:.3f})")


def move_to_xyz(px, py, pz, duration=3.0, elbow_up=False):
    """Move end effector to Cartesian position [m] in model-local frame."""
    try:
        j1, j2, j3 = ik(px, py, pz, elbow_up)
    except ValueError as e:
        print(f"  IK error: {e}")
        return
    deg = math.degrees
    print(f"  → ({px:.2f},{py:.2f},{pz:.2f})  "
          f"IK: j1={deg(j1):.1f}°  j2={deg(j2):.1f}°  j3={deg(j3):.1f}°")
    move_to(j1, j2, j3, duration)


# ---------------------------------------------------------------------------
# Gripper control
#
# Le dita (left_finger_joint, right_finger_joint) sono giunti PRISMATICI.
# Il plugin dxsim-gazebo.so li rifiuta perché chiama Joint::Type() e accetta
# solo giunti revolute (i Dynamixel fisici sono motori rotazionali).
# Il sorgente di dxsim non è disponibile nel container Docker del lab
# (pacchetto robotpkg-dxsim-gazebo v2.1), quindi non è ricompilabile.
#
# Soluzione: plugin built-in di Gazebo Ionic (JointPositionController),
# configurato in ground_arm/model.sdf. Il comando arriva via "gz topic"
# invece che dal componente GenoM dynamixel.
# ---------------------------------------------------------------------------

GRIPPER_OPEN   = 0.0    # m  (dita completamente aperte)
GRIPPER_CLOSED = 0.04   # m  (dita chiuse, limite da SDF)

def _gz_pub(topic, value):
    subprocess.run(
        ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", f"data: {value}"],
        check=False
    )

def set_gripper(pos):
    """Invia posizione [m] a entrambe le dita via gz topic. 0=aperto, 0.04=chiuso."""
    pos = max(GRIPPER_OPEN, min(GRIPPER_CLOSED, pos))
    _gz_pub("/model/ground_arm/joint/left_finger_joint/cmd_pos",  pos)
    _gz_pub("/model/ground_arm/joint/right_finger_joint/cmd_pos", pos)

def open_gripper():
    set_gripper(GRIPPER_OPEN)

def close_gripper():
    set_gripper(GRIPPER_CLOSED)


# ---------------------------------------------------------------------------
# Setup / teardown
# ---------------------------------------------------------------------------

def setup():
    os.makedirs(LOG_DIR, exist_ok=True)
    dynamixel.connect({"serial": DYN_SERIAL, "baud": DYN_BAUD})
    time.sleep(0.5)


def stop():
    try:
        dynamixel.stop()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Demo sequence in Cartesian space
# ---------------------------------------------------------------------------

def arm_sequence():
    """
    Demo pick-and-place.

    Geometria (local frame del ground_arm, origine alla base, yaw=pi rispetto world):
      Cubo arancione sul tavolo:
        world  (1.08,  0, 0.225)  →  local (0.42, 0, 0.225)
      EE di presa al local (0.38, 0, 0.225):
        le dita si estendono +0.04 m in X locale → arrivano a x=0.42 dove c'è il cubo.

    Nota: home (0.55, 0, 0.30) supera di 1e-4 m il limite cinematico (L2+L3=0.55).
    Si usa (0.50, 0, 0.30) come posizione neutra sicura.
    """
    print("\n  [apri gripper]")
    open_gripper()
    time.sleep(0.5)

    print("\n  [home sicuro]")
    move_to_xyz(0.50, 0.00, 0.30, duration=3.0)

    print("\n  [sopra il cubo]")
    move_to_xyz(0.38, 0.00, 0.38, duration=3.0)

    print("\n  [scendi al cubo]")
    move_to_xyz(0.38, 0.00, 0.240, duration=3.0)

    print("\n  [chiudi gripper]")
    close_gripper()
    time.sleep(1.5)

    print("\n  [solleva]")
    move_to_xyz(0.38, 0.00, 0.45, duration=2.5)

    print("\n  [swing verso deposito]")
    move_to_xyz(0.20, 0.30, 0.42, duration=3.0)

    print("\n  [apri gripper - rilascia]")
    open_gripper()
    time.sleep(1.0)

    print("\n  [torna a home]")
    move_to_xyz(0.50, 0.00, 0.30, duration=3.0)


def simulation():
    print("--- SETUP ---")
    setup()

    print("--- HOLD NEUTRAL (3s) ---")
    move_to_xyz(0.50, 0.0, 0.30, duration=3.0)

    print("--- ARM SEQUENCE (Cartesian) ---")
    arm_sequence()

    print("\n--- DONE - Ctrl+C to exit ---")
    try:
        while True:
            run_controller(1.0)
    except KeyboardInterrupt:
        print("Stop.")
        stop()


if __name__ == "__main__":
    simulation()
