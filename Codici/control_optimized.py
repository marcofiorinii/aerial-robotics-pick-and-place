import genomix
import os
import re
import json
import signal
import sys
import subprocess
import time
import math
import threading
import shutil

# =============================================================================
# CONNESSIONE AL MIDDLEWARE
# =============================================================================
# genomix è il bridge Python → GenoM3/pocolibs. Ogni componente (nhfc, pom…)
# gira come processo separato; qui ci colleghiamo a tutti.
g = genomix.connect()
g.rpath(os.environ["HOME"] + "/openrobots/lib/genom/pocolibs/plugins")

optitrack = g.load("optitrack")
rotorcraft = g.load("rotorcraft")
pom       = g.load("pom")
maneuver  = g.load("maneuver")
nhfc      = g.load("nhfc")
phynt     = g.load("phynt")
dynamixel = g.load("dynamixel")

# =============================================================================
# PARAMETRI GLOBALI
# =============================================================================

# Directory fissa per i log — contiene sempre e solo l'ultima simulazione.
# Viene ripulita ad ogni avvio così i file vecchi non si accumulano.
# Path fisso (no timestamp) per rispettare il limite di 64 caratteri di GenoM.
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
LOG_DIR = os.path.join(BASE_DIR, 'logs', 'last')
if os.path.isdir(LOG_DIR):
    shutil.rmtree(LOG_DIR)
os.makedirs(LOG_DIR, exist_ok=True)
print(f"[LOG] directory: {LOG_DIR}")

ROBOT_SERIAL = "/tmp/pty-fahexa"
# Massa dinamica reale del modello Gazebo:
# base 2.30 + arm/gripper 0.19 + 6 rotori*0.07 = 2.91 kg.
# Il valore precedente (3.2 kg) sovrastimava il peso di circa il 10%: con Kiz
# ridotto in modalità PICK il drone restava 3-5 cm sopra il riferimento.
MASS   = 2.91         # kg — usato da nhfc e phynt per la dinamica
ARMLEN = 0.38998      # m  — lunghezza braccio rotore per la matrice di allocazione
CF     = 9.9016e-4    # coefficiente di portanza rotori
CT     = 1.9e-5       # coefficiente di coppia rotori

# ID joint sul bus Dynamixel (j1=yaw braccio, j2=spalla, j3=gomito, lf/rf=dita)
J1_ID = 1
J2_ID = 2
J3_ID = 3
LF_ID = 4
RF_ID = 5

# =============================================================================
# GRIPPER — controllo via gz topic
# =============================================================================
# I giunti left_finger_joint e right_finger_joint sono PRISMATICI.
# Il componente GenoM dxsim-gazebo ignora i giunti prismatici perché simula
# solo Dynamixel rotazionali. Soluzione: nel file fah_arm/model.sdf è
# configurato un JointPositionController di Gazebo Ionic che accetta comandi
# direttamente via topic gz. Usiamo subprocess per pubblicare sul topic.
#
# GRIPPER_OPEN = -0.02: valore negativo perché l'asse prismatico punta verso
# Y=0 (chiusura). Con pos=-0.02 le dita si spostano di 2cm all'esterno rispetto
# alla posizione di riposo (0), portandole a y=±0.055m dal centro del braccio.
# Cubo largo 5cm → ±0.025m dal centro → clearance 3cm/lato invece di 1cm.
# Il SDF è stato aggiornato con lower=-0.02 per permettere questa posizione.
GRIPPER_OPEN   = -0.02  # m — dita aperte (2cm oltre la pos. di riposo)
GRIPPER_CLOSED =  0.03  # m — dita chiuse (corsa massima dal model.sdf)

GRIP_DXY_TOL      = 0.015  # m — finestra stretta: il cubo è largo 5 cm (era 0.035, causava prese sul bordo)
GRIP_Z_TOL        = 0.015  # m — era 0.035
GRIP_STABLE_SECS  = 1.0    # s — EE dentro finestra prima di chiudere (era 0.6)
GRIP_MAX_WAIT     = 6.0    # s — extra hold massimo prima di rinunciare

# Numero massimo di tentativi di presa prima di abortire la missione.
# Evita il loop infinito in _grasp_until_confirmed.
MAX_GRASP_ATTEMPTS = 7

def set_gripper(pos, n_repeat=6, dt=0.12):
    """
    Aziona entrambe le dita, ripetendo il comando n_repeat volte.

    Problema: gz topic pubblica un singolo messaggio e termina. Se Gazebo è
    impegnato con un physics step nel momento della pubblicazione, il messaggio
    può essere perso. Con due Popen paralleli il rischio raddoppia (race con il
    subscriber del JointPositionController non ancora pronto).

    Soluzione: ripetere il comando n_repeat=6 volte a dt=0.12s di intervallo
    → finestra totale 0.72s → almeno 3-4 messaggi sicuramente recepiti.
    Il JointPositionController è idempotente: ricevere lo stesso target più
    volte non crea problemi, mantiene semplicemente la posizione.
    """
    pos = max(GRIPPER_OPEN, min(GRIPPER_CLOSED, pos))
    cmd = ["gz", "topic", "-m", "gz.msgs.Double", "-p", f"data: {pos}"]
    for i in range(n_repeat):
        p1 = subprocess.Popen(
            cmd + ["-t", "/model/fahexa/joint/left_finger_joint/cmd_pos"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        p2 = subprocess.Popen(
            cmd + ["-t", "/model/fahexa/joint/right_finger_joint/cmd_pos"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        p1.wait(); p2.wait()
        if i < n_repeat - 1:
            time.sleep(dt)

def open_gripper():
    set_gripper(GRIPPER_OPEN)

def close_gripper():
    set_gripper(GRIPPER_CLOSED)


def refresh_gripper_closed():
    """
    Rinforza la chiusura con un singolo comando rapido.
    Lanciato in un thread daemon per non bloccare il loop a 50 Hz del braccio:
    set_gripper con n_repeat=1 chiama p1.wait()/p2.wait() che prendono ~50 ms
    e farebbero slittare il timing del controller.
    """
    t = threading.Thread(
        target=set_gripper, args=(GRIPPER_CLOSED,), kwargs={"n_repeat": 1, "dt": 0.0},
        daemon=True)
    t.start()


def _get_cube_pose():
    """
    Legge la posa corrente del cubo direttamente dal simulatore Gazebo.
    Ritorna (x, y, z) del centro cubo in coordinate world, o None se fallisce.

    Strategia: tenta prima la decodifica JSON (più robusta e non dipendente
    dalla formattazione del testo protobuf). Se il flag --json non è supportato
    dalla versione di gz installata, cade sul parsing regex del testo protobuf.
    """
    topic_args = ["-e", "-n", "1", "-t", "/world/hexa_phynt/pose/info"]

    # --- tentativo JSON ---
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
        pass  # gz versione senza --json, o parsing fallito → fallback regex

    # --- fallback: parsing testo protobuf ---
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


# =============================================================================
# POSIZIONI WORLD PER LA DEMO PICK-AND-PLACE
# =============================================================================

# Il cubo arancione nel world ha centro a (1.08, 0, 0.225).
# Con il nuovo model.sdf le collisioni coincidono con il visual: la presa deve
# restare piu' alta del centro cubo per non strisciare sul piedistallo.
CUBE_X, CUBE_Y, CUBE_Z   = 1.08, 0.0, 0.260
PEDESTAL2_X, PEDESTAL2_Y = -1.0, 0.0
PEDESTAL2_Z              = 0.260   # stessa geometria del piedistallo 1

# Il drone si posiziona ARM_REACH m DIETRO il target (asse +X world con yaw=0)
# in modo che il braccio si estenda in avanti verso l'oggetto.
# Con ARM_REACH=0.27 m e PICKUP_ALT=0.62 m il drone resta piu' alto e un filo
# piu' arretrato rispetto al cubo, coerente con le collisioni del nuovo SDF.
CRUISE_ALT = 1.00   # quota di crociera [m]
PICKUP_ALT = 0.62   # quota per presa IK [m]
ARM_REACH  = 0.27   # m


# Nuovo workflow: durante decollo, traslazione e discesa il braccio resta teso
# nella posizione di riposo. La pinza, non il centro del drone, viene portata
# sopra il cubo: con j=(0,0,0) l'EE e' circa 35 cm davanti e 15 cm sotto al drone.
ARM_STRETCHED = (0.0, 0.0, 0.0)
CUBE_CENTER_Z_NOMINAL = 0.225
CUBE_SIZE = 0.05
GRASP_EE_Z_OFFSET = 0.0          # pinza centrata sull'altezza del cubo
GRASP_APPROACH_CLEARANCE = 0.14  # m sopra quota di presa, sempre con braccio teso
MIN_GRASP_DRONE_ALT = 0.36       # evita di scendere sotto la geometria sicura del modello
POST_GRASP_CHECK_ALT = 0.78      # quota sufficiente per distinguere cubo preso/non preso
PEDESTAL_TOP_Z = 0.20
PLACE_SURFACE_CLEARANCE = 0.012  # cubo circa 1 cm sopra il piedistallo prima dell'apertura
PLACE_EE_Z_OFFSET = 0.0
VERIFY_DXY_TOL = 0.060           # tolleranza cubo-pinza dopo la salita
VERIFY_Z_TOL = 0.080
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
IK_GRASP_Z_OFFSET = 0.035   # chiude piu' alto: evita piedistallo e collisioni SDF

# =============================================================================
# SICUREZZA — ZONA DI ESCLUSIONE DEI PROPULSORI
# =============================================================================
# Il rischio elica si verifica SOLO quando j2 < 0, cioè quando il primo link
# del braccio punta verso l'ALTO (convezione Gazebo: j2 > 0 = verso il basso).
# Un braccio che scende dall'alto verso il basso (j2 decrescente da >0 a 0)
# è sempre sicuro perché rimane sotto il piano dei rotori.
#
# Strategia:
#   1. _drone_ik() lancia ValueError se il risultato ha j2 < J2_HARD_MIN.
#   2. _step_arm_controller() clamp sempre j2 a >= J2_HARD_MIN prima di
#      inviare i comandi — ultima linea di difesa anche se il target è errato.
#   3. In caso di presa fallita, il braccio torna ad ARM_TUCKED (j2=60° giù)
#      invece che ARM_STRETCHED (j2=0 = orizzontale) per garantire clearance
#      durante la transizione verso qualsiasi target successivo.

J2_HARD_MIN = -0.8   # rad — limite estremo: primo link oltre -46° sopra l'orizzontale.
                     # Per la presa normale j2 ≈ -0.31 (18° sopra l'orizzontale) è valido e sicuro:
                     # il gomito si piega in giù e l'EE raggiunge il target sotto il drone.
                     # Solo valori molto negativi (< -0.8) portano il link1 in zone pericolose.

# Posizione di ritiro sicura dopo presa fallita.
# FK con j=(0, π/3, π/4): R≈0.13 m, z≈-0.41 m → molto al di sotto dei rotori.
# Usata come punto di passaggio intermedio prima di tornare ad ARM_STRETCHED,
# per evitare che la traiettoria di ritorno passi accidentalmente per j2<0.
ARM_TUCKED = (0.0, math.pi / 3, math.pi / 4)

# Tempo di assestamento extra PRIMA del calcolo dell'IK di presa.
# Il drone deve smettere di oscillare dopo il volo di avvicinamento/discesa
# prima di computare l'IK: un errore di posizione di 5 cm causa un errore
# sull'angolo j3 di circa 0.15 rad → EE fuori target di 3 cm.
PRE_GRASP_SETTLE_TIME = 2.5   # s

# =============================================================================
# CINEMATICA BRACCIO fah_arm
# =============================================================================
# Struttura (dal model.sdf):
#   J1 (yaw,  z = -0.10 m) — ruota il piano di lavoro del braccio
#   J2 (pitch,z = -0.15 m) — spalla: positivo = arm verso il basso (conv. Gazebo)
#   link2 L2 = 0.15 m
#   J3 (pitch)             — gomito: positivo = piegato verso il basso
#   link3+gripper L3 = 0.20 m (dalla J3 al centro delle dita)
_Z_J2 = -0.15   # m, sotto la base del drone
_L2   = 0.15    # m
_L3   = 0.20    # m
_J3_LIMIT = 2.35  # rad, coerente con arm_joint_3 nel model.sdf (circa 135 gradi)

def _drone_ik(px_b, py_b, pz_b, elbow_up=False):
    """
    IK analitica 2-link planare per fah_arm, frame body drone.
    Restituisce (j1, j2, j3) in convenzione Gazebo (j2/j3 > 0 = verso il basso).
    Lancia ValueError se il target è fuori workspace D ∈ [d_min, d_max].

    d_min ≈ 0.143 m con j3=±135° (gomito piu' chiuso del vecchio limite ±90°)
    d_max = L2+L3 = 0.350 m (braccio completamente disteso)
    """
    j1 = math.atan2(py_b, px_b)
    R  = math.hypot(px_b, py_b)
    h  = pz_b - _Z_J2                    # altezza target rispetto a J2
    D  = math.hypot(R, h)
    d_max = _L2 + _L3 - 1e-4
    d_min = math.sqrt(_L2**2 + _L3**2 + 2.0 * _L2 * _L3 * math.cos(_J3_LIMIT)) - 1e-4
    if D > d_max:
        raise ValueError(f"troppo lontano D={D:.3f} > {d_max:.3f}")
    if D < d_min:
        raise ValueError(f"troppo vicino D={D:.3f} < {d_min:.3f}")
    cos_j3 = (D**2 - _L2**2 - _L3**2) / (2.0 * _L2 * _L3)
    cos_j3 = max(-1.0, min(1.0, cos_j3))
    j3_geo = math.acos(cos_j3)
    j3_std = j3_geo if elbow_up else -j3_geo
    k1 = _L2 + _L3 * math.cos(j3_std)
    k2 = _L3 * math.sin(j3_std)
    j2_std = math.atan2(h, R) - math.atan2(k2, k1)
    # Inversione segno: convenzione Gazebo (positivo = giù) vs. math standard (positivo = su)
    j2_gz = -j2_std
    j3_gz = -j3_std
    # Nessun check j2 qui: il valore j2_gz ≈ -0.31 è normale per la presa
    # (primo link leggermente sopra l'orizzontale, gomito piegato verso il basso).
    # Il clamp estremo è gestito in _step_arm_controller.
    return j1, j2_gz, j3_gz


def _fk_arm(j1, j2, j3):
    """Cinematica diretta: angoli Gazebo → posizione EE in frame body drone."""
    phi2 = j2
    phi3 = j2 + j3
    R  = _L2 * math.cos(phi2) + _L3 * math.cos(phi3)
    dz = -(_L2 * math.sin(phi2) + _L3 * math.sin(phi3))
    return R * math.cos(j1), R * math.sin(j1), _Z_J2 + dz


def _cube_between_fingers(x_obj, y_obj, z_obj):
    """
    Usa FK + pom per stimare la posizione EE in world frame e verificare se
    è abbastanza vicino al target prima di chiudere il gripper.
    Soglie strette: il cubo e' largo 5 cm, quindi 7 cm XY permetteva chiusure
    quando le dita erano gia' fuori bersaglio.
    Ritorna (ok, stringa_debug).
    """
    pose = get_pom_pose()
    if pose is None:
        return False, "pom non disponibile"
    x_d, y_d, z_d, yaw_d = pose
    pos_j = _read_arm_pos()
    ax, ay, az = _fk_arm(pos_j[0], pos_j[1], pos_j[2])
    c, s  = math.cos(yaw_d), math.sin(yaw_d)
    ee_x  = x_d + ax * c - ay * s
    ee_y  = y_d + ax * s + ay * c
    ee_z  = z_d + az
    d_xy  = math.hypot(ee_x - x_obj, ee_y - y_obj)
    d_z   = abs(ee_z - z_obj)
    ok    = d_xy < GRIP_DXY_TOL and d_z < GRIP_Z_TOL
    desc  = (f"EE_world=({ee_x:.3f},{ee_y:.3f},{ee_z:.3f}) "
             f"d_xy={d_xy:.3f}/{GRIP_DXY_TOL:.3f} "
             f"d_z={d_z:.3f}/{GRIP_Z_TOL:.3f} → {'CHIUDO' if ok else 'SALTO'}")
    return ok, desc


def _world_to_body(dx_w, dy_w, yaw):
    """Ruota un offset orizzontale dal frame world al frame body (rotazione yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx_w + s * dy_w, -s * dx_w + c * dy_w


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))

# =============================================================================
# LETTURA POSIZIONE DRONE DA LOG DI POM
# =============================================================================
# pom pubblica la stima dello stato anche su file di log (1 kHz).
# Usiamo il log invece del port GenoM pom/frame/robot perché il port restituisce
# zeri quando i campi "optional" non sono popolati (bug noto), mentre il log
# contiene sempre tutti i valori numerici.
#
# Formato colonne del log:
#   ts(0) i(1) posp(2) attp(3) velp(4) avelp(5) accp(6)
#   x(7) y(8) z(9) roll(10) pitch(11) yaw(12) vx(13) ...
#
# Leggiamo gli ultimi n_avg campioni (≈ n_avg ms a 1 kHz) e ne facciamo la
# media per filtrare il rumore di misurazione dell'optitrack.

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

# =============================================================================
# THREAD DI LETTURA POM IN BACKGROUND
# =============================================================================
# Problema: leggere il file di log ad ogni iterazione del loop di controllo
# (50 Hz) introduce latenza variabile da I/O che disturba il timing del
# controller del braccio.
# Soluzione: un thread daemon legge pom a ~200 Hz e aggiorna una variabile
# condivisa (_pom_cache). Il loop del braccio legge solo dalla cache (RAM),
# senza alcuna operazione su disco nella hot loop.
# La media n_avg=50 corrisponde a ~50 ms di storia a 1 kHz → buona riduzione
# del rumore senza introdurre ritardo significativo per movimenti lenti del drone.

_pom_cache = None
_pom_lock  = threading.Lock()
_pom_stop  = threading.Event()

def _pom_reader_loop():
    while not _pom_stop.is_set():
        pose = _read_drone_pose(n_avg=50)
        with _pom_lock:
            global _pom_cache
            _pom_cache = pose
        time.sleep(0.005)   # 200 Hz

def start_pom_reader():
    _pom_stop.clear()
    t = threading.Thread(target=_pom_reader_loop, daemon=True)
    t.start()

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

# =============================================================================
# CONTROLLER BRACCIO (P in velocità)
# =============================================================================
# dynamixel.set_position_effort() non funziona: il componente GenoM dynamixel
# ha un loop interno che sovrascrive i target di posizione ogni ciclo.
# Soluzione: controller proporzionale in velocità (come in motion_arm.py):
#   vel[i] = KP_ARM * (target[i] - pos[i]),  clampato a ±MAX_VEL

KP_ARM      = 0.7   # guadagno P [rad/s / rad]
# Ridotto da 1.0 a 0.7: accelerazione braccio più lenta → meno forze di
# reazione sul drone → meno disturbo durante la fase di tracking.
MAX_VEL     = 0.6   # velocità massima [rad/s]
# Ridotto da 0.8 a 0.6: braccio più lento → forze di reazione più piccole.
# Le forze di reazione sul drone sono proporzionali all'accelerazione del
# braccio: F_reazione ≈ m_arm × a_arm. Con MAX_VEL minore, le accelerazioni
# nelle fasi di avvio/frenata sono ridotte → drone meno disturbato.
_arm_target = [0.0, 0.0, 0.0]   # target corrente [j1, j2, j3], aggiornato dal tracking
_arm_lock   = threading.Lock()   # protegge _arm_target da accessi concorrenti

def _read_arm_pos():
    state = dynamixel.motors()
    return state.get("motors", {}).get("position", [0.0] * 8)

def _step_arm_controller():
    """Un passo del controller P in velocità. Ritorna (errore massimo, posizioni)."""
    pos = _read_arm_pos()
    vel = [0.0] * 8
    max_err = 0.0
    with _arm_lock:
        target = list(_arm_target)  # snapshot atomico dei 3 target
    # Sicurezza propulsori — ultima linea di difesa:
    # blocca solo angoli estremi (< -0.8 rad ≈ -46° sopra l'orizzontale).
    # j2 ≈ -0.31 (normale per la presa) è lasciato passare.
    if target[1] < J2_HARD_MIN:
        print(f"[SAFETY] j2={target[1]:.3f} < J2_HARD_MIN={J2_HARD_MIN:.2f}: clampato")
        target[1] = J2_HARD_MIN
        with _arm_lock:
            _arm_target[1] = J2_HARD_MIN
    for i in range(3):
        e = target[i] - pos[i]
        max_err = max(max_err, abs(e))
        vel[i] = _clamp(KP_ARM * e, -MAX_VEL, MAX_VEL)
    dynamixel.set_velocity({"velocity": vel})
    return max_err, pos


def _run_arm_controller(duration, grip=False):
    """Esegue il loop P a 50 Hz; grip=True reinvia la chiusura periodicamente."""
    t_end = time.time() + duration
    last_grip_t = 0.0
    while time.time() < t_end:
        _step_arm_controller()
        if grip:
            now = time.time()
            if now - last_grip_t >= 0.5:
                refresh_gripper_closed()
                last_grip_t = now
        time.sleep(0.02)   # 50 Hz


def move_arm_to(j1=0.0, j2=0.0, j3=0.0, duration=3.0, grip=False):
    """Porta il braccio agli angoli dati e li mantiene per duration secondi."""
    with _arm_lock:
        _arm_target[0] = j1
        _arm_target[1] = j2
        _arm_target[2] = j3
    _run_arm_controller(duration, grip=grip)

# =============================================================================
# COSTANTI DI TUNING PER IL TRACKING IK
# =============================================================================
# Definite a livello di modulo (non dentro _arm_track) per evitare shadowing
# silenzioso: se ridefinite localmente come variabili, modificarle qui non
# avrebbe effetto sul comportamento reale senza un errore esplicito.

TRACK_ALPHA         = 0.05   # smoothing j2/j3 → target nominale (τ≈0.4s a 50Hz)
TRACK_ALPHA_FINAL   = 0.02   # alpha più lento per la discesa finale sul cubo:
                             # braccio converge più lentamente → forze di reazione
                             # più piccole → drone meno disturbato prima della chiusura.
                             # (τ≈1s a 50Hz, vs τ≈0.4s con TRACK_ALPHA=0.05)
TRACK_ALPHA_J1      = 0.02   # j1 laterale (τ≈1s), senza muovere j2/j3 live
TRACK_CONVERGE_THRESH = 0.05 # rad — soglia convergenza
TRACK_STABLE_SECS   = 0.6   # s   — durata minima stabilità per early exit
TRACK_SETTLE_TIME   = 3.5   # s   — freeze j2/j3, smorzamento meccanico (era 1.5s)
TRACK_SETTLE_J1_ALPHA = 0.02
TRACK_IK_POSE_AVG_TIME = 0.8
TRACK_IK_POSE_CLAMP_XY = 0.05  # m — compensa offset stazionari senza inseguire oscillazioni
TRACK_IK_POSE_CLAMP_Z  = 0.06  # m

# =============================================================================
# TRACKING IK CHIUSO PER PICK E PLACE
# =============================================================================
# Problema: il drone non sta esattamente alla posizione comandata (residuo
# integratore, disturbi del braccio). Se calcoliamo l'IK una volta sola,
# l'EE non arriva sull'oggetto.
#
# Soluzione a due fasi:
#
# FASE 1 — TRACKING (loop a 50 Hz, durata ≈ 3.5 s):
#   Ogni 20 ms leggiamo la posizione del drone dalla cache pom, ricalcoliamo
#   l'IK per puntare l'EE sull'oggetto e aggiorniamo il target del controller.
#   Il braccio "insegue" continuamente la posizione reale del drone, correggendo
#   in tempo reale il drift e le oscillazioni.
#
# FASE 2 — HOLD (loop a 50 Hz, durata 2.5 s):
#   L'IK si congela sull'ultimo valore valido. Il controller continua a girare
#   ma il target non cambia più → il braccio converge meccanicamente alla
#   posizione stabile senza essere disturbato dal rumore di pom.
#   Solo dopo questa stabilizzazione si aziona il gripper.
#
# Questo evita che il gripper si chiuda/apra mentre il braccio sta ancora
# oscillando, che era la causa principale dei pick falliti.

def _arm_track(x_obj, y_obj, z_obj, x_drone_cmd, y_drone_cmd, duration, close_at_end):
    """
    IK a target FISSO (posizione comandata) + allineamento j1 lento da pom.
    close_at_end=True → chiude (presa); False → apre (deposito).

    DIAGNOSI (da log):
      Tutti i tentativi precedenti di tracking live (IK dalla posizione pom reale)
      creavano un loop di feedback positivo:
        drone oscilla in X → IK aggiorna j2/j3 → braccio si muove → forze di
        reazione → drone oscilla ancora di più.
      Con smoothing α=0.05 su j2/j3, il target oscillava tra j3=0.88 e j3=1.48
      seguendo l'oscillazione del drone da x=0.74 a x=0.97.

    SOLUZIONE — IK nominale (posizione comandata):
      j2/j3 calcolati UNA VOLTA per la posizione COMANDATA del drone (costante).
      Il braccio converge a una posizione fissa → le forze di reazione sono
      solo transitorie (durante il movimento iniziale), non oscillanti in steady-state.
      j1 è aggiornato lentamente (α=0.005) dalla posizione pom reale per
      l'allineamento laterale, senza creare forze significative.

    FASI:
      1. TRACKING (duration s): j2/j3 convergono al target nominale (fisso).
         j1 si aggiusta lentamente per Y. Uscita anticipata su convergenza.
      2. SETTLE (1.5s): IK congelato, smorzamento meccanico finale.
      3. GRIPPER: chiude solo se FK+pom confermano EE vicino al target (±7cm XY).
    """
    # Le costanti di tuning sono definite a livello di modulo (vedi sotto)
    # per evitare shadowing silenzioso dei valori globali.
    # MOTIVAZIONE SETTLE_TIME=3.5s:
    #   Il braccio meccanicamente si assesta in << 1s (τ_mech ≈ 2I/damping ≈ 0.2ms).
    #   Il DRONE invece, con gains PICK (Kpxy=12, Kvxy=18), ha:
    #     ωn=1.94 rad/s, ζ=1.45 → τ=0.36s → 5τ=1.8s di assestamento.
    #   Con gains VOLO (Kpxy=8) sarebbe 5τ=2.7s.
    #   3.5s > 5τ in entrambi i casi → gripper si chiude a drone assestato.

    # ── IK nominale congelata su posa filtrata ────────────────────────────────
    # j2/j3 NON inseguono POM live: si usa una media breve della posa reale solo
    # all'inizio della fase, clampata attorno al comando. Questo compensa errori
    # stazionari (soprattutto quota) senza ricreare il feedback positivo osservato
    # quando l'IK veniva aggiornata ad ogni ciclo.
    pose_ik = _average_pom_pose(duration=TRACK_IK_POSE_AVG_TIME)
    if pose_ik is None:
        x_ik, y_ik, z_ik, yaw_ik = x_drone_cmd, y_drone_cmd, PICKUP_ALT, 0.0
        print("  [TRACK] POM non disponibile: IK da comando nominale")
    else:
        x_m, y_m, z_m, yaw_m = pose_ik
        x_ik = x_drone_cmd + _clamp(x_m - x_drone_cmd, -TRACK_IK_POSE_CLAMP_XY, TRACK_IK_POSE_CLAMP_XY)
        y_ik = y_drone_cmd + _clamp(y_m - y_drone_cmd, -TRACK_IK_POSE_CLAMP_XY, TRACK_IK_POSE_CLAMP_XY)
        z_ik = PICKUP_ALT + _clamp(z_m - PICKUP_ALT, -TRACK_IK_POSE_CLAMP_Z, TRACK_IK_POSE_CLAMP_Z)
        yaw_ik = yaw_m
        print(f"  [TRACK] POM medio=({x_m:.3f},{y_m:.3f},{z_m:.3f}, yaw={yaw_m:.3f}) "
              f"→ IK pose=({x_ik:.3f},{y_ik:.3f},{z_ik:.3f})")

    dx_nom, dy_nom = _world_to_body(x_obj - x_ik, y_obj - y_ik, yaw_ik)
    dz_nom = z_obj - z_ik
    try:
        j1_nom, j2_nom, j3_nom = _drone_ik(dx_nom, dy_nom, dz_nom)
        print(f"  [TRACK] IK congelata: j1={j1_nom:.3f} j2={j2_nom:.3f} j3={j3_nom:.3f} "
              f"(D={math.hypot(math.hypot(dx_nom,dy_nom), dz_nom-_Z_J2):.3f}m)")
    except ValueError as e:
        print(f"  [TRACK] ERRORE: IK congelata fallita — {e}")
        return

    # ── Seed dagli angoli REALI del braccio (evita jerk iniziale) ─────────────
    # Se partissimo dal seed nominale (0.291, 0.993) quando l'arm è a j3=1.4,
    # il controller applicherebbe subito MAX_VEL per correggere 0.4 rad →
    # forza impulsiva sul drone. Partendo dalla posizione reale la correzione
    # è graduale.
    pos0 = _read_arm_pos()
    j1_t = float(pos0[0])
    j2_t = float(pos0[1])
    j3_t = float(pos0[2])
    print(f"  [TRACK] Seed da posizione reale: j=({j1_t:.3f},{j2_t:.3f},{j3_t:.3f})")

    stable_since = None
    dbg_i = 0
    t_end = time.time() + duration

    while time.time() < t_end:
        pose = get_pom_pose()
        if pose is not None:
            x_d, y_d, z_d, yaw_d = pose
        else:
            x_d, y_d, z_d, yaw_d = x_drone_cmd, y_drone_cmd, PICKUP_ALT, 0.0

        # j1: converge al valore NOMINALE fisso (calcolato dall'IK iniziale).
        # NON insegue la posizione pom in loop: j1_live = atan2(y_pom, x_pom) crea
        # un loop di feedback positivo in Y:
        #   drone oscilla in Y → j1 si aggiorna → momento su drone → oscilla di più.
        # Con j1 fisso a j1_nom il braccio non reagisce alle oscillazioni del drone;
        # l'errore Y residuo viene corretto solo da nhfc (termine proporzionale + integrale).
        j1_t += TRACK_ALPHA_J1 * (j1_nom - j1_t)

        # j2/j3: convergono al target NOMINALE (fisso, calcolato sopra).
        # Nella fase di discesa (close_at_end=True) si usa TRACK_ALPHA_FINAL
        # (τ≈1s) invece di TRACK_ALPHA (τ≈0.4s): il braccio converge più
        # lentamente riducendo le forze di reazione sul drone nell'istante
        # critico prima della chiusura del gripper.
        alpha_now = TRACK_ALPHA_FINAL if close_at_end else TRACK_ALPHA
        j2_t += alpha_now * (j2_nom - j2_t)
        j3_t += alpha_now * (j3_nom - j3_t)

        with _arm_lock:
            _arm_target[0] = j1_t
            _arm_target[1] = j2_t
            _arm_target[2] = j3_t

        max_err, pos = _step_arm_controller()

        dbg_i += 1
        if dbg_i % 10 == 0:
            dx_b_dbg, dy_b_dbg = _world_to_body(x_obj - x_d, y_obj - y_d, yaw_d)
            D_cur = math.hypot(math.hypot(dx_b_dbg, dy_b_dbg), z_obj - z_d)
            stab = f"STAB {time.time()-stable_since:.1f}s" if stable_since else "----"
            print(f"  [TRACK t={t_end-time.time():4.1f}s] "
                  f"pom=({x_d:.2f},{y_d:.2f},{z_d:.2f}) D={D_cur:.3f} "
                  f"j_pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
                  f"j_tgt=({j1_t:.2f},{j2_t:.2f},{j3_t:.2f}) "
                  f"err={max_err:.3f} {stab}")

        if max_err < TRACK_CONVERGE_THRESH:
            if stable_since is None:
                stable_since = time.time()
            elif time.time() - stable_since >= TRACK_STABLE_SECS:
                print(f"  [CONV] Convergenza! err={max_err:.3f}")
                break
        else:
            stable_since = None

        time.sleep(0.02)   # 50 Hz

    # ── Settle: tutti i giunti congelati al nominale, smorzamento meccanico ─────
    # j1 continua a convergere lentamente a j1_nom (SETTLE_J1_ALPHA << ALPHA):
    # se il braccio aveva una piccola deviazione residua, questo la annulla
    # senza mai inseguire oscillazioni pom (j1_nom è costante).
    print(f"  [SETTLE] j=({j1_t:.2f},{j2_t:.2f},{j3_t:.2f}), pausa {TRACK_SETTLE_TIME:.1f}s...")
    settle_end = time.time() + TRACK_SETTLE_TIME
    while time.time() < settle_end:
        j1_t += TRACK_SETTLE_J1_ALPHA * (j1_nom - j1_t)
        with _arm_lock:
            _arm_target[0] = j1_t
            _arm_target[1] = j2_t
            _arm_target[2] = j3_t
        _step_arm_controller()
        time.sleep(0.02)

    # ── Verifica e gripper ────────────────────────────────────────────────────
    if close_at_end:
        print(f"  [GRIP] Attendo EE stabile entro {GRIP_DXY_TOL*100:.1f}cm XY "
              f"e {GRIP_Z_TOL*100:.1f}cm Z...")
        ready_since = None
        last_desc = "EE non valutato"
        deadline = time.time() + GRIP_MAX_WAIT
        while time.time() < deadline:
            j1_t += TRACK_SETTLE_J1_ALPHA * (j1_nom - j1_t)
            with _arm_lock:
                _arm_target[0] = j1_t
                _arm_target[1] = j2_t
                _arm_target[2] = j3_t
            _step_arm_controller()

            cube_ok, grip_desc = _cube_between_fingers(x_obj, y_obj, z_obj)
            last_desc = grip_desc
            now = time.time()
            if cube_ok:
                if ready_since is None:
                    ready_since = now
                elif now - ready_since >= GRIP_STABLE_SECS:
                    break
            else:
                ready_since = None
            time.sleep(0.02)

        stable_ok = ready_since is not None and time.time() - ready_since >= GRIP_STABLE_SECS
        print(f"  [GRIP] {last_desc}")
        if stable_ok:
            print("  [GRIP] Cubo stabile tra le dita → chiudo")
            close_gripper()
        else:
            print("  [GRIP] EE non stabile sul target — gripper NON chiuso")
    else:
        open_gripper()
    time.sleep(1.5)


def _arm_pick(x_obj, y_obj, z_obj, x_drone_cmd, y_drone_cmd):
    # Approccio in due fasi per evitare che il braccio urti il cubo durante lo swing di j1.
    #
    # Problema: il drone oscilla in Y mentre muoviamo il braccio. L'IK compensa
    # con grandi variazioni di j1 (fino a 35°). In quel swing laterale il dito
    # passa esattamente vicino al cubo e lo urta.
    #
    # Soluzione: fase 1 porta l'EE 6cm SOPRA il cubo → anche se j1 swinga,
    # il dito passa SOPRA il cubo (cubo top=0.250m, EE target=0.325m → 7.5cm gap).
    # Fase 2 parte con j1 già allineato → solo j2/j3 cambiano per scendere;
    # nessun swing laterale, nessuna collisione.
    APPROACH_OFFSET = 0.06   # m sopra il target finale

    open_gripper()
    time.sleep(0.5)   # attende che il gripper sia fisicamente aperto

    print(f"  Fase 1: allineamento sopra il cubo (z={z_obj+APPROACH_OFFSET:.3f} m)...")
    _arm_track(x_obj, y_obj, z_obj + APPROACH_OFFSET,
               x_drone_cmd, y_drone_cmd, duration=9.0, close_at_end=False)

    # Pausa tra le fasi: lascia al drone il tempo di stabilizzarsi
    # dopo i disturbi della fase 1 prima di scendere sul cubo.
    # Con gains PICK τ_drone≈0.32s → 5τ=1.8s + buffer per oscillazioni residue.
    # La pausa più lunga garantisce anche che il braccio smorzamento sull'oggetto
    # sopra-cubo non venga trasmesso come disturbo alla fase di discesa.
    print("  Pausa stabilizzazione inter-fasi...")
    time.sleep(5.0)   # aumentato da 3.0s: attende pieno assestamento del drone

    print(f"  Fase 2: discesa sul cubo (z={z_obj:.3f} m)...")
    _arm_track(x_obj, y_obj, z_obj,
               x_drone_cmd, y_drone_cmd, duration=8.0, close_at_end=True)


def _arm_place(x_target, y_target, z_target, x_drone_cmd, y_drone_cmd):
    # Il gripper è già chiuso (tiene il cubo) — non lo apriamo qui
    print("  Tracking IK avviato (deposito)...")
    _arm_track(x_target, y_target, z_target, x_drone_cmd, y_drone_cmd, duration=5.0, close_at_end=False)

# =============================================================================
# SETUP — configurazione componenti GenoM
# =============================================================================
def setup():
    # optitrack: mocap locale via UDP multicast (porta 1509 come nel world file)
    optitrack.connect({
        "host": "localhost",
        "host_port": "1509",
        "mcast": "",
        "mcast_port": "0",
    })

    # rotorcraft: collegamento al simulatore tramite pseudo-terminale
    rotorcraft.connect({"serial": ROBOT_SERIAL, "baud": 0})
    rotorcraft.set_sensor_rate({"rate": {"imu": 1000, "mag": 0, "motor": 20, "battery": 1}})
    rotorcraft.set_imu_filter({"gfc": [20, 20, 20], "afc": [5, 5, 5], "mfc": [20, 20, 20]})

    # pom: filtro di Kalman per la stima dello stato. Legge IMU + mocap optitrack.
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

    # maneuver: generatore di traiettorie. Riceve waypoint e pubblica desired state.
    maneuver.connect_port({"local": "state", "remote": "pom/frame/robot"})
    maneuver.set_bounds({
        "xmin": -5, "xmax": 5, "ymin": -5, "ymax": 5,
        "zmin": 0,  "zmax": 5, "yawmin": -3.14, "yawmax": 3.14,
    })

    # phynt: osservatore del wrench esterno (forze di reazione del braccio).
    # Struttura a due modalità:
    #   wo=True, af=False  → solo stima (safe per il decollo e il volo)
    #   wo=True, af=True   → stima + compensazione attiva (pick/place)
    #
    # Con af=True il filtro ammettenza aggiunge a phynt/desired un offset
    # δx tale che:  M_v·δẍ + B_v·δẋ + K_v·δx = -f_esterno
    # Con K_v alto (molla rigida) δx ≈ -f/K → piccolo spostamento opposto
    # alla forza del braccio → nhfc compensa il disturbo in feed-forward.
    phynt.set_mass({"mass": MASS})

    # Wrench observer: cutoff frequency (Hz) per i 6 assi [fx,fy,fz, tx,ty,tz].
    # 5 Hz forza (τ≈32ms), 2 Hz momento (τ≈80ms): buon compromesso rumore/velocità.
    phynt.set_wo_fc({"fc": [5.0, 5.0, 5.0, 2.0, 2.0, 2.0]})

    # Wrench observer threshold: filtra stime di forza sotto soglia (rumore sensore).
    # DIAGNOSI LOG: con thresh=1.0N il WO vedeva efx=efy=0 per tutto il tempo —
    # le forze di reazione del braccio (<1N in XY) venivano filtrate completamente
    # e l'AF non compensava mai. Ridotto a 0.3N per rilevare anche forze piccole.
    # [fx,fy,fz,tx,ty,tz] in N e N·m.
    phynt.set_wo_thresh({"thresh": [0.3, 0.3, 0.5, 0.05, 0.05, 0.1]})

    # Admittance filter: diversa rigidità XY vs Z.
    #
    # PRINCIPIO: in steady-state l'AF risolve  M·δẍ + B·δẋ + K·δx = -F_braccio
    #   → δx_ss = -F_braccio / K
    #
    # PROBLEMA IDENTIFICATO DAL LOG (run 2026-05-26):
    #   Il WO rileva il peso del braccio esteso (~0.19 kg × 9.81 ≈ 1.86N verso il basso)
    #   come forza esterna. L'AF compensa alzando il drone:
    #     δz_ss = 1.86 / K_z = 1.86 / 60 = 3.1 cm verso l'alto
    #   Il drone si stabilizza a z = PICKUP_ALT + 3.1 cm = 0.681 m invece di 0.65 m.
    #   L'IK nominale calcolato per PICKUP_ALT=0.65 m punta l'EE a 0.265 m,
    #   ma con il drone a 0.681 m l'EE finisce a 0.296 m (3.1 cm troppo alto).
    #   Sommando altri effetti (tilt nhfc, integrale) l'errore raggiungeva 6.4 cm
    #   → check d_z=0.064 > soglia 0.060 → gripper NON chiuso.
    #
    # SOLUZIONE — asse Z molto rigido, XY cedevole:
    #   K_z = 500 N/m → δz = 1.86/500 = 3.7 mm (vs 3.1 cm prima!)
    #   Il drone rimane a z ≈ 0.654 m → EE a 0.654 - 0.385 = 0.269 m ✓ (d_z=4 mm)
    #   K_xy = 50 N/m: rimane cedevole per smorzare disturbi laterali del braccio.
    #
    # STABILITÀ asse Z: K_z=500, M=1.5 → B_z_crit = 2·√(500·1.5) = 2·√750 = 54.8
    #   B_z=70 → ζ_z = 70/(2·√750) = 1.28 → sovrasmorzato ✓
    #   τ_z = M/B_z = 1.5/70 = 0.021 s → AF_z si assesta in < 100 ms
    #
    # STABILITÀ asse XY: K_xy=50, M=1.5 → B_xy_crit = 17.3
    #   B_xy=22 → ζ_xy = 1.27 → sovrasmorzato ✓
    #
    # Indici [0..2] = forza (traslazione XYZ), [3..5] = momento (rotazione).
    phynt.set_af_parameters({
        "mass": 1.5,   # ridotto da 2.0: risposta più rapida alle forze del braccio
        "B":    [22.0, 22.0, 70.0, 1.0, 1.0, 0.6],  # B_z alto per stabilità con K_z=500
        "K":    [50.0, 50.0, 500.0, 2.0, 2.0, 1.0], # K_z molto alto: AF non alza il drone
        "J":    [0.1, 0.0, 0.0,
                 0.0, 0.1, 0.0,
                 0.0, 0.0, 0.05],
    })

    # Avvio con af=False: con wrench non ancora calibrato l'AF corromperebbe
    # la traiettoria. enable(True, True) viene chiamato in _enable_af().
    phynt.enable({"enable": {"wo": True, "af": False}})
    phynt.connect_port({"local": "reference",      "remote": "maneuver/desired"})
    phynt.connect_port({"local": "state",          "remote": "pom/frame/robot"})
    phynt.connect_port({"local": "wrench_measure", "remote": "nhfc/wrench_measure"})

    # nhfc: controllore nonlineare H-infinity per posizione+assetto.
    # rx=-20° perché i rotori nel SDF hanno un'inclinazione di 0.3491 rad ≈ 20°.
    # Con rx=+20° la matrice di allocazione yaw è invertita → feedback positivo
    # → drone ruota in CCW immediatamente al primo ciclo di controllo.
    # ix=1 abilita l'integratore (era ix=0 per default): senza integratore il
    # drone aveva un errore stazionario di ~38 cm in X e ~18 cm in Z.
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
    # Gains: due set distinti — VOLO (default) e PICK (durante arm tracking).
    # Il set PICK viene attivato da _set_pick_gains() prima di ogni arm_pick/place.
    # Viene ripristinato da _restore_flight_gains() prima del volo di trasferimento.
    #
    # ANALISI TEMPO ASSESTAMENTO (set VOLO):
    #   ωn_xy = √(Kpxy/mass) = √(8/2.91) ≈ 1.66 rad/s
    #   ζ_xy  = Kvxy/(2·mass·ωn) = 12/(2·2.91·1.66) ≈ 1.24  → sovrasmorzato
    #   τ_xy  = 1/(ζ·ωn) ≈ 0.53s → 5τ ≈ 2.7s  (troppo lento durante il pick)
    #
    # ANALISI TEMPO ASSESTAMENTO (set PICK):
    #   ωn_xy = √(12/2.91) ≈ 2.03 rad/s
    #   ζ_xy  = 18/(2·2.91·2.03) ≈ 1.52  → sovrasmorzato
    #   τ_xy  = 1/(1.45·1.94) ≈ 0.36s → 5τ ≈ 1.8s  (2× più veloce)
    #   → il drone si assesta prima che SETTLE_TIME (3.5s) scada → gripper OK
    nhfc.set_servo_gain({"gain": {
        "Kpxy": 8.0, "Kpz": 7.0,
        "Kvxy": 12.0, "Kvz": 9.0,
        "Kixy": 1.0,  "Kiz": 1.0,
        "Kqxy": 7.0, "Kqz": 0.8,
        "Kwxy": 2.0, "Kwz": 0.5,
    }})
    # tilt_prioritized: dalla documentazione TK3 è la modalità raccomandata per
    # aerial manipulators perché "reduces payload oscillation" e "avoids wasting
    # authority on yaw". Allinea prima la direzione di spinta (posizione), poi
    # corregge lo yaw senza influenzare la traslazione.
    # NOTA: in passato usavamo full_attitude perché lo yaw divergeva con
    # tilt_prioritized, ma quello era causato da rx=+20 (sbagliato). Con rx=-20
    # la matrice di allocazione è corretta e tilt_prioritized funziona.
    nhfc.set_control_mode({"att_mode": "::nhfc::tilt_prioritized"})
    # dx=0.10: soglia incertezza posizione per emergency descent.
    # Era 0.05m — troppo stretta: durante il tracking del braccio la covarianza
    # del filtro di Kalman pom può piccare brevemente sopra 5cm e far scattare
    # l'emergency descent. 10cm è più tollerante ma ancora sicuro.
    nhfc.set_emerg({"emerg": {"descent": 0.1, "dx": 0.10, "dq": 20, "dv": 0.3, "dw": 20}})
    nhfc.connect_port({"local": "state",         "remote": "pom/frame/robot"})
    nhfc.connect_port({"local": "reference",     "remote": "phynt/desired"})
    nhfc.connect_port({"local": "rotor_measure", "remote": "rotorcraft/rotor_measure"})
    rotorcraft.connect_port({"local": "rotor_input", "remote": "nhfc/rotor_input"})

    DYN_SERIAL = os.environ.get("DYNAMIXEL_SERIAL", "/tmp/pty-fahexa-arm")
    dynamixel.connect({"serial": DYN_SERIAL, "baud": 1000000})

# =============================================================================
# START / STOP
# =============================================================================
def start():
    # Avvia logging di tutti i componenti
    pom.log_state(os.path.join(LOG_DIR, "pom.log"))
    pom.log_measurements(os.path.join(LOG_DIR, "pom_measurements.log"))
    optitrack.set_logfile(os.path.join(LOG_DIR, "opti.log"))
    rotorcraft.log(os.path.join(LOG_DIR, "rotorcraft.log"))
    nhfc.log(os.path.join(LOG_DIR, "nhfc.log"))
    # phynt.log() viene chiamato DOPO phynt.servo() (vedi sotto):
    # il componente richiede un reference valido per iniziare a scrivere dati.
    # Se chiamato prima, scrive solo l'header e poi si blocca (bug noto).
    maneuver.log(os.path.join(LOG_DIR, "maneuver.log"))
    dynamixel.log(os.path.join(LOG_DIR, "dynamixel.log"))

    rotorcraft.start()
    time.sleep(0.2)

    # Avvia il thread di lettura pom dopo che rotorcraft ha iniziato a produrre
    # dati (altrimenti il log è vuoto e _read_drone_pose restituisce None)
    start_pom_reader()

    # Inizializza ogni componente alla posizione corrente prima di avviare i
    # loop servo. Senza questo, servo() lancia e_input perché i port di
    # riferimento sono vuoti.
    nhfc.set_current_position()
    time.sleep(0.1)
    maneuver.set_current_state()
    time.sleep(0.1)
    phynt.set_current_position()
    time.sleep(0.1)

    # Ordine critico: phynt pubblica "desired" → nhfc legge da phynt/desired
    # → rotorcraft legge da nhfc/rotor_input.
    phynt.servo(ack=True)
    nhfc.servo(ack=True)
    rotorcraft.servo(ack=True)

    # phynt.log DOPO servo: ora il reference è valido e il log scrive dati reali.
    # Il log registra: desired (posizione modificata AF), external_wrench (stima).
    try:
        phynt.log({"path": os.path.join(LOG_DIR, "phynt.log"), "decimation": 1})
    except Exception as e:
        print(f"[WARN] phynt.log fallito: {e}")

    try:
        dynamixel.servo(ack=True)
    except Exception:
        pass

def stop():
    """
    Arresta tutti i componenti. Ogni componente ha il proprio try/except
    per garantire che il fallimento di uno non impedisca lo stop degli altri.
    """
    for fn in [dynamixel.stop, dynamixel.log_stop]:
        try: fn()
        except Exception as e: print(f"[WARN] {fn.__qualname__}: {e}")

    for fn in [rotorcraft.stop, rotorcraft.log_stop]:
        try: fn()
        except Exception as e: print(f"[WARN] {fn.__qualname__}: {e}")

    for fn in [nhfc.stop, nhfc.log_stop]:
        try: fn()
        except Exception as e: print(f"[WARN] {fn.__qualname__}: {e}")

    for fn in [phynt.stop, phynt.log_stop]:
        try: fn()
        except Exception as e: print(f"[WARN] {fn.__qualname__}: {e}")

    for fn in [maneuver.stop, maneuver.log_stop]:
        try: fn()
        except Exception as e: print(f"[WARN] {fn.__qualname__}: {e}")

    stop_pom_reader()

    for fn in [pom.log_stop, optitrack.unset_logfile]:
        try: fn()
        except Exception as e: print(f"[WARN] {fn.__qualname__}: {e}")

# =============================================================================
# GESTIONE ADMITTANCE FILTER
# =============================================================================
# _enable_af(): da chiamare quando il drone è fermo e stabile, PRIMA che il
#   braccio inizi a muoversi. Azzera il bias del WO (set_wo_zero) in modo che
#   l'AF reagisca solo ai disturbi del braccio, non agli errori di modello.
# _disable_af(): da chiamare prima del volo di trasferimento. Con af=True
#   durante il volo in avanzamento, phynt potrebbe interpretare le forze
#   aerodinamiche come disturbi e corrompere la traiettoria.

# =============================================================================
# GAINS NHFC — MODALITÀ VOLO vs PICK
# =============================================================================
# Durante il pick il drone deve essere più rigido: il braccio crea forze di
# reazione continue, e con gains bassi il drone oscilla per 2-3s prima di
# assestare. Con gains più alti l'assestamento scende a ~1.8s, compatibile
# con SETTLE_TIME=3.5s.
#
# KIXY RIDOTTO IN PICK: l'integratore accumula errore nei ~8s di arm-tracking
# (braccio crea offset costante). Al termine, con Kixy alto, scarica bruscamente
# questo errore → overshoot → oscillazione proprio quando vogliamo chiudere
# il gripper. Kixy=0.3 limita l'accumulo a ~10% dell'effetto con Kixy=1.0.
# Il termine proporzionale (Kpxy=12) garantisce comunque accuratezza.

_FLIGHT_GAINS = {
    "Kpxy": 8.0,  "Kpz": 7.0,
    "Kvxy": 12.0, "Kvz": 9.0,
    "Kixy": 1.0,  "Kiz": 1.0,
    "Kqxy": 7.0,  "Kqz": 0.8,
    "Kwxy": 2.0,  "Kwz": 0.5,
}

_PICK_GAINS = {
    # Posizione: più rigido (ωn da 1.66 → 2.03 rad/s, τ da 0.49 → 0.32s)
    "Kpxy": 12.0, "Kpz": 10.0,
    "Kvxy": 18.0, "Kvz": 12.0,
    # Kixy ridotto: previene windup XY durante arm-tracking (braccio crea offset costante
    # su XY → l'integrale accumula → overshoot al termine). Kpxy=12 garantisce comunque
    # accuratezza XY in steady-state.
    #
    # Kiz IDENTICO a FLIGHT (1.0): CRITICO per evitare drift Z al cambio gains.
    #   Meccanismo: nhfc conserva il valore accumulato dell'integratore Z (I_z) anche
    #   quando si cambiano i gains. In volo stabile con Kiz_flight=1.0, I_z si calibra
    #   a un valore tale che  1.0 * I_z = forza_necessaria_per_hovering.
    #   Se Kiz scende a 0.5 → forza_corretta = 0.5 * I_z = forza_necessaria/2
    #   → deficit di spinta verticale → drone sale fino a raggiungere un nuovo equilibrio
    #   ~4 cm sopra il target (esattamente l'errore sistematico osservato nei run 1 e 2).
    #   Con Kiz=1.0 la transizione FLIGHT→PICK non altera la forza verticale → drone
    #   rimane a PICKUP_ALT con errore < 5 mm dopo la transizione.
    "Kixy": 0.3,  "Kiz": 1.0,
    # Assetto: più reattivo alle perturbazioni di coppia del braccio
    "Kqxy": 10.0, "Kqz": 1.0,
    "Kwxy": 3.0,  "Kwz": 0.8,
}

def _set_pick_gains():
    """Gains nhfc ad alta rigidità per pick/place. Drone oscilla meno."""
    print("  [GAINS] Modalità PICK: Kpxy=12, Kvxy=18, Kixy=0.3, Kqxy=10")
    # NOTA: dict(_PICK_GAINS) — copia esplicita obbligatoria.
    # genomix/pmt.py consuma il dizionario di input durante la serializzazione
    # (rimuove elementi via pop() mentre costruisce il JSON). Se si passa il
    # riferimento diretto, alla seconda chiamata il dict è vuoto → TypeError.
    nhfc.set_servo_gain({"gain": dict(_PICK_GAINS)})
    time.sleep(0.3)   # un ciclo per l'applicazione dei nuovi gains

def _restore_flight_gains():
    """Ripristina gains nominali per il volo di trasferimento."""
    print("  [GAINS] Modalità VOLO: Kpxy=8, Kvxy=12, Kixy=1.0, Kqxy=7")
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
    # τ_AF = mass/B = 1.5/22 ≈ 0.068s → 3τ ≈ 0.2s (convergenza AF)
    # Il drone può ancora aver piccole oscillazioni residue dal set_wo_zero.
    # 1.0s e sufficiente qui: il braccio resta teso e il primo movimento e una discesa lenta.
    # DIAGNOSI LOG: phynt.set_wo_zero() è un'activity che interrompe il task principale
    # di phynt, chiudendo il file di log. Dopo enable() il task riprende ma il log
    # non riparte automaticamente. Soluzione: riavvio esplicito del logging.
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


# =============================================================================
# MANEUVER: decollo robusto
# =============================================================================
_MANEUVER_PROFILES = {
    "takeoff": {"v": 0.45, "w": 0.55, "a": 0.55, "dw": 0.70, "j": 1.10, "ddw": 1.20, "s": 3.0, "dddw": 3.0},
    "cruise":  {"v": 0.55, "w": 0.55, "a": 0.55, "dw": 0.70, "j": 1.00, "ddw": 1.20, "s": 2.5, "dddw": 3.0},
    "approach":{"v": 0.38, "w": 0.45, "a": 0.45, "dw": 0.60, "j": 0.90, "ddw": 1.00, "s": 2.5, "dddw": 2.8},
    "lift":    {"v": 0.34, "w": 0.40, "a": 0.40, "dw": 0.55, "j": 0.80, "ddw": 0.90, "s": 2.2, "dddw": 2.5},
    "landing": {"v": 0.24, "w": 0.30, "a": 0.28, "dw": 0.35, "j": 0.45, "ddw": 0.60, "s": 1.2, "dddw": 1.5},
    "recovery":{"v": 0.70, "w": 0.80, "a": 0.90, "dw": 1.00, "j": 1.80, "ddw": 1.80, "s": 5.0, "dddw": 5.0},
}
_active_maneuver_profile = None


def _set_maneuver_profile(name):
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


def _takeoff_request(height, duration):
    attempts = [
        ("takeoff", duration),
        ("takeoff", max(duration * 1.55, duration + 1.8)),
        ("recovery", max(duration * 2.0, duration + 3.2)),
    ]
    last_error = None
    for profile, dur in attempts:
        _set_maneuver_profile(profile)
        _sync_maneuver_to_current_state()
        try:
            maneuver.take_off({"height": height, "duration": dur})
            if profile != "takeoff" or abs(dur - duration) > 1e-6:
                print(f"  [MANEUVER] takeoff retry OK: profilo={profile}, duration={dur:.1f}s")
            return dur
        except Exception as e:
            last_error = e
            if "e_limits" in str(e):
                print(f"  [MANEUVER] takeoff rifiutato da {profile} duration={dur:.1f}s: e_limits")
                continue
            raise
    raise last_error


def _goto_request(x, y, z, yaw, duration, profile):
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


# =============================================================================
# SEQUENZA DI SIMULAZIONE
# =============================================================================
def _goto(x, y, z, yaw=0.0, duration=5.0, wait=None, profile="cruise"):
    """Invia waypoint con retry se maneuver rifiuta i limiti cinematici."""
    used_duration = _goto_request(x, y, z, yaw, duration, profile)
    hold_time = used_duration + 1.0 if wait is None else max(wait, used_duration + 0.8)
    time.sleep(hold_time)


def _hold_arm(target=ARM_STRETCHED, duration=1.0, grip=False):
    """Mantiene il braccio; target=None conserva il target corrente."""
    if target is not None:
        _arm_target[0], _arm_target[1], _arm_target[2] = target
    t_end = time.time() + duration
    last_grip_t = 0.0
    while time.time() < t_end:
        _step_arm_controller()
        if grip:
            now = time.time()
            if now - last_grip_t >= 0.5:
                refresh_gripper_closed()
                last_grip_t = now
        time.sleep(0.02)


def _goto_with_arm(x, y, z, yaw=0.0, duration=5.0, wait=None,
                   arm=ARM_STRETCHED, grip=False, profile="cruise"):
    """Esegue un goto robusto mantenendo il braccio nella posa richiesta."""
    used_duration = _goto_request(x, y, z, yaw, duration, profile)
    hold_time = used_duration + 1.0 if wait is None else max(wait, used_duration + 0.8)
    _hold_arm(arm, hold_time, grip=grip)


def _end_effector_world():
    pose = get_pom_pose()
    if pose is None:
        return None
    x_d, y_d, z_d, yaw_d = pose
    pos_j = _read_arm_pos()
    ax, ay, az = _fk_arm(pos_j[0], pos_j[1], pos_j[2])
    c, s = math.cos(yaw_d), math.sin(yaw_d)
    return x_d + ax * c - ay * s, y_d + ax * s + ay * c, z_d + az


def _stretched_gripper_drone_pose(x_obj, y_obj, z_obj, yaw=0.0, z_offset=0.0):
    """Ritorna posa drone che mette la pinza tesa sopra/alla quota del target."""
    ax, ay, az = _fk_arm(*ARM_STRETCHED)
    c, s = math.cos(yaw), math.sin(yaw)
    ee_dx = ax * c - ay * s
    ee_dy = ax * s + ay * c
    x_d = x_obj - ee_dx
    y_d = y_obj - ee_dy
    z_d = max(MIN_GRASP_DRONE_ALT, z_obj + z_offset - az)
    return x_d, y_d, z_d, yaw


def _read_cube_pose_or_nominal():
    cube = _get_cube_pose()
    if cube is not None:
        return cube, True
    return (CUBE_X, CUBE_Y, CUBE_CENTER_Z_NOMINAL), False


def _cube_between_current_gripper(cube_pose=None, require_lift=False, min_lift_z=0.35):
    if cube_pose is None:
        cube_pose = _get_cube_pose()
    ee = _end_effector_world()
    if cube_pose is None or ee is None:
        return False, "posa cubo o EE non disponibile"
    d_xy = math.hypot(cube_pose[0] - ee[0], cube_pose[1] - ee[1])
    d_z = abs(cube_pose[2] - ee[2])
    lifted = (not require_lift) or cube_pose[2] > min_lift_z
    ok = d_xy < VERIFY_DXY_TOL and d_z < VERIFY_Z_TOL and lifted
    desc = (f"cube=({cube_pose[0]:.3f},{cube_pose[1]:.3f},{cube_pose[2]:.3f}) "
            f"EE=({ee[0]:.3f},{ee[1]:.3f},{ee[2]:.3f}) "
            f"d_xy={d_xy:.3f}/{VERIFY_DXY_TOL:.3f} "
            f"d_z={d_z:.3f}/{VERIFY_Z_TOL:.3f} "
            f"lift={'OK' if lifted else 'NO'}")
    return ok, desc


def _ik_joints_from_current_pose(x_obj, y_obj, z_obj, label, avg_time=0.6):
    pose = _average_pom_pose(duration=avg_time)
    if pose is None:
        print(f"  [IK {label}] POM non disponibile")
        return None

    x_d, y_d, z_d, yaw_d = pose
    dx_b, dy_b = _world_to_body(x_obj - x_d, y_obj - y_d, yaw_d)
    dz_b = z_obj - z_d
    try:
        joints = _drone_ik(dx_b, dy_b, dz_b)
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


def _grasp_until_confirmed(x_drone_cmd, y_drone_cmd):
    """Presa con braccio IK: approccio sopra-cubo, discesa, chiusura, lift e verifica.
    Ritorna la posa del cubo dopo la presa confermata, oppure None se tutti i
    MAX_GRASP_ATTEMPTS tentativi falliscono (evita loop infinito).
    """
    attempt = 1
    while True:
        if attempt > MAX_GRASP_ATTEMPTS:
            print(f"  [IK] Raggiunto MAX_GRASP_ATTEMPTS={MAX_GRASP_ATTEMPTS}: presa abortita")
            return None
        cube, measured = _read_cube_pose_or_nominal()
        source = "Gazebo" if measured else "nominale"
        if measured and cube[2] < CUBE_CENTER_Z_NOMINAL - 0.08:
            print(f"  [IK] CUBO CADUTO z={cube[2]:.3f} m: presa abortita")
            return None

        ox, oy, oz = GRASP_SCAN_OFFSETS[(attempt - 1) % len(GRASP_SCAN_OFFSETS)]
        target_x = cube[0] + ox
        target_y = cube[1] + oy
        target_z = cube[2] + oz
        print(f"--- TENTATIVO PRESA IK #{attempt} ({source}) ---")
        print(f"  [CUBE] ({cube[0]:.3f},{cube[1]:.3f},{cube[2]:.3f}) "
              f"offset=({ox:.3f},{oy:.3f},{oz:.3f})")

        open_gripper()
        _hold_arm(None, duration=0.5)

        # Attendi che il drone si stabilizzi prima di calcolare l'IK:
        # un errore di posizione di 5 cm durante l'oscillazione causa
        # uno scostamento di ~3 cm sull'EE → presa fuori bersaglio.
        print(f"  [SETTLE] attesa stabilizzazione drone ({PRE_GRASP_SETTLE_TIME:.1f}s)...")
        _hold_arm(None, duration=PRE_GRASP_SETTLE_TIME)

        app_j = _ik_joints_from_current_pose(
            target_x, target_y, target_z + IK_APPROACH_Z, "APP")
        if app_j is None:
            attempt += 1
            time.sleep(0.5)
            continue

        print(f"  [F1] braccio sopra il cubo z={target_z + IK_APPROACH_Z:.3f} m")
        move_arm_to(*app_j, duration=5.0)
        _hold_arm(None, duration=1.5)

        grasp_j = _ik_joints_from_current_pose(
            target_x, target_y, target_z + IK_GRASP_Z_OFFSET, "GRASP")
        if grasp_j is None:
            print("  [F2] IK presa fallita: ritiro sicuro e ritento")
            move_arm_to(*app_j, duration=2.0)
            # ARM_TUCKED come punto intermedio sicuro prima di ARM_STRETCHED:
            # garantisce che j2 non attraversi mai valori negativi nella
            # transizione verso la posizione di attesa.
            move_arm_to(*ARM_TUCKED, duration=2.0)
            _hold_arm(ARM_STRETCHED, duration=1.5)
            attempt += 1
            time.sleep(0.5)
            continue

        print(f"  [F2] discesa al cubo z={target_z + IK_GRASP_Z_OFFSET:.3f} m")
        move_arm_to(*grasp_j, duration=4.0)
        _hold_arm(None, duration=1.0)

        cube_now = _get_cube_pose() or cube
        pre_ok, pre_desc = _cube_between_current_gripper(cube_now, require_lift=False)
        print(f"  [PRE-GRIP] {pre_desc}")
        if not pre_ok:
            print("  [PRE-GRIP] non allineato: ritiro sicuro e ritento")
            move_arm_to(*app_j, duration=2.5)
            move_arm_to(*ARM_TUCKED, duration=2.0)
            _hold_arm(ARM_STRETCHED, duration=1.5)
            attempt += 1
            time.sleep(0.5)
            continue

        print("  [GRIP] chiudo e mantengo forza")
        close_gripper()
        _hold_arm(None, duration=1.2, grip=True)

        lift_j = _ik_joints_from_current_pose(
            cube_now[0], cube_now[1], cube_now[2] + IK_APPROACH_Z, "LIFT")
        if lift_j is None:
            lift_j = app_j
        print("  [LIFT] sollevo prima col braccio, poi col drone")
        move_arm_to(*lift_j, duration=3.0, grip=True)
        _goto_with_arm(x_drone_cmd, y_drone_cmd, POST_GRASP_CHECK_ALT,
                       yaw=0.0, duration=2.0, wait=3.5, arm=None, grip=True,
                       profile="lift")

        cube_after = _get_cube_pose()
        min_lift_z = max(CUBE_CENTER_Z_NOMINAL + 0.12, cube_now[2] + 0.10)
        ok, desc = _cube_between_current_gripper(
            cube_after, require_lift=True, min_lift_z=min_lift_z)
        print(f"  [GRASP CHECK] {desc}")
        if ok:
            print("  [GRASP CHECK] PRESA CONFERMATA (IK)")
            return cube_after

        print("  [GRASP CHECK] presa non confermata: riapro e ritento")
        open_gripper()
        move_arm_to(*ARM_TUCKED, duration=1.5)
        _hold_arm(ARM_STRETCHED, duration=1.0)
        _goto_with_arm(x_drone_cmd, y_drone_cmd, PICKUP_ALT,
                       yaw=0.0, duration=2.5, wait=4.0, profile="approach")
        attempt += 1
        time.sleep(0.5)


def _place_with_arm_motion():
    print("--- DEPOSITO CON BRACCIO MOBILE ---")
    print("  [STOW] stendo il braccio per il trasferimento")
    _hold_arm(ARM_STRETCHED, duration=3.0, grip=True)
    PLACE_YAW = math.pi
    place_z = PEDESTAL_TOP_Z + CUBE_SIZE / 2.0 + PLACE_SURFACE_CLEARANCE
    x_place = PEDESTAL2_X + ARM_REACH
    y_place = PEDESTAL2_Y

    print("  [TRANSFER] braccio teso, gripper chiuso")
    _goto_with_arm(x_place, y_place, CRUISE_ALT, yaw=PLACE_YAW,
                   duration=4.2, wait=5.2, arm=ARM_STRETCHED, grip=True)

    _set_pick_gains()
    print(f"  [PLACE] scendo a quota lavoro, cubo target z={place_z:.3f}")
    _goto_with_arm(x_place, y_place, PICKUP_ALT, yaw=PLACE_YAW,
                   duration=3.0, wait=3.5, arm=ARM_STRETCHED, grip=True,
                   profile="approach")

    app_j = _ik_joints_from_current_pose(
        PEDESTAL2_X, PEDESTAL2_Y, place_z + IK_APPROACH_Z, "PLACE_APP")
    if app_j is None:
        print("  [PLACE] IK approccio fallita: deposito abortito")
        _restore_flight_gains()
        return False

    print(f"  [PLACE F1] braccio sopra il piedistallo z={place_z + IK_APPROACH_Z:.3f} m")
    move_arm_to(*app_j, duration=3.0, grip=True)
    _hold_arm(None, duration=0.5, grip=True)

    place_j = _ik_joints_from_current_pose(
        PEDESTAL2_X, PEDESTAL2_Y, place_z, "PLACE")
    if place_j is None:
        print("  [PLACE] IK rilascio fallita: torno sopra il piedistallo")
        move_arm_to(*app_j, duration=1.5, grip=True)
        _restore_flight_gains()
        return False

    print(f"  [PLACE F2] cubo a {PLACE_SURFACE_CLEARANCE*100:.1f} cm sopra il piedistallo")
    move_arm_to(*place_j, duration=2.5, grip=True)
    _hold_arm(None, duration=0.5, grip=True)
    open_gripper()
    _hold_arm(place_j, duration=0.8)

    print("  [PLACE EXIT] risalgo col braccio, poi stendo")
    move_arm_to(*app_j, duration=1.6)
    _goto_with_arm(x_place, y_place, CRUISE_ALT, yaw=PLACE_YAW,
                   duration=2.8, wait=3.4, arm=app_j, profile="lift")
    _hold_arm(ARM_STRETCHED, duration=1.0)
    _restore_flight_gains()
    return True


def simulation():
    print("--- INIZIALIZZAZIONE ---")
    setup()

    # Registra handler per SIGINT (Ctrl+C) e SIGTERM dopo che i componenti
    # sono connessi, così stop() può chiuderli correttamente al posto di
    # lasciare i rotori in spin e i log aperti.
    def _abort(sig, frame):
        print(f"\n[ABORT] Segnale {sig} ricevuto — fermo tutto")
        try:
            stop()
        except Exception as e:
            print(f"[ABORT] stop() ha lanciato: {e}")
        sys.exit(0)
    signal.signal(signal.SIGINT,  _abort)
    signal.signal(signal.SIGTERM, _abort)

    print("--- AVVIO MOTORI E CONTROLLORI ---")
    start()

    # Il braccio deve restare teso per tutto il volo di avvicinamento/discesa.
    print("--- BRACCIO TESO + PINZA APERTA ---")
    set_gripper(GRIPPER_OPEN, n_repeat=2, dt=0.05)
    _hold_arm(ARM_STRETCHED, duration=0.1)

    print("--- DECOLLO CON BRACCIO TESO ---")
    takeoff_duration = _takeoff_request(CRUISE_ALT, duration=3.4)
    _hold_arm(ARM_STRETCHED, duration=takeoff_duration + 0.8)

    cube, measured = _read_cube_pose_or_nominal()
    source = "Gazebo" if measured else "nominale"
    print(f"--- VOLO VERSO CUBO ({source}: {cube[0]:.3f},{cube[1]:.3f},{cube[2]:.3f}) ---")
    x_pick = cube[0] - ARM_REACH
    y_pick = cube[1]
    _goto_with_arm(x_pick, y_pick, CRUISE_ALT, yaw=0.0, duration=3.6, wait=4.5)

    print("--- MODALITA' PICK ---")
    _set_pick_gains()
    _enable_af()

    print(f"--- DISCESA A PICKUP_ALT={PICKUP_ALT:.2f} m ---")
    _goto_with_arm(x_pick, y_pick, PICKUP_ALT, yaw=0.0, duration=3.0, wait=3.5,
                   profile="approach")

    grasp_result = _grasp_until_confirmed(x_pick, y_pick)

    _disable_af()
    _restore_flight_gains()

    if grasp_result is None:
        print("--- PRESA FALLITA: CUBO IRRAGGIUNGIBILE — RIENTRO ---")
    else:
        print("--- PRESA CONFERMATA: PASSO AL TRASFERIMENTO ---")
        _place_with_arm_motion()

    print("--- RIENTRO ---")
    _goto_with_arm(0.0, 0.0, CRUISE_ALT, duration=4.5, wait=5.5, profile="cruise")
    _hold_arm(ARM_STRETCHED, duration=1.0)

    print("--- ATTERRAGGIO ---")
    _goto_with_arm(0.0, 0.0, 0.05, yaw=0.0, duration=4.0, wait=6.0,
                   arm=ARM_STRETCHED, profile="landing")  # attende il touchdown

    print("--- OPERAZIONE COMPLETATA ---")
    stop()


if __name__ == "__main__":
    simulation()
