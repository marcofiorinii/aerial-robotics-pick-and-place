import argparse
import os
import math
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ============================================================
# CONFIG
# ============================================================
DEFAULT_LOG_DIR = Path(__file__).resolve().parent.parent / "logs" / "last"

XYZ_COLORS = {"x": "r", "y": "g", "z": "b"}
RPY_COLORS = {"roll": "r", "pitch": "g", "yaw": "b"}
WXYZ_COLORS = {"wx": "r", "wy": "g", "wz": "b"}
FORCE_COLORS = {"fx": "r", "fy": "g", "fz": "b"}
TORQUE_COLORS = {"tx": "r", "ty": "g", "tz": "b"}

N_PROPS = 6


# ============================================================
# LOG READING
# ============================================================
def read_log(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"Missing log file: {path}")

    with open(path, "r") as f:
        lines = [line.strip() for line in f if line.strip()]

    data_lines = [line for line in lines if not line.startswith("#")]
    if not data_lines:
        raise ValueError(f"No data lines found in {path}")

    header = data_lines[0].split()
    rows = [line.split() for line in data_lines[1:]]

    df = pd.DataFrame(rows, columns=header)
    df = df.replace("-", np.nan)

    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    return df


def normalize_time(dfs):
    t0 = min(df["ts"].min() for df in dfs if "ts" in df.columns and not df.empty)
    for df in dfs:
        if "ts" in df.columns:
            df["t"] = df["ts"] - t0


# ============================================================
# HELPERS
# ============================================================
def first_existing(df, candidates):
    for c in candidates:
        if c in df.columns:
            return c
    return None


def safe_plot(ax, df, xcol, y_candidates, **kwargs):
    if isinstance(y_candidates, str):
        y_candidates = [y_candidates]
    ycol = first_existing(df, y_candidates)
    if ycol is None or xcol not in df.columns:
        return None
    valid = df[[xcol, ycol]].dropna()
    if valid.empty:
        return None
    ax.plot(valid[xcol], valid[ycol], **kwargs)
    return ycol


def align_to_time(df_source, t_target):
    """
    Interpolate every numeric column of df_source on t_target.
    Returns a new DataFrame with column 't' and all interpolated columns.
    """
    if "t" not in df_source.columns:
        raise ValueError("Source dataframe has no 't' column")

    out = pd.DataFrame({"t": t_target})

    t_src = df_source["t"].to_numpy()
    valid_t = np.isfinite(t_src)

    for col in df_source.columns:
        if col in ["ts", "t"]:
            continue

        y = df_source[col].to_numpy()
        valid = valid_t & np.isfinite(y)

        if np.count_nonzero(valid) < 2:
            out[col] = np.nan
            continue

        out[col] = np.interp(t_target, t_src[valid], y[valid])

    return out


def add_euler_from_quat(df, qw="qw", qx="qx", qy="qy", qz="qz",
                        roll_name="roll", pitch_name="pitch", yaw_name="yaw"):
    needed = [qw, qx, qy, qz]
    if not all(c in df.columns for c in needed):
        return

    roll_list = []
    pitch_list = []
    yaw_list = []

    for w, x, y, z in zip(df[qw], df[qx], df[qy], df[qz]):
        if any(pd.isna(v) for v in [w, x, y, z]):
            roll_list.append(np.nan)
            pitch_list.append(np.nan)
            yaw_list.append(np.nan)
            continue

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        roll_list.append(roll)
        pitch_list.append(pitch)
        yaw_list.append(yaw)

    df[roll_name] = roll_list
    df[pitch_name] = pitch_list
    df[yaw_name] = yaw_list


def prepare_logs(pom, maneuver, phynt):
    # measured state
    add_euler_from_quat(pom, "qw", "qx", "qy", "qz", "roll", "pitch", "yaw")

    # nominal reference
    add_euler_from_quat(maneuver, "qw", "qx", "qy", "qz", "roll", "pitch", "yaw")

    # admittance-filtered desired
    add_euler_from_quat(phynt, "qw", "qx", "qy", "qz", "roll", "pitch", "yaw")


# ============================================================
# COLUMN MAPS
# ============================================================
POS_REF = {
    "x": ["x", "xd", "x_ref", "xref"],
    "y": ["y", "yd", "y_ref", "yref"],
    "z": ["z", "zd", "z_ref", "zref"],
}

VEL_REF = {
    "vx": ["vx", "vxd", "vx_ref", "vxref"],
    "vy": ["vy", "vyd", "vy_ref", "vyref"],
    "vz": ["vz", "vzd", "vz_ref", "vzref"],
}

ACC_REF = {
    "ax": ["ax", "axd", "ax_ref", "axref"],
    "ay": ["ay", "ayd", "ay_ref", "ayref"],
    "az": ["az", "azd", "az_ref", "azref"],
}

ATT_REF = {
    "roll": ["roll", "rolld", "roll_ref", "rollref"],
    "pitch": ["pitch", "pitchd", "pitch_ref", "pitchref"],
    "yaw": ["yaw", "yawd", "yaw_ref", "yawref"],
}

AVEL_REF = {
    "wx": ["wx", "wxd", "wx_ref", "wxref"],
    "wy": ["wy", "wyd", "wy_ref", "wyref"],
    "wz": ["wz", "wzd", "wz_ref", "wzref"],
}

EXT_FORCE = {
    "fx": ["fx", "force_x", "ext_fx", "fext_x"],
    "fy": ["fy", "force_y", "ext_fy", "fext_y"],
    "fz": ["fz", "force_z", "ext_fz", "fext_z"],
}

EXT_TORQUE = {
    "tx": ["tx", "torque_x", "ext_tx", "text_x"],
    "ty": ["ty", "torque_y", "ext_ty", "text_y"],
    "tz": ["tz", "torque_z", "ext_tz", "text_z"],
}


# ============================================================
# FIGURE 1: TRACKING
# ============================================================
def plot_tracking_figure(pom, phynt_ref):
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    fig.suptitle("Tracking", fontsize=14)

    # pos
    ax = axes[0, 0]
    for meas, key in [("x", "x"), ("y", "y"), ("z", "z")]:
        safe_plot(ax, pom, "t", meas,
                  color=XYZ_COLORS[key], linestyle="-", label=f"{key} meas")
        safe_plot(ax, phynt_ref, "t", POS_REF[meas],
                  color=XYZ_COLORS[key], linestyle="--", label=f"{key} ref")
    ax.set_title("pos (x,y,z)")
    ax.set_ylabel("[m]")
    ax.grid(True)
    ax.legend()

    # att
    ax = axes[0, 1]
    for meas in ["roll", "pitch", "yaw"]:
        safe_plot(ax, pom, "t", meas,
                  color=RPY_COLORS[meas], linestyle="-", label=f"{meas} meas")
        safe_plot(ax, phynt_ref, "t", ATT_REF[meas],
                  color=RPY_COLORS[meas], linestyle="--", label=f"{meas} ref")
    ax.set_title("att (roll,pitch,yaw)")
    ax.set_ylabel("[rad]")
    ax.grid(True)
    ax.legend()

    # vel
    ax = axes[1, 0]
    for meas, key in [("vx", "x"), ("vy", "y"), ("vz", "z")]:
        safe_plot(ax, pom, "t", meas,
                  color=XYZ_COLORS[key], linestyle="-", label=f"{meas} meas")
        safe_plot(ax, phynt_ref, "t", VEL_REF[meas],
                  color=XYZ_COLORS[key], linestyle="--", label=f"{meas} ref")
    ax.set_title("vel (v_x,v_y,v_z)")
    ax.set_ylabel("[m/s]")
    ax.grid(True)
    ax.legend()

    # avel
    ax = axes[1, 1]
    for meas in ["wx", "wy", "wz"]:
        safe_plot(ax, pom, "t", meas,
                  color=WXYZ_COLORS[meas], linestyle="-", label=f"{meas} meas")
        safe_plot(ax, phynt_ref, "t", AVEL_REF[meas],
                  color=WXYZ_COLORS[meas], linestyle="--", label=f"{meas} ref")
    ax.set_title("avel (w_x,w_y,w_z)")
    ax.set_ylabel("[rad/s]")
    ax.grid(True)
    ax.legend()

    # acc
    ax = axes[2, 0]
    for meas, key in [("ax", "x"), ("ay", "y"), ("az", "z")]:
        safe_plot(ax, pom, "t", meas,
                  color=XYZ_COLORS[key], linestyle="-", label=f"{meas} meas")
        safe_plot(ax, phynt_ref, "t", ACC_REF[meas],
                  color=XYZ_COLORS[key], linestyle="--", label=f"{meas} ref")
    ax.set_title("acc (a_x,a_y,a_z)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("[m/s²]")
    ax.grid(True)
    ax.legend()

    axes[2, 1].axis("off")

    for a in axes.flat:
        if a.has_data():
            a.set_xlim(left=0.0)

    fig.tight_layout()


# ============================================================
# FIGURE 2: ERRORS
# ============================================================
def plot_error_figure(pom, phynt_ref):
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    fig.suptitle("Errors", fontsize=14)

    # e_pos
    ax = axes[0, 0]
    for axis, key in [("x", "x"), ("y", "y"), ("z", "z")]:
        ref_col = first_existing(phynt_ref, POS_REF[axis])
        if axis in pom.columns and ref_col is not None:
            e = pom[axis] - phynt_ref[ref_col]
            ax.plot(pom["t"], e, color=XYZ_COLORS[key], label=f"e_{axis}")
    ax.set_title("e_pos (e_x,e_y,e_z)")
    ax.set_ylabel("[m]")
    ax.grid(True)
    ax.legend()

    # e_att
    ax = axes[0, 1]
    for axis in ["roll", "pitch", "yaw"]:
        ref_col = first_existing(phynt_ref, ATT_REF[axis])
        if axis in pom.columns and ref_col is not None:
            e = pom[axis] - phynt_ref[ref_col]
            ax.plot(pom["t"], e, color=RPY_COLORS[axis], label=f"e_{axis}")
    ax.set_title("e_att (e_roll,e_pitch,e_yaw)")
    ax.set_ylabel("[rad]")
    ax.grid(True)
    ax.legend()

    # e_vel
    ax = axes[1, 0]
    for axis, key in [("vx", "x"), ("vy", "y"), ("vz", "z")]:
        ref_col = first_existing(phynt_ref, VEL_REF[axis])
        if axis in pom.columns and ref_col is not None:
            e = pom[axis] - phynt_ref[ref_col]
            ax.plot(pom["t"], e, color=XYZ_COLORS[key], label=f"e_{axis}")
    ax.set_title("e_vel (e_vx,e_vy,e_vz)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("[m/s]")
    ax.grid(True)
    ax.legend()

    # e_avel
    ax = axes[1, 1]
    for axis in ["wx", "wy", "wz"]:
        ref_col = first_existing(phynt_ref, AVEL_REF[axis])
        if axis in pom.columns and ref_col is not None:
            e = pom[axis] - phynt_ref[ref_col]
            ax.plot(pom["t"], e, color=WXYZ_COLORS[axis], label=f"e_{axis}")
    ax.set_title("e_avel (e_wx,e_wy,e_wz)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("[rad/s]")
    ax.grid(True)
    ax.legend()

    for a in axes.flat:
        if a.has_data():
            a.set_xlim(left=0.0)

    fig.tight_layout()


# ============================================================
# FIGURE 3: PROPELLERS
# ============================================================
def plot_propeller_figure(rotorcraft):
    fig, ax = plt.subplots(figsize=(12, 6))
    fig.suptitle("f_props for each i in [1,n_props]", fontsize=14)

    prop_colors = ["r", "g", "b", "m", "c", "y"]

    found_any = False
    for i in range(N_PROPS):
        candidates = [f"meas_v{i}", f"cmd_v{i}", f"v{i}", f"rotor{i}"]
        col = first_existing(rotorcraft, candidates)
        if col is not None:
            valid = rotorcraft[["t", col]].dropna()
            if not valid.empty:
                ax.plot(valid["t"], valid[col],
                        color=prop_colors[i], linestyle="-", label=col)
                found_any = True

    ax.set_xlabel("time [s]")
    ax.set_ylabel("[Hz]")
    ax.grid(True)
    if found_any:
        ax.legend(ncol=2)
    ax.set_xlim(left=0.0)
    fig.tight_layout()


# ============================================================
# FIGURE 4: EXTERNAL FORCES / TORQUES
# ============================================================
def plot_external_wrench_figure(phynt):
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle("Estimated external forces and torques", fontsize=14)

    # forces
    ax = axes[0]
    for key in ["fx", "fy", "fz"]:
        safe_plot(ax, phynt, "t", EXT_FORCE[key],
                  color=FORCE_COLORS[key], linestyle="-", label=key)
    ax.set_title("external force")
    ax.set_ylabel("[N]")
    ax.grid(True)
    ax.legend()

    # torques
    ax = axes[1]
    for key in ["tx", "ty", "tz"]:
        safe_plot(ax, phynt, "t", EXT_TORQUE[key],
                  color=TORQUE_COLORS[key], linestyle="-", label=key)
    ax.set_title("external torque")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("[Nm]")
    ax.grid(True)
    ax.legend()

    for a in axes:
        a.set_xlim(left=0.0)

    fig.tight_layout()


# ============================================================
# FIGURE 5: NOMINAL vs ADMITTANCE TRAJECTORY
# ============================================================
def plot_nominal_vs_admittance_figure(maneuver_ref, phynt_ref):
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("Nominal reference vs trajectory from admittance filter", fontsize=14)

    for idx, axis in enumerate(["x", "y", "z"]):
        safe_plot(axes[idx], maneuver_ref, "t", POS_REF[axis],
                  color=XYZ_COLORS[axis], linestyle="--", label=f"{axis} nominal")
        safe_plot(axes[idx], phynt_ref, "t", POS_REF[axis],
                  color=XYZ_COLORS[axis], linestyle="-", label=f"{axis} admittance")
        axes[idx].set_ylabel(axis)
        axes[idx].grid(True)
        axes[idx].legend()

    axes[-1].set_xlabel("time [s]")
    fig.tight_layout()


# ============================================================
# MAIN
# ============================================================
def main(argv=None):
    parser = argparse.ArgumentParser(description="Plot a recorded aerial pick-and-place simulation.")
    parser.add_argument(
        "--log-dir", type=Path, default=DEFAULT_LOG_DIR,
        help="Directory containing pom.log, maneuver.log, phynt.log and rotorcraft.log (default: logs/last).",
    )
    args = parser.parse_args(argv)
    pom = read_log(args.log_dir / "pom.log")
    maneuver = read_log(args.log_dir / "maneuver.log")
    phynt = read_log(args.log_dir / "phynt.log")
    rotorcraft = read_log(args.log_dir / "rotorcraft.log")

    normalize_time([pom, maneuver, phynt, rotorcraft])

    prepare_logs(pom, maneuver, phynt)

    # allinea reference e phynt sul tempo di pom per tracking/errori
    phynt_on_pom = align_to_time(phynt, pom["t"].to_numpy())
    maneuver_on_pom = align_to_time(maneuver, pom["t"].to_numpy())

    plot_tracking_figure(pom, phynt_on_pom)
    plot_error_figure(pom, phynt_on_pom)
    plot_propeller_figure(rotorcraft)
    plot_external_wrench_figure(phynt)
    plot_nominal_vs_admittance_figure(maneuver, phynt)

    plt.show()


if __name__ == "__main__":
    main()
