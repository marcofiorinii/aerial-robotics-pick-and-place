# Scripts

| File | Purpose |
| --- | --- |
| `control_new.py` | Main aerial pick-and-place mission: online arm IK, grasp verification, retries, Phynt integration and placement checks |
| `project.sh` | Starts the project world, GenoMix server and seven GenoM/pocolibs components |
| `plot.py` | Offline plotting of POM, Maneuver, Phynt and Rotorcraft logs; defaults to `../logs/last` |
| `motion_arm.py` | Separate ground-fixed arm experiment; the ground arm is disabled in the active project world |
| `control_clean.py` | Alternative controller with different geometry/tuning; does not load Phynt |
| `control.py`, `control_old.py` | Earlier mission implementations |
| `control_optimized.py`, `marco_files/control_optimized.py` | Earlier optimization variants |

Use `control_new.py` for the workflow in the main README. The other versions
are retained as development material and should be read with their own
constants and model assumptions.

The mission scripts connect to GenoM components during import. Inspect their
source without importing them on a machine that is not running the configured
course stack.
