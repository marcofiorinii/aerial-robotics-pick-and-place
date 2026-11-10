# Reproducibility and configuration notes

## Scope of the repository

The main mission is `Codici/control_new.py`, as identified by the original
repository's development note and the latest upstream commit, `1672010`.
The accompanying PDF describes the project and reports ten simulation
trials. It should be read together with the configuration differences below.

The publication update preserves the mission controller and all SDF/world
parameters. It adds documentation and the report, fixes launcher paths, and
lets the plotting script read the logs produced by the main mission.

## Report and code differences

| Parameter | Report | Checked-in main mission/model |
| --- | ---: | ---: |
| Gripper-finger effort limit | 100 N | 60 N for each finger |
| Release clearance above the supported cube center | 0.018 m | 0.050 m |
| Nominal cube-center release height | 0.243 m | 0.275 m |
| Allowed cube-center height at release | 0.231-0.257 m | 0.257-0.297 m |

The source values are in `gazebo/models/fah_arm/model.sdf` and the
`PLACE_RELEASE_*` constants in `Codici/control_new.py`. The PDF gives the
report values in Sections 4.4 and Equations 26-28. The horizontal release
radius of 0.085 m agrees with the final release check in the main controller.

The ten-run statistics are reported results, not a benchmark independently
reproduced for this checkout. Matching those experiments requires the
controller/model configuration actually used for those runs, or new trials
with the current snapshot and an updated report. Changing parameters without
rerunning the simulation would not resolve that difference.

## External environment

The project was developed in a course-provided Docker environment. This
repository does not contain its image identifier, image digest, Dockerfile,
initialization script or exact component/plugin versions. Those are required
to make the environment independently reproducible.

Before running, initialize that environment so that Gazebo, the GenoM
executables, Python `genomix` and the shared-library plugins can be found.
The main mission additionally expects GenoM plugins at
`$HOME/openrobots/lib/genom/pocolibs/plugins`.

The launcher adds the repository's `gazebo/models` directory to
`GZ_SIM_RESOURCE_PATH`; it does not install or build the external framework.

## Logs and evaluation

The main mission creates `logs/last/`, containing state/reference logs,
component diagnostics and the grasp-debug CSV. Each new invocation removes
the previous `logs/last/` directory. Archive each trial before restarting.

The raw logs and individual outcome records for the ten reported trials
are not included. Adding an anonymized trial summary and the corresponding
logs would support checking the reported counts and regenerating the analysis.

Do not treat the final console message as a success flag: the mission returns
and lands after some failed manipulation attempts too. Record successful
placement from the cube state, using the success criteria described in the
report.

## Development variants

Earlier controllers remain available in `Codici/`. They are not interchangeable
with `control_new.py`: link lengths, altitude choices, grasp logic, logging
paths and use of Phynt differ between versions. The ground-arm model is a
separate experiment and is commented out in the active world.

## Verification of this publication update

Python syntax, XML structure, local documentation links, model references,
report text preservation and the plotting command can be checked locally.
An end-to-end mission rerun requires the external Gazebo/TeleKyb3 environment.
