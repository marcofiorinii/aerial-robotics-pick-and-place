#!/bin/sh

middleware=pocolibs
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
project_root=$(CDPATH= cd -- "$script_dir/.." && pwd)
gz_world="$project_root/gazebo/worlds/project.world"
export TK3LAB_WS="${TK3LAB_WS:-$project_root}"
export GZ_SIM_RESOURCE_PATH="$project_root/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

if [ ! -f "$gz_world" ]; then
    printf 'Cannot find world file: %s\n' "$gz_world" >&2
    exit 1
fi

# PTY virtuale creato da dxsim-gazebo.so per il braccio del drone.
# In simulazione punta al pty di Gazebo; su hardware reale cambiare in /dev/ttyUSB0
export DYNAMIXEL_SERIAL=/tmp/pty-fahexa-arm

# Genom3 components to run
components="
  nhfc
  phynt
  maneuver
  pom
  optitrack
  rotorcraft
  dynamixel
"

# list of process ids to clean, populated after each spawn
pids=

# cleanup, called after ctrl-C
atexit() {
    trap - 0 INT CHLD
    set +e

    kill $pids
    wait
    case $middleware in
        pocolibs) h2 end;;
    esac
    exit 0
}
trap atexit 0 INT
set -e

case $middleware in
    pocolibs) h2 init;;
    ros) roscore & pids="$pids $!";;
    *) echo "invalid middleware: $middleware";
esac

# optionally run a genomix server for remote control
genomixd & pids="$pids $!"

# spawn required components
for c in $components; do
    $c-$middleware & pids="$pids $!"
done

# start gazebo
gz sim "$gz_world" & pids="$pids $!"

# wait for ctrl-C or any background process failure
trap atexit CHLD
wait
