#!/bin/bash

# Starts NCOPTERS ArduPilot SITL instances, one per tmux window, each wired to its own
# drone in the ProjectAirSim scene.
#
# Port arithmetic MUST stay in sync with simulation/interfaces/iarc.py. Drone i (0-based):
#   MAVLink TCP  5760 + 10i  serial0, claimed by MAVProxy
#   MAVLink TCP  5762 + 10i  serial1, what dronekit connects to (state_machine/drone.py)
#   UDP in       9003 + 10i  AirSim -> SITL, sensor data
#   UDP out      9002 + 10i  SITL -> AirSim, servo output
#
# All four come from --instance: the SITL binary's own help says it "adds 10*instance to
# all port numbers", which covers the AirSim UDP pair as well as the MAVLink TCP ports.
# Do not try to set the UDP ports explicitly here -- --sim-port-in/--sim-port-out are
# options on the arducopter binary, not on sim_vehicle.py, so they would have to go
# through -A, and they would then double up with the offset --instance already applied.

set -u

SESSION_NAME="MultipleRotors"
NCOPTERS="${1:-${NUM_DRONES:-1}}"
OUT_PORT="${2:-${OUT_PORT:-14550}}"
OUT_HOST="${3:-${OUT_HOST:-127.0.0.1}}"

# Where the Unreal/ProjectAirSim server is reachable from inside this container. When
# Unreal runs on Windows and this container runs in WSL, set it to the Windows-side WSL
# adapter IP (`ipconfig` -> "vEthernet (WSL)"), e.g. AIRSIM_HOST=172.27.192.1.
AIRSIM_HOST="${AIRSIM_HOST:-127.0.0.1}"

PORT_STRIDE=10

echo "Starting $NCOPTERS drone(s), AirSim at $AIRSIM_HOST"

tmux kill-session -t "$SESSION_NAME" 2>/dev/null

# Start a new detached tmux session with a placeholder window
tmux new-session -d -s "$SESSION_NAME" -n "init"

for ((i = 0; i < NCOPTERS; i++)); do
    WINDOW_NAME="Drone_$i"

    # Each instance needs its own working directory: eeprom.bin, logs and terrain cache
    # are written to cwd, and instances sharing a directory corrupt each other's state.
    DRONE_DIR="/dronedata/drone$i"
    mkdir -p "$DRONE_DIR"

    # Give each instance its own GCS forwarding port, otherwise every SITL blindly UDPs
    # into the same 14550 and the streams interleave into garbage.
    INSTANCE_OUT_PORT=$((OUT_PORT + i * PORT_STRIDE))

    echo "Starting drone $i: sim in $((9003 + i * PORT_STRIDE)), sim out $((9002 + i * PORT_STRIDE)), mavlink tcp $((5762 + i * PORT_STRIDE))"

    tmux new-window -d -t "$SESSION_NAME" -n "$WINDOW_NAME"
    tmux send-keys -t "$SESSION_NAME:$WINDOW_NAME" \
        "cd $DRONE_DIR; python /ardupilot/Tools/autotest/sim_vehicle.py \
            -v ArduCopter \
            -f airsim-copter \
            -w \
            --add-param-file=/ardupilot/Tools/autotest/multidrone.parm \
            --instance $i \
            --auto-sysid \
            --sim-address=$AIRSIM_HOST \
            --out=$OUT_HOST:$INSTANCE_OUT_PORT" Enter

    # Wait until this instance finishes building before starting the next. Concurrent waf
    # builds in the same tree collide, and this is more reliable than guessing a delay.
    sleep 0.1
    while (( $(tmux capture-pane -t "$SESSION_NAME:$WINDOW_NAME.0" -pS - | grep -c "BUILD SUMMARY") < 1 )); do
        sleep 0.3
    done
done

# Kill the initial placeholder window
tmux kill-window -t "$SESSION_NAME:init"

# Attach to the session
tmux attach-session -t "$SESSION_NAME"
