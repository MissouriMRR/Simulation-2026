#!/bin/bash

SESSION_NAME="MultipleRotors"
NCOPTERS="${1:-1}"
OUT_PORT="${2:-14550}"
OUT_HOST="${3:-127.0.0.1}"

# experimental multiprocessing thing
# PROCS=$(nproc)

tmux kill-session -t "$SESSION_NAME"

# Start a new detached tmux session with a placeholder window
tmux new-session -d -s "$SESSION_NAME" -n "init"

echo "Starting $NCOPTERS drones..."

for ((i = 0; i < $NCOPTERS; i++)); do
    WINDOW_NAME="Drone_$i"
    echo "Starting drone $i..."

    DRONE_DIR="/dronedata/drone$i"

    # also part of experimental multiprocessing thing
    # CPU_ID=$((i % PROCS))

    mkdir -p "$DRONE_DIR"

    tmux new-window -d -t "$SESSION_NAME" -n "$WINDOW_NAME"
    tmux send-keys -t "$SESSION_NAME:$WINDOW_NAME" "cd $DRONE_DIR; /ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f airsim-copter -w --add-param-file=/ardupilot/Tools/autotest/multidrone.parm --instance $i" Enter
    
    # experimental --- running drones on different cores. unclear if this works with the funny tmux thing we doing
    # tmux send-keys -t "$SESSION_NAME:$WINDOW_NAME" "cd $DRONE_DIR; taskset -c $CPU_ID /ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f airsim-copter -w --add-param-file=/ardupilot/Tools/autotest/multidrone.parm --instance $i" Enter
    
    # wait until the previous drone is finished building as they must do this individually...
    # NOTE: this is very cooked but it works, and is more reliable than guessing how long to wait
    sleep 0.1
    while (( $(tmux capture-pane -t "$SESSION_NAME:$WINDOW_NAME.0" -pS - | grep -c "BUILD SUMMARY") < 1 )); do
        sleep 0.3
    done
done

# Kill the initial placeholder window
tmux kill-window -t "$SESSION_NAME:init"

# Attach to the session
tmux attach-session -t "$SESSION_NAME"