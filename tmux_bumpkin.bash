#! /bin/bash

##### LICENSE #####
# © Bumpkin 2025 ©
# Using this script means every user agrees to give the author (@ranais) free food
# for the rest of his life. This is a legally binding contract.
# If you do not agree to these terms, may your hand-eye calibration be fkd
###################

SESSION="bumpkin"

# Check if the session already exists
tmux has-session -t $SESSION 2>/dev/null

if [ $? != 0 ]; then
    # Create a new session and first window for roscore
    tmux new-session -d -s $SESSION -n 'roscore'
    tmux send-keys -t $SESSION:0 'roscore' C-m

    # Create a new window for Google Chrome
    tmux new-window -t $SESSION:2 -n 'google-chrome'
    tmux send-keys -t $SESSION:2 'ssh -X student@iam-bashful' C-m
    tmux send-keys -t $SESSION:2 'google-chrome' C-m

    # Create a new window for control-pc
    tmux new-window -t $SESSION:3 -n 'control-pc'
    tmux send-keys -t $SESSION:3 'cd /home/student/Documents/frankapy' C-m
    tmux send-keys -t $SESSION:3 'bash ./bash_scripts/start_control_pc.sh -u student -i iam-bashful -g 0' C-m

    # Create a new window for running docker-terminal
    tmux new-window -t $SESSION:4 -n 'run docker-terminal'
    tmux send-keys -t $SESSION:4 'cd /home/student/ws_bumpkin/bumpkin' C-m
    tmux send-keys -t $SESSION:4 'bash ./run_docker.sh' C-m

    # sleep because tun docker is SLOW
    sleep 1s

    # Create a new window for joining docker-terminal
    tmux new-window -t $SESSION:5 -n 'join docker-terminal'
    tmux send-keys -t $SESSION:5 'cd /home/student/ws_bumpkin/bumpkin' C-m
    tmux send-keys -t $SESSION:5 'bash ./terminal_docker.sh && pip install mediapipe && cp src/devel_packages/bumpkin/src/hand_landmarker.task /root/.ros
' C-m
fi

# Attach to the session
tmux attach-session -t $SESSION
