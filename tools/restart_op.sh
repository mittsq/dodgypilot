#!/usr/bin/bash

tmux kill-session -t comma
rm -f /tmp/safe_staging_overlay.lock
tmux new -s comma -d "echo $$ > /dev/cpuset/app/tasks && echo $PPID > /dev/cpuset/app/tasks && /data/openpilot/launch_openpilot.sh"
