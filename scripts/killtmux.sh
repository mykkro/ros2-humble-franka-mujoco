#!/bin/bash

# Kills all TMUX sessions.

tmux list-sessions | awk 'BEGIN{FS=":"}{print $1}' | xargs -n 1 tmux kill-session -t
