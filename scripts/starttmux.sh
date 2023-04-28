#!/bin/bash

tmux new-session \; split-window -h \; split-window -v \; select-pane -t 0 \; split-window -v \; send-keys -t 0 'mc' Enter \; send-keys -t 1 'mc' Enter \;

