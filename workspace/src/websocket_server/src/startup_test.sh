#!/bin/bash

gnome-terminal &

sleep 5

xdotool type 'python ~/como/workspace/src/websocket_server/src/client-py2.py'
xdotool key Return
