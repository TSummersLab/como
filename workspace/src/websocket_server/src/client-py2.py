#!/usr/bin/env python

import roslaunch
import rospy
import subprocess
import os
import sys
import shlex
import time
from websocket import create_connection

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

NAMESPACE = rospy.get_namespace()
NAMESPACE = NAMESPACE[1:-1] # removes slash at the end

class ROSProgram:
    def __init__(self, filename):
        self.filename = filename
        self.launch = None

    def start(self):
        filepath = os.path.abspath('/home/conlab-minii/como/workspace/src/como_driver/launch/' + self.filename + '.launch')
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [filepath], is_core=True)
        self.launch.start()
        print("ROS Program Started")

    def stop(self):
        if self.launch is not None:
            self.launch.shutdown()
        print("ROS Program Stopped")

def send_identification(websocket):
    client_id = NAMESPACE
    client_type = "COMO"
    websocket.send(client_id + " " + client_type)
    #print("namespace: " + NAMESPACE)
    print("Sent Client ID: {}".format(client_id))

''' 
def prog_cmd(message):
	#method 1 without using xdotool
	terminal_command = ['gnome-terminal', '--', 'bash', '-c', f"{message}; exec bash"]
	
	try:
		subprocess.Popen(terminal_command)
	except Exception as e:
		print(f"Failed to execute command: {e}")'''
		

def prog_cmd(message):
    # Define the terminal command to open a new terminal and keep it open with exec bash
    terminal_command = ['gnome-terminal', '--', 'bash', '-c', 'exec bash']

    try:
        # Open a new terminal
        proc = subprocess.Popen(terminal_command)

        # Give the terminal some time to open
        time.sleep(1.5)

        # Find the window ID of the most recently opened terminal
        window_id = subprocess.check_output(['xdotool', 'search', '--sync', '--onlyvisible', '--class', 'gnome-terminal']).split()[-1].decode('utf-8')

        # Focus on the new terminal window
        subprocess.run(['xdotool', 'windowfocus', window_id])

        # Type the command in the new terminal
        subprocess.run(['xdotool', 'type', message])
        subprocess.run(['xdotool', 'key', 'Return'])

    except Exception as e:
        print(f"Failed to execute command: {e}")

def handle_messages(websocket):
    global program

    while True:
        message = websocket.recv()
        if message is None:
            break
        command = message.split()[0]

        if command == 'init':
            program = ROSProgram(message.split()[1])
        elif command == 'start':
            program.start()
        elif command == 'stop':
            program.stop()
        elif command == 'disconnect':
        	sys.exit()
        elif command == 'prog_cmd':
        	prog_cmd(message.split(maxsplit=1)[1])
        else:
            # Handle other commands or unexpected messages
            print("Unhandled message: {}".format(message))

def run():
    uri = "ws://192.168.1.104:8765"

    websocket = create_connection(uri)
    try:
        send_identification(websocket)
        handle_messages(websocket)
    finally:
        websocket.close()

if __name__ == "__main__":
    run()
    print("Client finished running.")
