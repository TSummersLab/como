#!/usr/bin/env python

import roslaunch
import rospy
import subprocess
import os
import sys
import time
import getpass
from websocket import create_connection

# Generate or retrieve unique roslaunch identifier
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

# Retrieve unique namespace of the client
NAMESPACE = rospy.get_namespace()
NAMESPACE = NAMESPACE[1:-1] # removes slash at the end and beginning
USER = getpass.getuser()

# Class for starting and stopping ros program
class ROSProgram:
    def __init__(self, packagename, filename):
        self.filename = filename
        self.packagename = packagename
        self.launch = None

    # Function to start a ros program using roslaunch api
    def start(self):
    	# Absolute path used to run .launch files. Update path as needed
        filepath = os.path.abspath('/home/' + USER + '/como/workspace/src/' + self.packagename + '/launch/' + self.filename + '.launch')
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [filepath], is_core=True)
        self.launch.start()
        print("ROS Program Started")

    # Function to stop ros program if it is running
    def stop(self):
        if self.launch is not None:
            self.launch.shutdown()
        print("ROS Program Stopped")

# Function to send client identification to server
def send_identification(websocket):
    client_id = NAMESPACE
    client_type = "COMO"
    websocket.send(client_id + " " + client_type)
    #print("namespace: " + NAMESPACE)
    print("Sent Client ID: {}".format(client_id))

# Directly calls ros service
def call_rosservice(message):
	msg = message.split()
	srv_name = msg[2]
	srv_args = " ".join(msg[3:])
	try:
		srv_args = srv_args[1:-1]
		srv_command = ['rosservice', 'call', srv_name, srv_args]
		subprocess.call(srv_command)
	except subprocess.CalledProcessError as e:
		print("Service call failed: {}".format(e.output))


# Opens a new terminal and runs any terminal command
def prog_cmd(message):
	msg = " ".join(message.split()[1:])
	terminal_command = ['gnome-terminal', '--', 'bash', '-c', "{}; exec bash".format(msg)]
	
	try:
		subprocess.Popen(terminal_command)
	except Exception as e:
		print("Failed to execute command: {}".format(msg))


'''
# Opens a new terminal, types in a terminal command using xdotool and executes it
def prog_cmd(message):
	msg = " ".join(message.split()[1:])
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
		subprocess.call(['xdotool', 'windowfocus', window_id])

		# Type the command in the new terminal
		subprocess.call(['xdotool', 'type', msg])
		subprocess.call(['xdotool', 'key', 'Return'])

	except Exception as e:
		print("Failed to execute command: {}".format(e))'''

# Function for handling messages received from the server
def handle_messages(websocket):
    global program

    while True:
        message = websocket.recv()
        if message is None:
            break
        command = message.split()[0]

        if command == 'init':
            program = ROSProgram(message.split()[1], message.split()[2])	# Initializes ros program
        elif command == 'start':
            program.start()
        elif command == 'stop':
            program.stop()
        elif command == 'disconnect':
        	sys.exit()	# Disconnects client from server
        elif command == 'prog_cmd':
        	prog_cmd(message)	# Handles custom messages
        elif command == 'rosservice':
        	call_rosservice(message)
        else:
        	print("Unhandled message: {}".format(message))

def run():
    uri = "ws://192.168.0.180:8765" # IP address, check ip address by running ifconfig and update accordingly.

    websocket = create_connection(uri) # websocket cocnnection to server
    try:
        send_identification(websocket)
        handle_messages(websocket) # Handle incoming messages from server
    finally:
        websocket.close()

if __name__ == "__main__":
    run()
    print("Client finished running.")
