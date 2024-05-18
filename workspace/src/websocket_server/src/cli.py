#!/usr/bin/env python

import asyncio
from websockets import connect
import json

async def login(websocket):
    await websocket.send(input() + " cli")

async def send_command(websocket, client_id, command, args):
    await websocket.send(client_id + " " + command + " " + args)

async def run():
    uri = "ws://192.168.1.104:8765"

    async with connect(uri, ping_interval=None) as websocket:
        await login(websocket)

        print("Connected to server. You can now enter your commands.")

        # Continuously accept input commands from the user
        while True:
            input_cmd = input("> ")

            # If the user types 'exit', break the loop and end the program
            if input_cmd.lower() == "exit":
                print("Exiting.")
                break

            parts = input_cmd.split(maxsplit=2)
            if len(parts) < 2:
                print("Invalid command. Format: client_id command [arguments]")
                continue

            client_id = parts[0]
            command = parts[1]
            args = parts[2] if len(parts) > 2 else ""  # Assign empty string if no args

            await send_command(websocket, client_id, command, args)

asyncio.run(run())

print("Client finished running.")
