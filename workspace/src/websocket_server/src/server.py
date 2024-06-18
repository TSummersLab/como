#!/usr/bin/env python

import asyncio
from websockets.server import serve

# Dictionary to keep track of connected clients
clients = {}

# Register a new client
async def register(client):
	# If client is already in dictionary, send client already registered"
    if client["id"] in clients:
        await clients[client["id"]]["websocket"].send(f"Client {client['id']} already registered")
    else: # else register new client
        clients[client["id"]] = client
        print(f"Client {client['id']} registered")

# Remove client from clients dictionary
async def unregister(client_id):
    if client_id in clients:
        clients.pop(client_id)
        print(f"Client {client_id} unregistered")

# Handle incoming messages from clients
async def read(websocket, path):
    client_id = None
    try:
        async for message in websocket:
            print(f"Received: {message}")
            if client_id is None:
                message = message.split()
                client_id = message[0]
                client_type = message[1]

                client = {
                    "id": client_id,
                    "type": client_type,
                    "websocket": websocket
                }

                await register(client)
            else:
                # Handle commands from client
                message = message.split(maxsplit=2)

                client_id = message[0]
                command = message[1]
                args = message[2] if len(message) > 2 else ""

                # Send commands and arguments to specific clients based on client_id/namespace
                if client_id == "all":
                	for cid, client in clients.items():
                		if client["type"] != "cli":
                			await client["websocket"].send(f"{command} {args}")
                if client_id in clients:
                    await clients[client_id]["websocket"].send(f"{command} {args}")

                await websocket.send("Received")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if client_id and client_id in clients:
            await unregister(client_id)

# Start the server
async def main():
    # IP address, check ip address by running ifconfig and update below accordingly.
    async with serve(read, "192.168.0.180", 8765, ping_interval=None):
        await asyncio.Future()

asyncio.run(main())
