#!/usr/bin/env python

import asyncio
from websockets.server import serve

clients = {}

async def register(client):
    if client["id"] in clients:
        await clients[client["id"]]["websocket"].send(f"Client {client['id']} already registered")
    else:
        clients[client["id"]] = client
        print(f"Client {client['id']} registered")

async def unregister(client_id):
    if client_id in clients:
        clients.pop(client_id)
        print(f"Client {client_id} unregistered")

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

                # send to it to the client
                if client_id in clients:
                    await clients[client_id]["websocket"].send(f"{command} {args}")

                await websocket.send("Received")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if client_id and client_id in clients:
            await unregister(client_id)

async def main():
    async with serve(read, "192.168.1.104", 8765, ping_interval=None):
        await asyncio.Future()

asyncio.run(main())
