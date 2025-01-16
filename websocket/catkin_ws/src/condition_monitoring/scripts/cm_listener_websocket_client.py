import asyncio
import websockets

async def websocket_client(uri):
    async with websockets.connect(uri) as websocket:
        print(f"Connected to {uri}")

        try:
            while True:
                message = await websocket.recv()
                print(f"Received message: {message}")
        except websockets.exceptions.ConnectionClosed:
            print("Connection to the server closed.")

if __name__ == "__main__":
    server_uri = "ws://localhost:8785"  # Replace with your server's URI
    asyncio.run(websocket_client(server_uri))
