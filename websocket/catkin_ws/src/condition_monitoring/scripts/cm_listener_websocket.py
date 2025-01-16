#!/usr/bin/env python

import json
import asyncio
import websockets
import rospy
from condition_monitoring.msg import msg_cm as RosJointState
# from cm_listener.msg import msg_cm as RosJointState

class WebSocketServer:
    def __init__(self, loop, port=8765):
        self.websocket_uri = f"ws://localhost:{port}"
        self.websocket = None
        self.websocket_connected = False
        self.shutdown_event = asyncio.Event()
        self.loop = loop

    async def parse_msg(self, message):
        position = message.position
        temperature = message.temperature
        voltage = message.voltage
        time_stamp = message.header.stamp.to_sec()
        time_stamp = time_stamp % 1e3

        motor_names = ['motor1', 'motor2', 'motor3', 'motor4', 'motor5', 'motor6']
        output = {}
        for i in range(len(motor_names)):
            tmap = {
                "a": position[i],
                "t": temperature[i],
                "v": voltage[i]
                }
            output[motor_names[i]] = tmap

        final_out = {"data": output, "time": time_stamp}
        json_str = json.dumps(final_out)

        return json_str

    async def ros_callback(self, msg):
        # print(msg)
        if self.websocket_connected:
            parsed_data = await self.parse_msg(msg)
            # print("sending msg at {}".format(parsed_data.time))
            await self.websocket.send(parsed_data)
            await asyncio.sleep(0)  # yield control to the event loop

    async def websocket_handler(self, websocket, path):
        self.websocket_connected = True
        self.websocket = websocket

        print(f"Client connected from {websocket.remote_address}")
        
        try:
            while not self.shutdown_event.is_set() and not rospy.is_shutdown():
                # Check if the connection is closed
                if self.websocket.closed:
                    print(f"Client {websocket.remote_address} disconnected!")
                    self.websocket_connected = False
                    break
                await asyncio.sleep(1)  # Check every second for WebSocket connection
        except asyncio.CancelledError:
            pass
        finally:            
            # Check if the connection is closed before attempting to close it
            if not self.websocket.closed:
                await self.websocket.close()
            

def ros_callback_wrapper(websocket_server, msg):
    asyncio.run_coroutine_threadsafe(websocket_server.ros_callback(msg), loop=websocket_server.loop)

async def main(port=8785):
    loop = asyncio.get_event_loop()
    websocket_server = WebSocketServer(loop, port)
    
    # Start the WebSocket server
    start_server = await websockets.serve(
        websocket_server.websocket_handler,
        "localhost",  # Set your desired host
        port          # Set your desired port
    )

    print(f"WebSocket server started. Listening on {start_server.sockets[0].getsockname()}")

    # Set up ROS node and subscriber
    rospy.init_node("cm_listener_websocket", anonymous=True)
    rospy.Subscriber("condition_monitoring", RosJointState, lambda msg: ros_callback_wrapper(websocket_server, msg))

    try:
        while not websocket_server.shutdown_event.is_set() and not rospy.is_shutdown():
            await asyncio.sleep(1)  # Perform other tasks or checks
    except KeyboardInterrupt:
        print("Received KeyboardInterrupt. Shutting down...")
    finally:
        print("Server shutdown. Cleaning up resources.")
        # Explicitly close the WebSocket server
        if start_server:
            start_server.close()
            await start_server.wait_closed()

if __name__ == "__main__":
    port = 8785
    # asyncio.run(main(port))
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main(port))
    loop.close()
