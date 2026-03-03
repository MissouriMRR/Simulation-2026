import collections.abc

# Redirect the old location to the new one
collections.MutableMapping = collections.abc.MutableMapping
collections.Mapping = collections.abc.Mapping
collections.Sequence = collections.abc.Sequence

import asyncio
import time
import socket  # For sending python commands directly to UE4

from dronekit import LocationGlobalRelative, VehicleMode, connect
# from flight.camera import CameraAirSim

import asyncio
import os
import tempfile
# import cv2
import numpy as np
from datetime import datetime

from projectairsim import ProjectAirSimClient, Drone, World
from projectairsim.utils import projectairsim_log, unpack_image
from projectairsim.image_utils import ImageDisplay

address = "172.22.32.1"
port_topics = "8990"

client = ProjectAirSimClient(address=address)

client.connect()

world = World(client=client, scene_config_name="/SUAS/simulation/sim_config/scene_suas.jsonc")
drones = [
    Drone(client=client, world=world, name=f"Drone{i}")
    for i in range(1, 3)
]
# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# sock.connect(("127.0.0.1", 9900))
# print("TCP Server Connected @ 127.0.0.1:9900!")

async def main():
    # Set the drone to be ready to fly
    drones[0].enable_api_control()
    drones[0].arm()

    projectairsim_log().info("takeoff_async: starting")
    takeoff_task = (
        await drones[0].takeoff_async()
    )  # schedule an async task to start the command

    # Example 1: Wait on the result of async operation using 'await' keyword
    await takeoff_task
    projectairsim_log().info("takeoff_async: completed")

    # Command the drone to move up in NED coordinate system at 1 m/s for 4 seconds
    move_up_task = await drones[0].move_by_velocity_async(
        v_north=0.0, v_east=0.0, v_down=-1.0, duration=4.0
    )
    projectairsim_log().info("Move-Up invoked")

    await move_up_task
    projectairsim_log().info("Move-Up completed")

    # Control drone
    # type n to move north, s south, e east, w west, u up, and d down
    # you can put multiple at once, such as nnneeeuuu to do multiple commands at once
    # type q to quit/land
    lat, lon, alt = 0.0, 0.0, 0.0
    while True:
        command: str = input("Give instructions: ").lower()

        if command.lower() in ("die", "q", "quit"):
            projectairsim_log().info("land_async: starting")
            land_task = await drone.land_async()
            await land_task
            projectairsim_log().info("land_async: completed")
            break

        lat, lon, alt = 0.0, 0.0, 0.0
        for cmd in command:
            if cmd == "n":
                lat += 1
            elif cmd == "s":
                lat -= 1
            elif cmd == "e":
                lon += 1
            elif cmd == "w":
                lon -= 1
            elif cmd == "u":
                alt -= 5
            elif cmd == "d":
                alt += 5
            # elif cmd == "p":
    #             print("Attempting to capture photo")
    #             await takepic(cam)
    #             print("Photo captured")
    #         elif cmd == "m":
    #             print("Attempting release!")
    #             send_rpc_msg("RELEASE")

        new_move = await drones[0].move_by_velocity_async(
            v_north=lat, v_east=lon, v_down=alt, duration=1.0
        )
        while not new_move.done():
            await asyncio.sleep(0.01)

        projectairsim_log().info("new_move: started")

    # Close vehicle object before exiting script
    drones[0].disarm()
    drones[0].disable_api_control()
    # sock.close() # Release the socket server


# --- RPC Server Send ---
def send_rpc_msg(cmd):
    sock.send(cmd.encode('utf-8'))

if __name__ == "__main__":
    asyncio.run(main())
