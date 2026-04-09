"""
Copyright (C) Microsoft Corporation.
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Demonstrates flying a FastPhysics quadrotor using an ardupilot controller.

Note: Ardupilot controller also should be running for the Iris airframe
      (FRAME_CLASS 1 and FRAME_TYPE 1) see Project AirSim docs for more info.

      Mission Planner can be used to control the drone.
"""

import asyncio

from projectairsim import ProjectAirSimClient, Drone, World
from projectairsim.utils import projectairsim_log
# from projectairsim.image_utils import ImageDisplay


# Async main function to wrap async drone commands
async def main():
    # Create a Project AirSim client
    client = ProjectAirSimClient()

    # Initialize an ImageDisplay object to position up to 2 pop-up sub-windows
    # image_display = ImageDisplay()

    try:
        # Connect to simulation environment
        client.connect()

        # Create a World object to interact with the sim world and load a scene
        world = World(client, "/SUAS/simulation/sim_config/scene_ardu_quadrotor.jsonc", delay_after_load_sec=2)

        # Create a Drone object to interact with a drone in the loaded sim world
        drone = Drone(client, world, "Drone1")

    # ------------------------------------------------------------------------------
    # Subscribe to chase camera sensor
    #     chase_cam_window = "ChaseCam"
    #     image_display.add_chase_cam(chase_cam_window)
    #     client.subscribe(
    #         drone.sensors["Chase"]["scene_camera"],
    #         lambda _, chase: image_display.receive(chase, chase_cam_window),
    #     )
    #
    #     # Subscribe to the drone's sensors with a callback to receive the sensor data
    #     rgb_name = "RGB-Image"
    #     image_display.add_image(rgb_name, subwin_idx=0)
    #     client.subscribe(
    #         drone.sensors["DownCamera"]["scene_camera"],
    #         lambda _, rgb: image_display.receive(rgb, rgb_name),
    #     )
    #
    #     depth_name = "Depth-Image"
    #     image_display.add_image(depth_name, subwin_idx=2)
    #     client.subscribe(
    #         drone.sensors["DownCamera"]["depth_camera"],
    #         lambda _, depth: image_display.receive(depth, depth_name),
    #     )
    #
    #     image_display.start()

        # ------------------------------------------------------------------------------
        # Currently control APIs like arm, takeoff, move, etc. are not supported.
        # Wait for a user input while Ardupilot is controlled externally
        input("Press any key to stop seeing the drone's camera images...")


        # ------------------------------------------------------------------------------

    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)

    finally:
        # Always disconnect from the simulation environment to allow next connection
        client.disconnect()

        # image_display.stop()


if __name__ == "__main__":
    asyncio.run(main())  # Runner for async main function


# import asyncio
# from dronekit import connect, VehicleMode
# from projectairsim import ProjectAirSimClient, Drone, World
#
#
# async def main():
#     # --- 1. SET UP THE SIMULATION WORLD (Project AirSim) ---
#     sim_client = ProjectAirSimClient(
#         address="127.0.0.1",
#         port_topics=8989,
#         port_services=8990
#     )
#
#     sim_client.connect()
#     world = World(sim_client, "/SUAS/simulation/sim_config/scene_ardu_quadrotor.jsonc")
#     # We use this primarily for cameras or resetting the environment
#     sim_drone = Drone(sim_client, world, "Drone1")
#
#     # --- 2. SET UP THE FLIGHT CONTROL (DroneKit) ---
#     # Connect to ArduPilot SITL.
#     # Replace '127.0.0.1:14551' with the IP of the machine running ArduPilot.
#     print("Connecting to ArduPilot via DroneKit...")
#     vehicle = connect("tcp:127.0.0.1:14550", wait_ready=True)
#     print("Connected to ArduPilot!")
#
#     try:
#         # --- 3. EXECUTE COMMANDS VIA DRONEKIT ---
#         print(f" Mode: {vehicle.mode.name}")
#         print(f" Armed: {vehicle.armed}")
#         print(f" GPS: {vehicle.location.global_frame}")
#
#         # Example: Simple Takeoff command using DroneKit
#         if vehicle.is_armable:
#             vehicle.mode = VehicleMode("GUIDED")
#             vehicle.armed = True
#             # vehicle.simple_takeoff(10) # Take off to 10m
#
#         # Keep the sim client alive to see cameras/physics
#         while True:
#             await asyncio.sleep(1)
#
#     finally:
#         vehicle.close()
#         sim_client.disconnect()
#
#
# if __name__ == "__main__":
#     asyncio.run(main())
