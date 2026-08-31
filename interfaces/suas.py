import asyncio

import collections
import collections.abc

# Add the missing attribute back to the collections module
collections.MutableMapping = collections.abc.MutableMapping

from projectairsim import ProjectAirSimClient, Drone, World
from projectairsim.utils import projectairsim_log
from projectairsim.image_utils import ImageDisplay

import asyncio
import multiprocessing as mp
import time
import sys

from dronekit import LocationGlobalRelative, VehicleMode, connect

from ..multidrone_world import MultidroneWorld

class DronekitDrone:

    def __init__(self, connection_string):
        self._connection_string = connection_string
        self._drone = None

    def connect(self, timeout=30):
        print("connecting", self._connection_string)
        vehicle = connect(self._connection_string, wait_ready=True, timeout=timeout)

        # Get some vehicle attributes (state)
        print("Get some vehicle attribute values:")
        print(" GPS: %s" % vehicle.gps_0)
        print(" Battery: %s" % vehicle.battery)
        print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
        print(" Is Armable?: %s" % vehicle.is_armable)
        print(" System status: %s" % vehicle.system_status.state)
        print(" Mode: %s" % vehicle.mode.name)

        while not vehicle.is_armable:
            print("Waiting for vehicle to initialize...")
            time.sleep(1)

        vehicle.parameters["ARMING_CHECK"] = 0
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True

        self._drone = vehicle

    def takeoff(self, alt):
        self._drone.simple_takeoff(alt)
        self._takeoff_alt = alt

    def translate(self, dlat, dlon, dalt):
        loc = self._drone.location.global_relative_frame
        lat, lon, alt = loc.lat, loc.lon, loc.alt

        self._drone.simple_goto(
            LocationGlobalRelative(lat + dlat, lon + dlon, alt + dalt)
        )

    def goto(self, lat, lon, alt):
        self._drone.simple_goto(LocationGlobalRelative(lat, lon, alt))

    @property
    def took_off(self):
        return self._drone.location.global_relative_frame.alt >= self._takeoff_alt * 0.9

    @property
    def loc(self):
        return self._drone.location.global_relative_frame

    def land(self):
        self._drone.mode = VehicleMode("LAND")

    def close(self):
        self._drone.close()

def run_drone(connection_string, queue, timeout=30):
    drone = DronekitDrone(connection_string)
    drone.connect(timeout)
    time.sleep(3)

    while True:
        cmd = queue.get()

        if cmd is None:
            drone.land()
            drone.close()
            break
        elif cmd == "takeoff":
            drone.takeoff(20)
            while not drone.took_off:
                print("Waiting for drone to finish takeoff...")
                time.sleep(1)
        else:
            drone.translate(*cmd)

def run_suas_code():
    project_root = "/workspace"
    command = ["uv", "run", "run.py", "--airsim"]
    try:
        process = subprocess.run(
            command,
            cwd=project_root,
            check=True,
            text=True,
            capture_output=False
        )
        print("Flight script executed successfully!")
    except subprocess.CalledProcessError as err:
        print(f"The simulation failed with exit code: {err}")
    except FileNotFoundError:
        print("Error: 'uv' is not installed")


def main():
    # Initialize Project AirSim Client
    client = ProjectAirSimClient()

    try:
        print("Connecting to projectAirSim...")
        client.connect()
        # Load the world and vehicle defined in your JSONC
        world = World(client, "scene_ardu_empty.jsonc", delay_after_load_sec=2, sim_config_path="./simulation/sim_config")

        input("Start your sim container now. Press enter to continue (add drones to scene)")

        # SET DRONE GRID HERE
        drone_grid = (4, 4)
        processes = []

        world = MultidroneWorld(client, "scene_ardu_quadrotor_template.jsonc", delay_after_load_sec=2,
                                sim_config_path="./simulation/sim_config", drone_grid=drone_grid)

        input("Press enter to start connections (may need to wait a while for drones to get ready)")
        # Create a World object to interact with the sim world and load a scene
        base_port = 5762
        drone_count = drone_grid[0] * drone_grid[1]
        queues = [mp.Queue() for _ in range(drone_count)]

        # start drone processes, assign connection string
        for port, queue in zip(range(base_port, base_port + 10 * drone_count, 10), queues):
            proc = mp.Process(target=run_drone, args=(f"tcp:127.0.0.1:{port}", queue, 120))
            proc.start()

            processes.append(proc)

        for queue in queues:
            queue.put("takeoff")

        # Execute the flight logic
        run_suas_code()

    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)
    finally:
        client.disconnect()

        for p in processes:
            p.join()

if __name__ == "__main__":
    main()