import time
from dronekit import connect, VehicleMode, APIException
from projectairsim import ProjectAirSimClient, Drone, World
from projectairsim.utils import projectairsim_log

def stable_connect():
    vehicle = None
    while not vehicle:
        try:
            print("Attempting to reach ArduPilot...")
            # Use a longer timeout and wait_ready=False to prevent early exit
            vehicle = connect('127.0.0.1:14550', wait_ready=False, timeout=60)
        except (APIException, Exception) as e:
            print(f"SITL not ready yet ({e}). Retrying in 2s...")
            time.sleep(2)

    print("Link established. Waiting for parameters...")
    vehicle.wait_ready(True, timeout=60)
    return vehicle

def run_dronekit_logic():
    """Handles the ArduPilot flight commands via DroneKit"""
    vehicle = stable_connect()
    vehicle.parameters['ARMING_CHECK'] = 0

    # Now wait for the vehicle to be truly ready
    vehicle.wait_ready(True, timeout=60)
    print("Vehicle ready!")

    try:
        print("Basic pre-arm checks...")
        # Wait for vehicle to be armable
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialize...")
            time.sleep(1)

        print("Arming motors")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True

        print("Taking off!")
        target_altitude = 50
        vehicle.simple_takeoff(target_altitude)

        # Wait until the vehicle reaches a safe height
        while True:
            print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
            if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

        print("Hovering for 5 seconds...")
        time.sleep(5)

        print("Landing...")
        vehicle.mode = VehicleMode("RTL")

        # Wait for the drone to touch down
        while vehicle.armed:
            print(f" Landing... Altitude: {vehicle.location.global_relative_frame.alt}")
            time.sleep(1)
        
        print("Landed and Disarmed.")

    finally:
        print("Closing vehicle object")
        vehicle.close()

def main():
    # Initialize Project AirSim Client
    client = ProjectAirSimClient()
    
    try:
        print("Connecting...")
        client.connect()
        # Load the world and vehicle defined in your JSONC
        world = World(client, "/SUAS/simulation/sim_config/scene_ardu_quadrotor.jsonc", delay_after_load_sec=2)
        drone = Drone(client, world, "Drone1")

        # Execute the flight logic
        print("Running dronekit logic...")
        run_suas_code()

    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)
    finally:
        client.disconnect()

if __name__ == "__main__":
    main()