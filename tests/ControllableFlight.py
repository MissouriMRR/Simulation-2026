import asyncio
import time
import socket  # For sending python commands directly to UE4

from dronekit import LocationGlobalRelative, VehicleMode, connect
from flight.camera import CameraAirSim

connection_string = "tcp:127.0.0.1:5762"

print("connect", connection_string)
vehicle = connect(connection_string, wait_ready=True)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("127.0.0.1", 9900))
print("TCP Server Connected @ 127.0.0.1:9900!")

cam = CameraAirSim()

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


async def takepic(cam):
    await cam.capture_photo()


async def main():
    while not vehicle.armed:
        print(" Waiting for v1 arming...")
        time.sleep(1)

    takeoff_alt = 20

    vehicle.simple_takeoff(takeoff_alt)

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print("Waiting for drone to finish takeoff...")
        if vehicle.location.global_relative_frame.alt >= takeoff_alt * 0.95:
            break
        time.sleep(1)

    loc = vehicle.location.global_relative_frame
    lat, lon, alt = loc.lat, loc.lon, loc.alt

    # control drone
    # type n to move north, s south, e east, w west, u up, and d down
    # you can put multiple at once, such as nnneeeuuu to do multiple commands at once
    # type q to quit/land
    while True:
        print("Loc:", lat, lon, alt)
        command: str = input("Give instructions: ").lower()

        if command.lower() in ("die", "q", "quit"):
            vehicle.mode = VehicleMode("LAND")
            break

        for cmd in command:
            match cmd:
                case "n":
                    lat += 0.0001
                case "s":
                    lat -= 0.0001
                case "e":
                    lon += 0.0001
                case "w":
                    lon -= 0.0001
                case "u":
                    alt += 5
                case "d":
                    alt -= 5
                case "p":
                    print("Attempting to capture photo")
                    await takepic(cam)
                    print("Photo captured")
                case "m":
                    print("Attempting release!")
                    send_rpc_msg("RELEASE")

        vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt))

    # Close vehicle object before exiting script
    vehicle.close()
    sock.close() # Release the socket server


# --- RPC Server Send ---
def send_rpc_msg(cmd):
    sock.send(cmd.encode('utf-8'))

if __name__ == "__main__":
    asyncio.run(main())
