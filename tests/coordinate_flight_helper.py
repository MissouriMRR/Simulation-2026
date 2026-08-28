"""Flight-side half of the coordinate accuracy test. Run by drone_coordinates_accuracy.py.

The env container installs projectairsim into the system Python and the flight dependencies
into the uv project venv, and no single interpreter has both. Rather than asking anyone to
patch their container, the test is split the way the rest of the repo already splits it:
interfaces/iarc.py runs on the system Python and spawns `uv run run.py` for the flight code.
This module is that child -- it talks to the SITL over DroneKit and answers questions from
its parent, and deliberately imports nothing from projectairsim.

Protocol: one JSON object per line on stdin, one reply per command on stdout. Replies carry
a `@@` prefix so DroneKit's own chatter on the same stream cannot be mistaken for one.

    {"cmd": "takeoff", "alt_m": 3.048}   -> @@{"ok": true}
    {"cmd": "yaw", "heading_deg": 45}    -> @@{"ok": true, "heading": 45.0}
    {"cmd": "pose"}                      -> @@{"ok": true, "pose": {"lat":..., "yaw_deg":...}}
    {"cmd": "land"}                      -> @@{"ok": true}
    {"cmd": "quit"}                      -> @@{"ok": true}

Every reply is either `{"ok": true, ...}` or `{"ok": false, "error": "..."}`; the parent is
never left waiting on a command that failed.

Run standalone for a smoke test:
    echo '{"cmd":"pose"}' | uv run python simulation/tests/coordinate_flight_helper.py
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import dronekit
from pymavlink import mavutil

REPLY_PREFIX = "@@"

SETTLE_TIMEOUT_SEC = 45.0
SETTLE_SPEED_M_S = 0.25
SETTLE_ALT_BAND_M = 0.35

YAW_RATE_DEG_S = 25.0
YAW_TIMEOUT_SEC = 30.0
YAW_TOLERANCE_DEG = 5.0


def log(message: str) -> None:
    """Progress goes to stderr, which the parent forwards but never parses."""
    print(f"[flight] {message}", file=sys.stderr, flush=True)


def reply(**payload) -> None:
    print(REPLY_PREFIX + json.dumps(payload), flush=True)


class FlightSide:
    """Owns the DroneKit connection and executes one command at a time."""

    def __init__(self, address: str, timeout: float):
        log(f"connecting to {address}")
        self.vehicle = dronekit.connect(address, wait_ready=True, timeout=timeout)
        # AirSim's simulated sensors do not always satisfy the full preflight suite, and the
        # test is about camera geometry rather than arming logic.
        self.vehicle.parameters["ARMING_CHECK"] = 0
        log("connected")

    def close(self) -> None:
        self.vehicle.close()

    def takeoff(self, alt_m: float) -> dict:
        vehicle = self.vehicle
        while not vehicle.is_armable:
            log("waiting for the vehicle to initialise...")
            time.sleep(1)

        vehicle.mode = dronekit.VehicleMode("GUIDED")
        vehicle.armed = True
        while not vehicle.armed or vehicle.mode.name != "GUIDED":
            log("waiting for arming...")
            time.sleep(1)

        log(f"taking off to {alt_m:.2f} m")
        # No overshoot margin: the hover altitude is the measurement's baseline, and a drone
        # descending out of a margin would still be moving when the markers are photographed.
        vehicle.simple_takeoff(alt_m)
        deadline = time.monotonic() + SETTLE_TIMEOUT_SEC * 2
        while time.monotonic() < deadline:
            if (vehicle.location.global_relative_frame.alt or 0.0) >= alt_m * 0.95:
                break
            time.sleep(0.5)

        settled = self.settle(alt_m)
        return {"alt_m": vehicle.location.global_relative_frame.alt, "settled": settled}

    def settle(self, target_alt_m: float) -> bool:
        """Wait for the hover to stop moving, so pose and photograph describe one instant."""
        deadline = time.monotonic() + SETTLE_TIMEOUT_SEC
        while time.monotonic() < deadline:
            altitude = self.vehicle.location.global_relative_frame.alt or 0.0
            velocity = self.vehicle.velocity or [0.0, 0.0, 0.0]
            speed = max(abs(component or 0.0) for component in velocity)
            if abs(altitude - target_alt_m) < SETTLE_ALT_BAND_M and speed < SETTLE_SPEED_M_S:
                log(f"settled at {altitude:.2f} m (max axis speed {speed:.2f} m/s)")
                return True
            time.sleep(0.5)
        log(f"did not settle within {SETTLE_TIMEOUT_SEC:.0f}s; continuing")
        return False

    def yaw(self, heading_deg: float) -> dict:
        """Yaw to an absolute compass heading and wait for it to arrive.

        The parent asks for an off-axis heading on purpose: at heading 0 an inverted yaw
        convention produces the same answers as a correct one, so the run would prove nothing
        about how yaw is applied.
        """
        heading = heading_deg % 360.0
        message = self.vehicle.message_factory.command_long_encode(
            0,
            0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading,  # target angle, degrees
            YAW_RATE_DEG_S,  # angular speed
            1,  # direction: 1 = clockwise (ignored for absolute angles)
            0,  # 0 = absolute heading rather than relative offset
            0,
            0,
            0,
        )
        self.vehicle.send_mavlink(message)

        deadline = time.monotonic() + YAW_TIMEOUT_SEC
        reached = False
        while time.monotonic() < deadline:
            current = self.vehicle.heading
            if current is not None:
                # Shortest angular distance, so 359 -> 1 is 2 degrees rather than 358.
                if abs((current - heading + 180.0) % 360.0 - 180.0) < YAW_TOLERANCE_DEG:
                    reached = True
                    break
            time.sleep(0.5)

        if reached:
            log(f"heading settled at {self.vehicle.heading} deg")
        else:
            log(f"heading did not reach {heading:.0f} deg; now {self.vehicle.heading}")
        return {"heading": self.vehicle.heading, "reached": reached}

    def pose(self) -> dict:
        """The pose the real flight code would be working from, in degrees."""
        location = self.vehicle.location.global_relative_frame
        attitude = self.vehicle.attitude
        return {
            "lat": float(location.lat),
            "lon": float(location.lon),
            # Relative to the arming position, which is the ground plane the parent measured.
            "alt_m": float(location.alt),
            # DroneKit reports attitude in radians; the conversion under test wants degrees.
            "yaw_deg": math.degrees(attitude.yaw),
            "pitch_deg": math.degrees(attitude.pitch),
            "roll_deg": math.degrees(attitude.roll),
            "heading": self.vehicle.heading,
        }

    def land(self) -> dict:
        self.vehicle.mode = dronekit.VehicleMode("LAND")
        return {}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--address", default="tcp:127.0.0.1:5762")
    parser.add_argument("--connect-timeout", type=float, default=120.0)
    args = parser.parse_args()

    try:
        flight = FlightSide(args.address, args.connect_timeout)
    except Exception as err:  # noqa: BLE001 -- must reach the parent as a reply, not a crash
        reply(ok=False, error=f"could not connect to {args.address}: {err}")
        return 1

    reply(ok=True, event="ready")

    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                request = json.loads(line)
                command = request["cmd"]
            except (ValueError, KeyError) as err:
                reply(ok=False, error=f"malformed request {line!r}: {err}")
                continue

            try:
                if command == "takeoff":
                    reply(ok=True, **flight.takeoff(float(request["alt_m"])))
                elif command == "yaw":
                    reply(ok=True, **flight.yaw(float(request["heading_deg"])))
                elif command == "pose":
                    reply(ok=True, pose=flight.pose())
                elif command == "land":
                    reply(ok=True, **flight.land())
                elif command == "quit":
                    reply(ok=True)
                    break
                else:
                    reply(ok=False, error=f"unknown command {command!r}")
            except Exception as err:  # noqa: BLE001 -- one bad command must not kill the run
                reply(ok=False, error=f"{command} failed: {err}")
    finally:
        flight.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
