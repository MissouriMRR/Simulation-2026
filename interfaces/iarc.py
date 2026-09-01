import collections
import collections.abc
import copy
import os

# commentjson (a projectairsim dependency) still uses the pre-3.10 collections aliases
collections.MutableMapping = collections.abc.MutableMapping

import socket
import subprocess
import time
from pathlib import Path

from projectairsim import Drone, ProjectAirSimClient, World
from projectairsim.utils import load_scene_config_as_dict, projectairsim_log

# this file lives at <repo>/simulation/interfaces/iarc.py
PROJECT_ROOT = Path(__file__).resolve().parents[2]
SIM_CONFIG_PATH = str(PROJECT_ROOT / "simulation" / "sim_config")

EMPTY_SCENE = "/IARC/simulation/sim_config/scene_ardu_empty.jsonc"
IARC_SCENE = "/IARC/simulation/sim_config/scene_iarc.jsonc"

# How many drones to fly. Must match NUM_DRONES passed to the sim container, since both
# sides derive their port assignments from it independently.
NUM_DRONES = int(os.environ.get("NUM_DRONES", "1"))

# Mission config the flight code reads. It must contain a drone_info entry for every drone
# ID this script spawns (1..NUM_DRONES).
MISSION_CONFIG = os.environ.get("MISSION_CONFIG", "mission_config.json")

# Whether this script also runs the flight code. Set to 0 when the flight code lives
# somewhere else -- e.g. one Raspberry Pi per drone, each connecting back to the SITLs on
# this host. In that case this script only stands the scene up and holds it open.
SPAWN_FLIGHT_CODE = os.environ.get("SPAWN_FLIGHT_CODE", "1") not in ("0", "false", "False")

# Where the ArduPilot SITL is reachable from the machine running Unreal. When Unreal runs
# on Windows and the SITL runs in a WSL container, this is the WSL VM's IP (`hostname -I`
# inside WSL); both default to loopback for an all-on-one-host setup.
SITL_HOST = os.environ.get("SITL_HOST", "127.0.0.1")
# Address the Unreal side binds to receive actuator packets from the SITL. 0.0.0.0 accepts
# them regardless of which interface they arrive on.
AIRSIM_BIND_HOST = os.environ.get("AIRSIM_BIND_HOST", "0.0.0.0")

# Port arithmetic. MUST stay in sync with simulation/sim_start_drones.sh and with the port
# computed in state_machine/drone.py. For drone index i (0-based):
#   dronekit talks to SITL serial1 at 5762 + 10i; serial0 (5760 + 10i) is taken by MAVProxy
#   AirSim sends sensor data to  9003 + 10i
#   AirSim listens for servos on 9002 + 10i
SITL_MAVLINK_PORT = int(os.environ.get("SITL_MAVLINK_PORT", "5762"))
# Where *this* process reaches the SITLs' MAVLink TCP ports. Usually the same host AirSim
# sends sensors to, but kept separate because the two can differ when the SITL container
# publishes MAVLink on a different address than the one it receives UDP on.
SITL_MAVLINK_HOST = os.environ.get("SITL_MAVLINK_HOST", SITL_HOST)
PORT_STRIDE = 10
SITL_WAIT_SEC = 300

# Metres between adjacent drones in the scene, along the scene's x axis. Without this they
# spawn on top of each other and collide on takeoff.
DRONE_SEPARATION_M = float(os.environ.get("DRONE_SEPARATION_M", "3.0"))

# Log directory name shared by every drone in this run. All the flight-code processes are
# children of this one, so setting it here is what lets tools/analyze_flight.py put them on
# a single timeline -- without it each process invents its own id and the logs scatter.
# Logs land in /IARC/Logs, a bind mount of the repo, so they show up on the host as well.
FLIGHT_LOG_RUN = os.environ.get("FLIGHT_LOG_RUN") or time.strftime(
    "run_%Y%m%d_%H%M%S", time.gmtime()
)

# The chase camera is a 1280x720 stream per drone. One is useful for watching the run; N of
# them is a large GPU cost for no benefit, so it is kept only on the first drone.
CHASE_CAMERA_ID = "Chase"


def drone_name(drone_id: int) -> str:
    """Scene actor name for a drone ID. IDs are 1-based to match mission_config.json."""
    return f"Drone{drone_id}"


class ArduWorld(World):
    """World that rewrites each robot's ArduPilot endpoints before loading the scene.

    ProjectAirSim reads robot configs straight off disk, so the only place to override the
    SITL addresses without maintaining a per-machine copy of robot_ardu_quadrotor.jsonc is
    between the config being parsed and the scene being loaded. Same interception trick as
    MultidroneWorld in simulation/multidrone_world.py.

    For multi-drone runs this also clones the single robot declared in the scene config
    into `num_drones` actors, incrementing each one's UDP ports and spawn position so they
    line up with the SITL instances started by sim_start_drones.sh.
    """

    def __init__(
        self,
        client: ProjectAirSimClient,
        scene_config_name: str,
        delay_after_load_sec: int = 0,
        sim_config_path: str = "sim_config/",
        sim_instance_idx: int = -1,
        num_drones: int = 1,
    ):
        self.client = client
        self.sim_config_path = sim_config_path
        self.sim_instance_idx = sim_instance_idx
        self.parent_topic = "/Sim/SceneBasicDrone"  # default-scene's ID

        self.sim_config = None
        self.home_geo_point = None

        config_dict, config_paths = load_scene_config_as_dict(
            scene_config_name, sim_config_path, sim_instance_idx
        )

        config_dict["actors"] = self._build_actors(config_dict.get("actors", []), num_drones)

        self.scene_config_path = config_paths[0]
        self.robot_config_paths = config_paths[1]
        self.envactor_config_paths = config_paths[2]
        self.load_scene(config_dict, delay_after_load_sec=delay_after_load_sec)

    def _build_actors(self, actors: list, num_drones: int) -> list:
        """Expand the scene's single robot template into `num_drones` configured actors."""
        if not actors:
            return actors

        template = actors[0]
        start_x, start_y, z = map(float, template["origin"]["xyz"].split())

        built = []
        for index in range(num_drones):
            actor = copy.deepcopy(template)
            actor["name"] = drone_name(index + 1)
            actor["origin"]["xyz"] = " ".join(
                str(v) for v in (start_x + index * DRONE_SEPARATION_M, start_y, z)
            )

            settings = actor.get("robot-config", {}).get("controller", {}).get("ardupilot-settings")
            if settings is None:
                projectairsim_log().warning(
                    f"Actor '{actor['name']}' has no ardupilot-settings; leaving it as-is."
                )
                built.append(actor)
                continue

            # The template carries drone 0's ports, so offset from those rather than from a
            # hardcoded base -- that keeps robot_ardu_quadrotor.jsonc the single source of
            # truth for the starting port numbers.
            settings["ardupilot-udp-port"] += index * PORT_STRIDE
            settings["local-host-udp-port"] += index * PORT_STRIDE
            settings["ardupilot-ip"] = SITL_HOST
            settings["local-host-ip"] = AIRSIM_BIND_HOST

            if index > 0:
                self._disable_chase_camera(actor)

            projectairsim_log().info(
                f"Actor '{actor['name']}': sending sensors to SITL at "
                f"{settings['ardupilot-ip']}:{settings['ardupilot-udp-port']}, "
                f"listening for control on "
                f"{settings['local-host-ip']}:{settings['local-host-udp-port']}"
            )
            built.append(actor)

        return built

    @staticmethod
    def _disable_chase_camera(actor: dict) -> None:
        """Turn off the chase camera on an actor to keep the GPU cost of N drones sane."""
        for sensor in actor.get("robot-config", {}).get("sensors", []):
            if sensor.get("id") == CHASE_CAMERA_ID:
                sensor["enabled"] = False


def wait_for_sitl_launch() -> None:
    """Pause until the user confirms the ArduPilot SITL has been launched.

    This cannot be automated by probing a port. AirSim::recv_fdm() blocks in a retry loop
    until the first sensor packet arrives from AirSim, so a freshly launched SITL has not
    yet opened any of its MAVLink TCP ports -- it sits there printing "No sensor message
    received in last 1s, resending servos". Those ports only appear *after* a drone spawns
    and starts feeding it. So the only observable signal at this point is the sim
    container's own output.

    Set SITL_START_DELAY to a number of seconds to wait blindly instead of prompting.
    """
    delay = os.environ.get("SITL_START_DELAY")
    if delay is not None:
        projectairsim_log().info(f"Waiting {delay}s for the SITL to launch...")
        time.sleep(float(delay))
        return

    projectairsim_log().info(
        f"Empty scene loaded. Now start {NUM_DRONES} SITL(s) in another terminal:\n"
        f"    NUM_DRONES={NUM_DRONES} AIRSIM_HOST=<windows-wsl-adapter-ip> ./run_container.sh sim\n"
        "Wait until every tmux window reports 'Waiting for heartbeat from tcp:127.0.0.1:5760', "
        "then press Enter here to spawn the drones."
    )
    input("(press enter once the sim container is up) ")


def wait_for_mavlink(host: str = SITL_MAVLINK_HOST, port: int = SITL_MAVLINK_PORT) -> None:
    """Block until the SITL is actually emitting MAVLink.

    A successful TCP connect is not enough: the listening socket is open from the moment
    the SITL starts, but nothing is written to it while the physics loop is stuck in
    recv_fdm() waiting on AirSim. So read until a MAVLink frame header shows up (0xFE for
    v1, 0xFD for v2), which only happens once the drone is feeding the SITL sensor data.
    """
    projectairsim_log().info(f"Waiting for SITL MAVLink heartbeats on {host}:{port}...")
    deadline = time.monotonic() + SITL_WAIT_SEC
    while time.monotonic() < deadline:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(2.0)
            if sock.connect_ex((host, port)) == 0:
                try:
                    data = sock.recv(64)
                except (socket.timeout, OSError):
                    data = b""
                if any(b in data for b in (b"\xfd", b"\xfe")):
                    projectairsim_log().info(f"SITL on {host}:{port} is emitting MAVLink.")
                    return
        time.sleep(2)
    raise TimeoutError(
        f"no MAVLink from the SITL on {host}:{port} after {SITL_WAIT_SEC}s. The SITL is "
        "stuck in recv_fdm() waiting on AirSim sensor packets, so AirSim's UDP is not "
        f"reaching {SITL_HOST}, or {host} is not where its MAVLink ports are exposed. Check "
        "the WSL Hyper-V firewall inbound rule, SITL_HOST, SITL_MAVLINK_HOST, --sim-address, "
        "and that NUM_DRONES matches on both sides."
    )


def run_iarc_code(drone_ids: list[int]) -> None:
    """Run one flight-code process per drone and wait for them all to finish.

    Each process gets its own -i so it picks up the matching drone_info entry (and so
    Drone.use_settings computes the matching SITL port). They talk to each other over the
    interdrone loopback addresses declared in the mission config.
    """
    project_root = "/IARC"
    processes: list[tuple[int, subprocess.Popen]] = []

    # Inherited by every child, so all drones write into the same run directory.
    child_env = dict(os.environ, FLIGHT_LOG_RUN=FLIGHT_LOG_RUN)
    projectairsim_log().info(
        f"Flight logs for this run: /IARC/Logs/{FLIGHT_LOG_RUN} "
        f"(analyze with: python tools/analyze_flight.py Logs/{FLIGHT_LOG_RUN})"
    )

    try:
        for drone_id in drone_ids:
            command = ["uv", "run", "run.py", "--airsim", "-i", str(drone_id)]
            if MISSION_CONFIG:
                command += ["--config", MISSION_CONFIG]
            projectairsim_log().info(f"Launching flight code for drone {drone_id}: {command}")
            processes.append(
                (drone_id, subprocess.Popen(command, cwd=project_root, text=True, env=child_env))
            )

        for drone_id, process in processes:
            code = process.wait()
            if code == 0:
                print(f"Drone {drone_id} flight script executed successfully!")
            else:
                print(f"Drone {drone_id} flight script exited with code {code}")
    except FileNotFoundError:
        print("Error: 'uv' is not installed")
    finally:
        for _, process in processes:
            if process.poll() is None:
                process.terminate()


def wait_for_external_flight_code() -> None:
    """Hold the scene open while the flight code runs elsewhere.

    Disconnecting the client does not tear the Unreal scene down, but this process owns the
    Drone handles created in main(), so exiting here would drop the sensor streams the SITLs
    depend on. Block until interrupted instead.
    """
    projectairsim_log().info(
        "Scene is up and all SITLs are emitting MAVLink. Start the flight code on each "
        "drone's machine now. On the Pi for drone <id>, with <sim-host> the LAN address of "
        "the machine running Unreal:\n"
        "    SITL_MAVLINK_HOST=<sim-host> uv run run.py --airsim -i <id>\n"
        f"Drone <id> connects to <sim-host>:{SITL_MAVLINK_PORT} + {PORT_STRIDE}*(<id>-1), "
        f"i.e. {', '.join(str(SITL_MAVLINK_PORT + i * PORT_STRIDE) for i in range(NUM_DRONES))} "
        f"for drones 1..{NUM_DRONES}.\n"
        "Press Ctrl-C here once the run is over."
    )
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        projectairsim_log().info("Shutting the scene down.")


def main():
    # Initialize Project AirSim Client
    client = ProjectAirSimClient(address=os.environ.get("PAS_HOST", "127.0.0.1"))
    drone_ids = list(range(1, NUM_DRONES + 1))

    try:
        print(f"Connecting to projectAirSim for {NUM_DRONES} drone(s)...")
        client.connect()

        # 1. An empty scene, so the SITL has something to pull scene data from on startup.
        World(client, EMPTY_SCENE, delay_after_load_sec=2)

        # 2. The SITLs, which must be running before any drone spawns -- a drone that finds
        #    no SITL on spawn softlocks and cannot recover.
        wait_for_sitl_launch()

        # 3. The real scene. Spawning the drones starts the sensor streams that unblock the
        #    SITLs' physics loops, which in turn makes them open their MAVLink ports.
        world = ArduWorld(client, IARC_SCENE, delay_after_load_sec=2, num_drones=NUM_DRONES)
        for drone_id in drone_ids:
            Drone(client, world, drone_name(drone_id))

        # 4. Only now can anything speak MAVLink to the SITLs.
        for index in range(NUM_DRONES):
            wait_for_mavlink(port=SITL_MAVLINK_PORT + index * PORT_STRIDE)

        # 5. The mission. run.py opens its own dronekit connection to the SITL rather than
        #    reusing the ProjectAirSim handle above, which only carries sensor/telemetry
        #    topics.
        if SPAWN_FLIGHT_CODE:
            run_iarc_code(drone_ids)
        else:
            wait_for_external_flight_code()

    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)
    finally:
        client.disconnect()


if __name__ == "__main__":
    main()
