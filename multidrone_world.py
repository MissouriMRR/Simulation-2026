import copy
import random

from projectairsim import ProjectAirSimClient
from projectairsim.utils import load_scene_config_as_dict
from projectairsim.world import World


class MultidroneWorld(World):

    # ProjectAirsim has reading config from a file deeply integrated into it
    # You can't even trick it using a buffer or something.
    # Thus, the easiest strat (without creating a million temp files) is
    # to essentially intercept the initialization process after the config is
    # read to a dict and inject our generated settings there.
    # Most of the below code is the same as the normal World class __init__
    def __init__(
        self,
        client: ProjectAirSimClient,
        scene_config_name: str = "",
        delay_after_load_sec: int = 0,
        sim_config_path: str = "sim_config/",
        sim_instance_idx: int = -1,
        drone_grid: tuple[int, int] | None = None,
        x_sep: float = 3.0,
        y_sep: float = 3.0,
    ):
        """ProjectAirSim World Interface.

        Args:
            client (ProjectAirSimClient): ProjectAirSim client object
            scene_config (str): Name of the scene config JSON file to load in the sim
            delay_after_load_sec (int): Time in seconds to wait after the scene is loaded
            sim_config_path (string): Relative path to search for the scene_config
            sim_instance_idx (int): the instance index of the simulation (for distributed sim only)
        """
        self.client = client
        self.sim_config_path = sim_config_path
        self.sim_instance_idx = sim_instance_idx
        self.parent_topic = "/Sim/SceneBasicDrone"  # default-scene's ID

        self.sim_config = None
        self.home_geo_point = None
        if scene_config_name:
            config_loaded, config_paths = load_scene_config_as_dict(
                scene_config_name,
                sim_config_path,
                sim_instance_idx,
            )
            config_dict = config_loaded

            if drone_grid is not None:
                row, col = drone_grid
                template = config_dict["actors"][0]
                template["name"] = "Drone_0_0"
                start_x, start_y, z = map(float, template["origin"]["xyz"].split())

                for r in range(row):
                    for c in range(col):
                        # don't remake existing drone (i.e., the template)
                        if r == 0 and c == 0:
                            continue 

                        new_drone = copy.deepcopy(template)
                        new_drone["name"] = f"Drone_{r}_{c}"

                        new_drone["origin"]["xyz"] = " ".join(map(str, [start_x + c * x_sep, start_y + r * y_sep, z]))

                        drone_num = col * r + c
                        ardu_settings = new_drone["robot-config"]["controller"]["ardupilot-settings"]
                        ardu_settings["ardupilot-udp-port"] += 10 * drone_num 
                        ardu_settings["local-host-udp-port"] += 10 * drone_num 

                        config_dict["actors"].append(new_drone)

            self.scene_config_path = config_paths[0]
            self.robot_config_paths = config_paths[1]
            self.envactor_config_paths = config_paths[2]
            self.load_scene(config_dict, delay_after_load_sec=delay_after_load_sec)
        random.seed()
        self.import_ned_trajectory(
            "null_trajectory", [0, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]
        )