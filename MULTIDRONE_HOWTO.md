# How to use Multiple Drones with Project Airsim

## Quick start: N drones through the IARC flight code

This is the current path, driven by [interfaces/iarc.py](./interfaces/iarc.py). It runs the
real state machine with interdrone comms, one process per drone. The standalone dronekit
example described further down predates it and is kept for reference.

`NUM_DRONES` must match on both sides -- `iarc.py` and `sim_start_drones.sh` derive their
port assignments from it independently, and nothing checks that they agree.

1. Start the Unreal / ProjectAirSim simulation on Windows.
2. In the `env` container, start the orchestrator:

   ```shell
   NUM_DRONES=2 MISSION_CONFIG=mission_config_2drone.json \
     SITL_HOST=<wsl-vm-ip> PAS_HOST=<windows-wsl-adapter-ip> \
     python iarc.py
   ```

   It loads the empty scene, then blocks waiting for the SITLs.
3. In a second terminal, start the SITLs:

   ```shell
   NUM_DRONES=2 AIRSIM_HOST=<windows-wsl-adapter-ip> ./run_container.sh sim
   ```

   This attaches to a tmux session with one window per drone. Wait until every window
   reports `Waiting for heartbeat from tcp:127.0.0.1:5760`.
4. Press Enter in the first terminal. It spawns the drones, waits for MAVLink on each
   drone's port, then launches `run.py --airsim -i <id>` per drone.

`SITL_HOST` is the WSL VM's IP (`hostname -I` inside WSL) and `PAS_HOST` / `AIRSIM_HOST`
are the Windows-side `vEthernet (WSL)` adapter IP (`ipconfig` on Windows). Both default to
loopback when everything shares one network namespace. Step 4's MAVLink wait dials
`SITL_MAVLINK_HOST`, which defaults to `SITL_HOST`; set it explicitly only if the SITL
container exposes its MAVLink TCP ports on a different address than the one it receives
AirSim's sensor UDP on.

### Port map

For drone index `i` (0-based; mission config IDs are 1-based, so `i = id - 1`):

| Port | Purpose | Set by |
| --- | --- | --- |
| `5760 + 10i` | SITL serial0, claimed by MAVProxy | `--instance` |
| `5762 + 10i` | SITL serial1, what dronekit connects to | `--instance`, read in `state_machine/drone.py` |
| `9003 + 10i` | AirSim -> SITL, sensor data | `--instance`, `ardupilot-udp-port` |
| `9002 + 10i` | SITL -> AirSim, servo output | `--instance`, `local-host-udp-port` |
| `5001 + i` | interdrone comms | `drone_info` in the mission config |

The SITL side of all four comes from `--instance` alone -- `arducopter --help` describes it
as adding `10*instance` to *all* port numbers. The scene config side is computed to match in
`ArduWorld._build_actors`. Note that `--sim-port-in`/`--sim-port-out` are options on the
`arducopter` binary, not on `sim_vehicle.py`; they are not needed here and setting them
would double up with the offset `--instance` already applies.

Three files have to agree on this arithmetic: `sim_start_drones.sh`, `interfaces/iarc.py`,
and `state_machine/drone.py`.

## Environment Setup

The environment setup process is unchanged from the normal sim environment setup: <https://missourimrr.github.io/docs/simulation/installation/windows/#environment-setup>

## What are the Docker containers for??

### `env` Container

The `env` container ([./Env.Containerfile](./Env.Containerfile)) essentially acts as a virtual environment in which to run flight code, such as the SUAS code itself or one of the test scrips in `/tests`. If you can run flight code locally on your machine, you do not need to use this container. It only exists to simplify the environment setup process.

To start the `env` container, run the following in a bash-compatible terminal (with `podman` and `podman-compose`), such as WSL:

```shell
./run_container.sh env
```

Running the container in this way automatically attaches to it.

### `sim` Container

The `sim` container ([./Sim.Containerfile](./Sim.Containerfile)) acts as a universal environment capable of running Ardupilot's `sim_vehicle.py` command ([click here for more info](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)), which starts a SITL for the drone(s). By default, this script is automatically called when this container is launched, meaning that starting this container is equivalent to starting the drone SITL.

```shell
./run_container.sh sim
```

Running the container in this way automatically attaches to it.

#### How does `sim` work?

Upon startup, `sim` runs the `sim_start_drones.sh` script, which automatically starts multiple drones (or just one by default). The number of drones is configured using the `NUM_DRONES` environment variable set when starting the container:

```shell
NUM_DRONES=10 ./run_container.sh sim
```

Each drone is started using the following command and run in individual `tmux` windows:

```sh
# i is just an index from a for loop
/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f airsim-copter -w --instance $i
```

The `--instance` argument is 0-based and automatically increments relevant ports by 10 per instance. In dronekit terms, this means the first drone's connection string is `tcp:127.0.0.1:5762`, the second's is `tcp:127.0.0.1:5772`, the third's is `tcp:127.0.0.1:5782`, and so on. In terms of Airsim settings, the first drone's control ports (for `ardupilot-udp-port` and `local-host-udp-port`, respectively) are `9003` and `9002`, the second drone's are `9013` and `9012`, the third's are `9023` and `9022`, and so on. It is highly recommended that these are automated, which is what our current example does.

## How the Multidrone Example Works

The example in question is [/tests/ProjectAirsimMultidrone.py](/tests/ProjectAirsimMultidrone.py). It provides a single- or multidrone-environment with minimal terminal-based controls. The relevant config files are `scene_ardu_empty.jsonc`, `scene_ardu_quadrotor_template.jsonc`, and `robot_ardu_quadrotor.jsonc`. The file expects to be run from the SUAS repositoy root.

The `main` function begins by connecting to the Unreal simulation, so the Unreal simulation should be started first. Then, an _empty_ scene is initialized with

```python
World(client, "scene_ardu_empty.jsonc", delay_after_load_sec=2, sim_config_path="./simulation/sim_config")
```

This is done because `sim_vehicle.py` (i.e., the `sim` container) expects a scene to be loaded before starting, since the SITL downloads some scene data when starting. If no scene is not initialized, the download will fail, potentially softlocking the SITL.

Next, the `main` procedure waits for user input. During this time, the `sim` container should be started, since the next steps require their existence to prevent the Unreal simulation from softlocking. Once the `sim` containter has fully started, the user may press enter to continue.

Now, our actual scene, including all drones, is finally initialized:

```python
world = MultidroneWorld(client, "scene_ardu_quadrotor.jsonc", delay_after_load_sec=2, sim_config_path="./simulation/sim_config", drone_grid=drone_grid)
```

A custom class, `MultidroneWorld`, is used to automatically generate a grid of drones based on `drone_grid`, a row-column pair/tuple (see [What is MultidroneWorld and why does it exist?](#what-is-multidroneworld-and-why-does-it-exist)). This will initialize all simulated drones in the Unreal simulation, which the SITLs running in the `sim` container will promptly connect to.

> If the drones are spawned into the Unreal simulation before the SITLs are started, they will softlock, having failed to connect to an SITL immediately. It does not retry this connection, and you must restart the entire simulation to fix this. This is why we prompt the user to continue; it gives them time to start the SITL before reaching this step.

> Thus, the SITL expects an initialized simulation scene to function and the simultaion scene (the drones, specifically) expects an SITL to be running. This is a bit of a Catch-22, but it's fixed by initializing that empty scene first, as the SITL does not need a scene with drones in it to start, only a scene.

Next, the user is prompted to continue again, which is meant to give time for the SITLs to connect to started drones before connection attempts are made. After some time, you may continue to the connection step.

Drones are controlled and connected to via `dronekit`. To make multidrone simulation easier, a custom `DronekitDrone` class exists to streamline interactions and commands. To improve performance (or attempt to), each drone is ran in its own subprocess.

As connections are made, drones should slowly begin taking off. Once all drones take off, you can start providing instructions: `n` for North, `e` for East, `s` for South, `w` for West, `u` for up, and `d` for down. Multiple instructions can be provided in one batch, such as `nnnnnnnnuuuuuuwwwww`. To land drones, submit `q`, `quit`, or `die`.

### Running the Example (in steps)

1. Start the Unreal simulation
2. Run the example code (make sure the `drone_grid` variable matches the number of drones you are starting)
3. Once the first continue prompt halts execution, run the `sim` container
   1. make sure to run it with the correct number of drones, such as `NUM_DRONES=4 ./run_container.sh sim`
   2. press continue (Enter) once the sim container fully starts
4. Once teh second continue prompt halts execution, wait for the drones to initialize
   1. there is no exact science to this, but the more drone you are running, the longer you'll have to wait (probably)
   2. press continue once you think the drones are initalize
5. wait for all drones to connect and take off, then start controlling the drones

## What is MultidroneWorld and why does it exist?

Unlike legacy AirSim, ProjectAirSim uses a multi-file configuration structure (see the [official config docs](https://github.com/iamaisim/ProjectAirSim/blob/main/docs/config.md)). This means that each drone would have to have an individual config file, as the control ports we need to change per drone are stored in the robot config files. To circumvent this, we have created the `MultidroneWorld` class, which is a subclass of ProjectAirSim's `World` class, that intercepts the config loading process to inject new robot config settings automatically rather than storing and loading many files.

`MultidroneWorld` only overrides the `__init__` function of `World`, and it is nearly identical to `World`'s config, but it modifies the loaded config file (stored as a Python dictionary) before it's used to reload the Unreal scene. It does this based on a new `drone_grid` argument, which is a `(num_rows, num_cols)` tuple. `MultdroneWorld` expects the loaded scene config file to contain one robot already, and it uses that robot's settings as a template for all generated drones. In other words, it deep-copies the drone already existing in the scene and increments its control ports accordingly (matching how the `--instance` argument of `sim_vehicle.py` increments its ports by ten). Additionally, each drone's `xyz` offsets are incremented such that spawned drones form a grid. Their names are of the form `Drone_{row_index}_{col_index}` (in case you want to access them using ProjectAirSim's Python package).

## Expanding Beyond the Example

Here are the main take aways on how to use multidrone with ProjectAirsim:

1. before anything, start the Unreal simulation
2. before starting the `sim` container (or the drone's SITLs), make sure an empty scene (i.e., a scene with no drones in it) is initialized
   1. this is done via Python using the ProjectAirSim `World` class (see the example)
   2. **Note:** this does not need to be done in the same script that your flight code is in---the Unreal simulation is running an API, and the Python code just connects to it; that is, closing your Python code will not undo any initializations you've already made, so it can be a separate script if you'd like
   3. once this is complete, start the SITLs
3. once the SITLs have started, initialize a scene with drones in it
   1. use the `MultidroneWorld` class to automatically add new drones in a grid/matrix shape, using the single drone already provided in the scene config as a base
4. wait for the simulated drones and the SITLs to connect/initialize, then connect to the drones using `dronekit`
5. fly the drones

It may be smart to have a "simulation init" Python script that initializes the scene properly, then run your flight code as usual after everything is ready.

## Further Areas of Development

- find a way to better automate these steps --- it'd be nice to be able to start a multi-drone simulation with a single command or something
- make `MultidroneWorld` more customizable??
  - I was thinking that we could create different callable classes that automatically modify the config dict differently, such as doing different shapes and such
  - for instance (for existing grid setup):
  
    ```python
    class DroneGridConfig:
        def __init__(self, num_rows, num_cols):
            self.num_rows = num_rows
            self.num_cols = num_cols
        
        def __call__(self, config: dict) -> dict:
            # generate new config dict
    
    func = DroneGridConfig(2, 2)
    # would be used like func(scene_config) in MultidroneWorld's init

    world = MultidroneWorld(..., config_func=func)
    ```
