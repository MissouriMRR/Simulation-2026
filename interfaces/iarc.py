def run_iarc_code():
    project_root = "/IARC"
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
        world = World(client, "/IARC/simulation/sim_config/scene_iarc.jsonc", delay_after_load_sec=2)
        drone = Drone(client, world, "SUAS_Drone")

        # Execute the flight logic
        run_iarc_code()

    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)
    finally:
        client.disconnect()

if __name__ == "__main__":
    main()