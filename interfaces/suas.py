def run_suas_code():
    project_root = "/SUAS"
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
        world = World(client, "/SUAS/simulation/sim_config/scene_suas.jsonc", delay_after_load_sec=2)
        drone = Drone(client, world, "SUAS_Drone")

        # Execute the flight logic
        run_suas_code()

    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)
    finally:
        client.disconnect()

if __name__ == "__main__":
    main()