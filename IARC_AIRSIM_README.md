# Running IARC on Project AirSim

Cold-boot runbook for flying the IARC state machine against Project AirSim: Unreal on
Windows, ArduPilot SITLs and the flight code in podman containers under WSL2, and
(optionally) the Android app driving it.

Companion docs:

- [MULTIDRONE_HOWTO.md](./MULTIDRONE_HOWTO.md) — why the port arithmetic looks the way it
  does, what the two containers are, how `ArduWorld`/`MultidroneWorld` inject per-drone
  configs. Read this when something doesn't line up.
- [PI_SIM_RUNBOOK.md](./PI_SIM_RUNBOOK.md) — the variant where the flight code runs on four
  Raspberry Pis instead of inside the `env` container.

## Prerequisites

Install these once, following the team docs at
<https://missourimrr.github.io/docs/simulation/>:

- Unreal Engine **5.2.x** and the Project AirSim Unreal project (Windows)
- IARC-10 (with SIM submodule), Iarc2025App, and SIM Unreal repo cloned in. Note: Must be on MST wifi to clone SIM Unreal repo
- WSL2 with `podman` and `podman-compose`
- `socat` in WSL (`sudo apt install -y socat`) — only if you're running the app
- Android Studio / `adb` on Windows — only if you're running the app

Clone this repo inside WSL and make sure the `simulation/` submodule is checked out. (NOTE/TODO: IARC 10 does not officially have SIM sub module. We need to do this still)

## Architecture

```
Windows                          WSL2                             Android emulator
-------                          ----                             ----------------
Unreal + Project AirSim  <---->  env container                    the app
  (scene, physics, sensors)        interfaces/iarc.py
                                   run.py  x N  (dronekit)
                                 sim container
                                   arducopter SITL x N
```

Three separate connections have to come up, in this order:

1. `iarc.py` → Unreal, over the Project AirSim API at `PAS_HOST`.
2. Unreal → SITL, UDP sensor packets to `SITL_HOST:9003+10i`; SITL → Unreal, servo output
   to `AIRSIM_HOST:9002+10i`.
3. flight code → SITL, dronekit over TCP `5762+10i`.

`NUM_DRONES` must match between `iarc.py` and the `sim` container. Both derive their port
assignments from it independently and **nothing checks that they agree** — a mismatch shows
up as the 300 s `no MAVLink from the SITL` timeout.

## 0. Collect the two addresses (every boot)

Only `172.27.192.1` is stable. The WSL VM address is DHCP and changes on every
`wsl --shutdown` or reboot; a stale value is the most common cause of a failed run.

```powershell
wsl hostname -I
ipconfig
```

| Name                       | Where it comes from                  | Last seen               |
| -------------------------- | ------------------------------------ | ----------------------- |
| `SITL_HOST`                | first address from `wsl hostname -I` | `172.27.193.57`         |
| `PAS_HOST` / `AIRSIM_HOST` | `ipconfig` → `vEthernet (WSL)` IPv4  | `172.27.192.1` (stable) |

`AIRSIM_HOST` is already pinned in `simulation/.env`, which podman-compose auto-loads, so
you usually only have to supply `SITL_HOST` by hand.

If you have `networkingMode=mirrored` set in `.wslconfig`, WSL and Windows share
`127.0.0.1` and you can drop both variables entirely.

## 1. Unreal

1. Launch the Unreal simulation project (Unreal Engine 5.2.x).
2. Open the **IARC level** — Ctrl+Space opens the content drawer to find it.
3. Click **Play**.

This has to be up first: `iarc.py` connects to `PAS_HOST` as its very first action.

## 2. Terminal 1 — the `env` container and the orchestrator

In a WSL terminal:

```bash
cd ~/IARC-10/simulation && ./run_container.sh shutdown
```

Worth running first in case a previous run left containers up.

```bash
./run_container.sh env
```

That builds if needed and drops you into a shell inside the container, with the repo bind
mounted at `/IARC`. Inside it:

```bash
cd /IARC/simulation/interfaces && NUM_DRONES=4 SITL_HOST=172.27.193.57 PAS_HOST=172.27.192.1 python iarc.py
```

Wait for `Empty scene loaded.` and the `(press enter once the sim container is up)` prompt.

**Do not press Enter yet.** `iarc.py` loads an empty scene first on purpose: the SITL needs
_a_ scene to pull data from at startup, but a drone that spawns before its SITL exists
softlocks and only a full sim restart recovers it. The prompt is the gap between those two
requirements.

Useful variables:

| Variable             | Default               | What it does                                                                                                    |
| -------------------- | --------------------- | --------------------------------------------------------------------------------------------------------------- |
| `NUM_DRONES`         | `1`                   | Drones to spawn. Must match the `sim` container.                                                                |
| `SITL_HOST`          | `127.0.0.1`           | Where Unreal sends sensor UDP — the WSL VM IP.                                                                  |
| `PAS_HOST`           | `127.0.0.1`           | Where the Project AirSim API lives — the `vEthernet (WSL)` IP.                                                  |
| `MISSION_CONFIG`     | `mission_config.json` | Passed to `run.py --config`.                                                                                    |
| `SPAWN_FLIGHT_CODE`  | `1`                   | Set to `0` to hold the scene open and run flight code elsewhere (see [PI_SIM_RUNBOOK.md](./PI_SIM_RUNBOOK.md)). |
| `SITL_MAVLINK_HOST`  | `$SITL_HOST`          | Only set this if the SITL exposes MAVLink TCP on a different address than it receives UDP on.                   |
| `DRONE_SEPARATION_M` | `3.0`                 | Spacing between spawn points; without it they collide on takeoff.                                               |
| `SITL_START_DELAY`   | unset                 | Seconds to wait blindly instead of prompting.                                                                   |

## 3. Terminal 2 — the SITLs

A second WSL terminal, from `simulation/` (that's where `compose.yml` and `.env` live):

```bash
cd ~/IARC-10/simulation && NUM_DRONES=4 AIRSIM_HOST=172.27.192.1 ./run_container.sh sim
```

This attaches to a tmux session with one window per drone. Cycle through **all** of them
(`Ctrl-b n`) and wait until each reports:

```
Waiting for heartbeat from tcp:127.0.0.1:5760
```

The first run of this container builds ArduCopter from source, so give it a few minutes.

## 4. Back to terminal 1 — spawn and fly

Press Enter. `iarc.py` then:

1. Loads `scene_iarc.jsonc` with `NUM_DRONES` actors, rewriting each one's ArduPilot
   endpoints to match the SITL instance it belongs to.
2. Creates a `Drone` handle per actor, which starts the sensor streams — this is what
   unblocks each SITL's physics loop and makes it finally open its MAVLink port.
3. Waits for real MAVLink frames on `5762 + 10i` for every drone.
4. Launches `uv run run.py --airsim -i <id>` per drone, all sharing one `FLIGHT_LOG_RUN`
   so `tools/analyze_flight.py Logs/<run>` can put them on a single timeline.

### Port map

For drone index `i` (0-based; mission config IDs are 1-based, so `i = id - 1`):

| Port         | Purpose                                 |
| ------------ | --------------------------------------- |
| `5760 + 10i` | SITL serial0, claimed by MAVProxy       |
| `5762 + 10i` | SITL serial1, what dronekit connects to |
| `9003 + 10i` | AirSim → SITL, sensor data              |
| `9002 + 10i` | SITL → AirSim, servo output             |
| `5001 + i`   | interdrone comms                        |

## 5. Optional — the Android app

Emulator only. The chain is fiddly because the emulator, Windows, and WSL each have their
own idea of `127.0.0.1`, and WSL's localhost forwarding mirrors container ports onto
Windows `127.0.0.1` **only** — never onto the Wi-Fi LAN IP.

In the mission config, set `"app_opperable": true`.

### App → drone

In Windows PowerShell:

```bash
adb reverse tcp:5001 tcp:5001
```

Then in the app set **Drone 1 IP** to `127.0.0.1`.

Alternatively, skip `adb` entirely and set **Drone 1 IP** to `10.0.2.2` — the emulator's
fixed NAT alias for the Windows host loopback, which is exactly where WSL mirrors port 5001. Either works; don't do both and then wonder which one is live.

> `10.0.2.2`, not `10.0.0.2`. The digits matter — `10.0.0.2` is an arbitrary private
> address with nothing on it, and it is the mistake that cost us an afternoon.

### Drone → app

Leave the app's **App IP** field **blank**. That field is a bind address, not an announce
address, so anything other than an address on the emulated device (`10.0.2.16`) makes the
listen server fail to bind — it will read `Listen server: not bound`.

Turn the **emulator loopback toggle ON**, then build the relay. Admin PowerShell, once per
boot:

```bash
netsh interface portproxy add v4tov4 listenaddress=172.27.192.1 listenport=5100 connectaddress=127.0.0.1 connectport=5100
```

WSL, outside the container:

```bash
socat TCP-LISTEN:5100,bind=127.0.0.1,fork,reuseaddr TCP:172.27.192.1:5100
```

Full chain: drone dials WSL `127.0.0.1:5100` → socat → `172.27.192.1:5100` → portproxy →
Windows `127.0.0.1:5100` → adb → app. The `env` container is `network_mode: host`, so it
shares WSL's loopback and the socat listener is visible to it with no extra plumbing.

Get the forward path (app → drone) working on its own before adding any of this.

## 6. Shutdown

Ctrl-C the flight code, then:

```bash
cd ~/IARC-10/simulation && ./run_container.sh shutdown
```

Stop Play in Unreal. Kill the `socat` process if you started one.

## Troubleshooting

| Symptom                                                                                            | Cause                                                                                                                                             |
| -------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `no MAVLink from the SITL ... after 300s`                                                          | Stale `SITL_HOST` after a WSL restart; or `NUM_DRONES` differs between the two terminals; or the WSL Hyper-V firewall is dropping the sensor UDP. |
| Connecting to Unreal hangs or refuses                                                              | Unreal isn't in Play mode, or `PAS_HOST` is wrong.                                                                                                |
| Drones spawn but never move; SITL prints `No sensor message received in last 1s, resending servos` | The SITL isn't getting sensor packets — check `AIRSIM_HOST` and that Unreal is bound where the SITL is sending.                                   |
| Drone softlocked right after spawn                                                                 | Enter was pressed before the SITLs were up. Restart the whole sim; there is no partial recovery.                                                  |
| `PreArm: Need Position Estimate` for a long time                                                   | Normal. The EKF needs GPS lock; it clears on its own.                                                                                             |
| App: `Connection to 10.0.0.2:5001 failed`                                                          | Typo — it is `10.0.2.2`.                                                                                                                          |
| App: `Listen server: not bound`                                                                    | The App IP field has a non-device address in it. Clear it.                                                                                        |
| `env` container rebuilds its venv every run (~2.5 min)                                             | `UV_PROJECT_ENVIRONMENT` isn't taking effect; check the `iarc_uv` named volume still exists.                                                      |
| Other issue not listed                                                                             | Ask another member for help or Claude. Then document the issue here!                                                                              |

## Known rough edges

- The two `NUM_DRONES` values are unchecked and silently disagree.
- Three files have to agree on the port arithmetic: `sim_start_drones.sh`,
  `interfaces/iarc.py`, and `state_machine/drone.py`.
- `SITL_HOST`, the portproxy rule, and the app's addresses all need re-editing after a WSL
  restart. `networkingMode=mirrored` in `.wslconfig` collapses most of that.
