# Pi + ProjectAirSim demo runbook

Flight code on four Raspberry Pis, SITLs and ProjectAirSim on the Windows/WSL host. This is
the command sequence from a cold boot. For what the pieces are and why the port arithmetic
looks like it does, see [MULTIDRONE_HOWTO.md](./MULTIDRONE_HOWTO.md).

Architecture: `simulation/interfaces/iarc.py` stands the scene up and holds it open
(`SPAWN_FLIGHT_CODE=0`); each Pi runs `run.py --airsim -i <id>` and opens dronekit over TCP
back to the host at `5762 + 10*(id-1)`. Pi Zero 2 Ws cannot run SITL themselves. Interdrone
comms ride the batman mesh on `bat0` and never touch the host.

## 0. Windows -- collect the three addresses

Nothing here is stable across a reboot except `172.27.192.1`. Do this every time.

```shell
wsl hostname -I
ipconfig
```

| Name | Where it comes from | Last seen |
| --- | --- | --- |
| `WSL_IP` | first address from `wsl hostname -I` | `172.27.193.57` |
| `PAS_HOST` / `AIRSIM_HOST` | `ipconfig` -> `vEthernet (WSL)` IPv4 | `172.27.192.1` (stable) |
| `LAN_IP` | `ipconfig` -> Wi-Fi adapter IPv4; what the Pis dial | `10.106.89.115` |

`WSL_IP` is DHCP and changes on every `wsl --shutdown` or reboot. A stale value is the single
most common cause of the 300 s `no MAVLink from the SITL` timeout in step 5.

Then **launch Unreal / ProjectAirSim**. `iarc.py` connects to `PAS_HOST` as its first action,
so this has to be up before anything else.

## 1. Admin PowerShell -- only when `WSL_IP` changed

WSL2 is NAT'd, so LAN clients cannot reach the SITLs without portproxy entries (or
`networkingMode=mirrored` in `.wslconfig`). The entries persist in the registry across
reboots, but they point at the old `WSL_IP` -- hence delete-then-add:

```powershell
$wsl = ((wsl hostname -I) -split '\s+')[0]
foreach ($p in 5762,5772,5782,5792) {
  netsh interface portproxy delete v4tov4 listenaddress=0.0.0.0 listenport=$p | Out-Null
  netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=$p connectaddress=$wsl connectport=$p
}
net stop iphlpsvc; net start iphlpsvc
netsh interface portproxy show v4tov4
```

The `iphlpsvc` bounce goes **after** the add, not before -- it is what makes a freshly added
entry take effect.

One-time, persists across reboots. Both are guarded, so re-running is safe:

```powershell
if (-not (Get-NetFirewallHyperVRule -Name "WSL-mavlink" -ErrorAction SilentlyContinue)) {
  New-NetFirewallHyperVRule -Name "WSL-mavlink" -DisplayName "WSL MAVLink 5760-5800" `
    -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' `
    -Protocol TCP -LocalPorts 5760-5800 -Action Allow
}
if (-not (Get-NetFirewallRule -Name "LAN-mavlink-in" -ErrorAction SilentlyContinue)) {
  New-NetFirewallRule -Name "LAN-mavlink-in" -DisplayName "MAVLink from Pis 5760-5800" `
    -Direction Inbound -Protocol TCP -LocalPort 5760-5800 -Action Allow -Profile Private,Public
}
```

The Hyper-V rule covers Windows -> WSL VM. The ordinary rule covers Pi -> Windows on the
Wi-Fi adapter, where the portproxy listener lives.

## 2. WSL terminal 1 -- orchestrator

```shell
cd ~/IARC-10/simulation && ./run_container.sh env
```

Then inside the `env` container:

```shell
cd /IARC/simulation/interfaces
NUM_DRONES=4 SPAWN_FLIGHT_CODE=0 SITL_HOST=<WSL_IP> PAS_HOST=172.27.192.1 python iarc.py
```

Wait for `Empty scene loaded.` and the `(press enter once the sim container is up)` prompt.
**Do not press Enter yet** -- a drone that spawns before its SITL exists softlocks and only a
full sim restart recovers it.

`MISSION_CONFIG` is irrelevant here: with `SPAWN_FLIGHT_CODE=0` the mission config that
matters is each Pi's own copy.

## 3. WSL terminal 2 -- SITLs

```shell
cd ~/IARC-10/simulation && NUM_DRONES=4 AIRSIM_HOST=172.27.192.1 ./run_container.sh sim
```

Must be run from `simulation/` -- that is where `compose.yml` and `.env` live. `.env` already
pins `AIRSIM_HOST=172.27.192.1`, so the inline value is redundant but kept as documentation.
`NUM_DRONES` must match terminal 1; nothing checks that it does.

This attaches to a tmux session with one window per drone. Cycle through all four
(`Ctrl-b n`) and wait until **each** reports
`Waiting for heartbeat from tcp:127.0.0.1:5760`.

## 4. WSL terminal 3 -- verify the listeners

```shell
ss -ltn | grep -E ':(5762|5772|5782|5792)'
```

Expect four rows bound to `0.0.0.0`. The `sim` container is `network_mode: host`, so its
listeners show up in the WSL host's namespace directly, and the portproxy from step 1 reaches
them with no relay in between.

Only if these show `127.0.0.1` do you need a relay -- and then it is one per port, bound to
the WSL interface:

```shell
for p in 5762 5772 5782 5792; do
  socat TCP-LISTEN:$p,bind=$(hostname -I | awk '{print $1}'),fork,reuseaddr TCP:127.0.0.1:$p &
done
```

It cannot bind `0.0.0.0:$p` -- that collides with the SITL's own loopback listener. A single
socat on some unrelated port (e.g. `TCP-LISTEN:15762`) does nothing: no other link in the
chain references that port.

## 5. Back to terminal 1

Press Enter. `iarc.py` spawns the four drones, which starts the sensor streams that unblock
the SITLs' physics loops, which is what finally makes them open their MAVLink ports. It then
polls 5762/5772/5782/5792 in turn (300 s timeout each). Wait for:

```
Scene is up and all SITLs are emitting MAVLink.
```

That line is the go signal for the Pis. Optional reachability check from any Pi first:

```shell
nc -vz <LAN_IP> 5762
```

## 6. Each Pi

One-time cleanup, so `sudo` is not needed. A `Permission denied` unlinking files under
`.venv/` means an earlier `sudo uv run` left root-owned files there:

```shell
sudo chown -R $USER:$USER ~/IARC-10/.venv && uv sync
```

If it comes back, something ran `uv` under `sudo` again -- repeat the chown rather than
re-adding `sudo`. `--airsim` touches no privileged device; only real mode needs `/dev/ttyS0`.

Then, per Pi, with `<id>` = 1, 2, 3, 4. **Each Pi gets its own id** -- it selects both the
`drone_info` entry and the SITL port `5762 + 10*(id-1)`:

```shell
SITL_MAVLINK_HOST=<LAN_IP> uv run run.py --airsim -i <id>
```

Preconditions on each Pi:

- `drone-flight@.service` must not be running. It runs real mode (`/dev/ttyS0`, no
  `--airsim`) and is wrong for sim demos: `sudo systemctl stop drone-flight@<id>`.
- Stash `mission_config.json` before `git pull`. `batman-mesh-setup.sh` rewrites it at boot
  with the `169.254.97.x` bat0 addresses; the committed version has `127.0.0.x`, which works
  only when all four processes share one host and breaks interdrone across four Pis.
- MST-GUEST does not isolate clients, so Pi -> host TCP works on it. `wlan1` carries MAVLink,
  `wlan0`/`bat0` carry the mesh.

## Failure modes

| Symptom | Cause |
| --- | --- |
| `no MAVLink from the SITL ... after 300s` | stale `SITL_HOST`/portproxy after a WSL IP change; or `NUM_DRONES` mismatch between terminals 1 and 2 |
| Instant `ECONNREFUSED` on a Pi (vs. a ~90 s dronekit timeout) | the Pi dialled its own loopback -- stale checkout without `SITL_MAVLINK_HOST` support |
| Pi connects, no interdrone traffic | `mission_config.json` overwritten with the committed `127.0.0.x` addresses |
| SITL windows stuck on `No sensor message received in last 1s` | drones never spawned, or AirSim's UDP is not reaching `SITL_HOST` |

The dronekit connection address is logged at INFO, and console logging drops to WARNING after
`flight_log.configure()` -- so it appears in `Logs/<run>/drone_<id>.log`, not on stdout.
