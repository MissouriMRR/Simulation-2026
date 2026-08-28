"""End-to-end accuracy check for vision/common/drone_coordinates.py against Project AirSim.

Flies one drone to a low hover, drops visible markers on the ground at positions the sim
knows exactly, photographs them with the nadir DownCamera, and asks
`pixel_to_geocoord_gimbal` where each marker's pixel points on the ground. The answer is
compared against the marker's true position, so the reported error is the real end-to-end
error of the flight pipeline -- GPS, attitude estimate and coordinate math together.

Why markers rather than a closed-form expectation: computing the "right" answer from the
drone's pose and the camera intrinsics would just be a second implementation of the same
projection, and the two would agree on any shared misunderstanding of the frames. A cube
sitting at a known spot on the ground cannot be argued with.

How a marker is located in the image without a detector: the marker is spawned *after* a
baseline frame has been captured, so the pixels that changed between the two frames are
exactly the marker. Segmentation frames are flat-shaded, so that difference is exact; the
scene frame is used as a fallback if segmentation capture is unavailable. Markers are
handled one at a time, which keeps the pixel-to-marker assignment unambiguous and lets each
one be measured against a freshly sampled pose.

Which blob is the marker
------------------------
"The largest thing that changed" is not enough. The drone drifts between the baseline and the
capture, so pixels change all over the frame, and a sprawling drift component can out-area the
marker; its centroid then lands near the principal point, which looks plausible for every
marker and is wrong for all of them -- and at a low hover the resulting miss is small enough
to sit inside a loose tolerance, so the run reports PASS having measured nothing.

Instead the marker's expected pixel area is known ahead of the capture (it is spawned at a
known size, at a known height) and blobs are filtered on it, on a ceiling fraction of the
frame, and on distance from where nadir geometry says the marker should be. That predicted
pixel is a second implementation of the projection, so it is used only to reject blobs, never
to score one: the error that gets asserted on is still the spawned marker's position against
the converted position. Two markers detected at the same pixel invalidate each other, since
markers metres apart cannot share one.

Pose in, truth out
------------------
The pose handed to `pixel_to_geocoord_gimbal` is the one the real flight code uses --
DroneKit's `location.global_relative_frame` and `attitude`. Expected values come from the
sim's ground truth. So a failure here is GPS/EKF error *or* a bug in the conversion. To tell
those apart, each row also reports what the conversion produces when fed the sim's exact
pose; that column is diagnostic only and is never asserted on. If the ground-truth column is
accurate and the DroneKit column is not, the math is fine and the autopilot's position
estimate is what moved.

Why it hovers at 45 degrees
---------------------------
The drone spawns pointing north, and at heading 0 any error in how the conversion applies
yaw vanishes -- a completely inverted yaw convention returns the same answers as a correct
one. Measuring nose-north would pass without testing yaw at all, so the run yaws off-axis
first. `--yaw-deg 0` gives the nose-north case for comparison; a run that passes at 0 and
fails at 45 has isolated the yaw handling.

At the default 10 ft the camera only sees about 6.1 m x 3.4 m of ground, so a metre of GPS
error is a sizeable fraction of the frame. `--alt-ft` raises the hover if the numbers are
too noisy to be informative.

Two processes, no environment changes
------------------------------------
The env container installs projectairsim into the system Python and the flight dependencies
into the uv project venv, so no single interpreter can import both projectairsim and
dronekit. This script is the projectairsim half and never imports dronekit; it spawns
coordinate_flight_helper.py under `uv run` for the flying and drives it over a pipe. That is
the same split interfaces/iarc.py already uses when it shells out to `uv run run.py`, and it
means the test runs on a stock container with nothing installed.

Usage (inside the `env` container, same handshake as interfaces/iarc.py):

    python simulation/tests/drone_coordinates_accuracy.py

Start the Unreal sim first; the script loads an empty scene, then waits for you to launch
the SITL container before spawning the drone. Exits non-zero if any marker misses by more
than `--tolerance-m`.
"""

from __future__ import annotations

import argparse
import collections
import collections.abc
import json
import math
import os
import queue
import re
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

# commentjson (a projectairsim dependency) still uses the pre-3.10 collections aliases.
# Must happen before projectairsim is imported, here and in iarc.py.
collections.MutableMapping = collections.abc.MutableMapping

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "simulation" / "interfaces"))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import iarc  # noqa: E402  -- ArduWorld plus the SITL/MAVLink handshake helpers
from projectairsim import Drone, ProjectAirSimClient, World  # noqa: E402
from projectairsim.types import ImageType, Pose, Quaternion, Vector3  # noqa: E402
from projectairsim.utils import (  # noqa: E402
    geo_to_ned_coordinates,
    projectairsim_log,
    quaternion_to_rpy,
    unpack_image,
)

from vision.common.drone_coordinates import (  # noqa: E402
    DronePose,
    GimbalPose,
    pixel_to_geocoord_gimbal,
)

FEET_TO_M = 0.3048

CAMERA_ID = "DownCamera"

# Where to put the markers, as (label, forward, right) fractions of the half-footprint the
# camera covers in each body axis at the hover altitude. Fractions rather than metres so the
# pattern still fits the frame when --alt-ft changes; 0.6 keeps them clear of the edge, where
# a marker straddling the border would bias its own centroid. A centre-only run would miss
# exactly the errors that grow with distance from the principal point, which is most of them.
MARKER_PLACEMENTS: list[tuple[str, float, float]] = [
    ("centre", 0.0, 0.0),
    ("forward", 0.6, 0.0),
    ("aft", -0.6, 0.0),
    ("right", 0.0, 0.6),
    ("left", 0.0, -0.6),
]

# A flat tile, not a cube: anything with height projects its top face rather than its
# footprint, and at a 3 m hover a 1 m tall object would throw the answer off by metres.
# These are scale factors on the asset's native size, so the preference list below favours
# the 1 m cubes, for which they are also metres.
#
# Width is sized as a fraction of the visible frame rather than fixed, so raising --alt-ft
# (the remedy when GPS noise swamps a 10 ft hover) does not shrink the marker below the
# detection floor. 0.08 puts it at roughly 32 px across on the 400 px-wide down camera.
MARKER_FRAME_FRACTION = 0.08
MARKER_MIN_SIZE_M = 0.5
MARKER_THICKNESS_M = 0.02

# Asset names to try for the marker, best first. Which of these exists depends on the
# packaged Unreal environment, so the list is probed against list_assets() at runtime.
MARKER_ASSET_PREFERENCES = [
    r"^1M_Cube_Chamfer$",
    r"^1M_Cube$",
    r".*Cube.*",
    r".*Box.*",
    r".*Cylinder$",
    r".*Sphere$",
]

# Pixels differing by more than this (0-255) count as changed when the scene image is used
# instead of segmentation. Segmentation frames are flat-shaded so they use a threshold of 0.
SCENE_DIFF_THRESHOLD = 25

# Blob plausibility. The marker's size in pixels is known in advance -- it is spawned at a
# known size at a known height -- so "the largest thing that changed" is a much weaker filter
# than the situation allows. Drift between the baseline and the capture changes pixels all
# over the frame, and a sprawling drift component can easily out-area the marker; taking it
# anyway yields a centroid near the principal point, which is the one answer that looks
# plausible for every marker and is wrong for all of them.
MARKER_AREA_MIN_RATIO = 0.25
MARKER_AREA_MAX_RATIO = 4.0
# No legitimate marker fills this much of the frame; anything that does is whole-frame drift.
MAX_BLOB_FRAME_FRACTION = 0.10
# Two markers metres apart cannot land on the same pixel. If they do, the detector is
# tracking something that is not the markers.
DUPLICATE_PIXEL_RADIUS = 8.0

# How far a detection may sit from where nadir geometry says the marker should be before it
# is treated as a mis-detection, as a fraction of the frame diagonal. This is a *detection*
# gate, never the accuracy measure: it constrains where the marker appears in the image, which
# the sim knows exactly, and not where the conversion says that pixel points, which is the
# thing under test. Tightening it therefore cannot hide a conversion error -- that error is
# measured downstream, from the converted position against the spawned position.
#
# 0.10 is ~80 px on a 640x480 frame. The slack it needs to cover is the nadir approximation
# (roll and pitch are ignored; 5 deg of tilt at a 3 m hover moves the marker ~40 px) plus
# centroid noise. At 0.25 the gate was useless for the case it exists to catch: with markers
# at 0.6 of the half-footprint, a blob stuck on the image centre sits ~190 px from the
# prediction and slipped straight through.
DETECTION_GATE_DIAGONAL_FRACTION = 0.10

DEFAULT_TOLERANCE_M = 2.0
# The tolerance has to be small next to what the camera can see, or it cannot fail: at a 10 ft
# hover the whole footprint is a few metres across, so a detector stuck on the image centre
# misses by less than the footprint's half-diagonal no matter what. A tolerance above this
# fraction of that half-diagonal is not measuring the conversion, so the run refuses to start.
MAX_TOLERANCE_FOOTPRINT_FRACTION = 1.0 / 3.0

# The flight half, run under `uv run` so it gets dronekit from the project venv. Arming,
# takeoff, yaw and settling all live over there; this side only sequences them.
HELPER_SCRIPT = Path(__file__).resolve().parent / "coordinate_flight_helper.py"
REPLY_PREFIX = "@@"
COMMAND_TIMEOUT_SEC = 60.0
# Generous: a cold container makes `uv run` sync the project venv before the child starts,
# and the child then waits on a DroneKit connection that itself allows two minutes.
STARTUP_TIMEOUT_SEC = 600.0
# Arming waits on is_armable, which needs the EKF to settle, then climbs and stabilises.
TAKEOFF_TIMEOUT_SEC = 300.0
YAW_TIMEOUT_SEC = 120.0

# The drone spawns 15 m up and falls; these govern the wait for it to come to rest so the
# ground plane can be measured.
GROUNDING_TIMEOUT_SEC = 60.0
GROUNDING_TOLERANCE_M = 0.02
GROUNDING_STABLE_SAMPLES = 4

# Heading to measure at. Not 0: at heading 0 an inverted yaw convention returns the same
# answers as a correct one, so a nose-north run proves nothing about how yaw is applied.
DEFAULT_YAW_DEG = 45.0


@dataclass
class CameraIntrinsics:
    """What the conversion needs to know about the camera, read from the robot config."""

    width: int
    height: int
    h_fov_rad: float
    v_fov_rad: float

    def footprint(self, altitude_m: float) -> tuple[float, float]:
        """Ground extent (forward, right) in metres visible from `altitude_m`, nadir."""
        forward = 2.0 * altitude_m * math.tan(self.v_fov_rad / 2.0)
        right = 2.0 * altitude_m * math.tan(self.h_fov_rad / 2.0)
        return forward, right


@dataclass
class GroundTruth:
    """The sim's exact answer for where the drone is and how it is oriented."""

    north: float
    east: float
    down: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass
class MarkerResult:
    """One marker's round trip from ground position to pixel and back."""

    label: str
    # Where the marker actually is. Every error below is measured against this.
    marker_ned: tuple[float, float]
    pixel: tuple[float, float] | None
    blob_pixels: int
    # Where the conversion says the pixel points, fed the DroneKit pose.
    computed_ned: tuple[float, float] | None
    computed_latlon: tuple[float, float] | None
    # Same, fed the sim's exact pose. Diagnostic only.
    gt_pose_ned: tuple[float, float] | None
    # Where nadir geometry says the marker should have appeared. Diagnostic only -- it gates
    # detection, it is never what the error is measured against.
    predicted_pixel: tuple[float, float] | None = None
    note: str = ""

    @property
    def error_m(self) -> float | None:
        """Horizontal miss, in metres, using the DroneKit pose. This is what is asserted."""
        return self._miss(self.computed_ned)

    @property
    def gt_pose_error_m(self) -> float | None:
        """Same miss recomputed from the sim's exact pose. Diagnostic only."""
        return self._miss(self.gt_pose_ned)

    def _miss(self, estimate: tuple[float, float] | None) -> float | None:
        if estimate is None:
            return None
        return math.hypot(estimate[0] - self.marker_ned[0], estimate[1] - self.marker_ned[1])


class CoordinateTestWorld(iarc.ArduWorld):
    """ArduWorld that also turns on the down camera's segmentation capture.

    The shared robot config leaves segmentation off, since nothing in the mission needs it.
    This test does: a segmentation frame is flat-shaded, so differencing two of them isolates
    a newly spawned marker exactly, with none of the lighting noise a scene-image difference
    picks up. Flipping it here rather than in robot_ardu_quadrotor.jsonc keeps the extra
    render cost to this test.
    """

    def __init__(self, *args, camera_id: str = CAMERA_ID, **kwargs):
        self._camera_id = camera_id
        self.robot_config: dict = {}
        super().__init__(*args, **kwargs)

    def _build_actors(self, actors: list, num_drones: int) -> list:
        built = super()._build_actors(actors, num_drones)
        for actor in built:
            robot_config = actor.get("robot-config", {})
            for sensor in robot_config.get("sensors", []):
                if sensor.get("id") != self._camera_id or sensor.get("type") != "camera":
                    continue
                for capture in sensor.get("capture-settings", []):
                    if capture.get("image-type") == int(ImageType.SEGMENTATION):
                        capture["capture-enabled"] = True
            # Every actor is a clone of the same template, so one copy describes them all.
            self.robot_config = robot_config
        return built


def read_camera_intrinsics(
    robot_config: dict, camera_id: str, fov_axis: str = "horizontal"
) -> CameraIntrinsics:
    """Pull resolution and field of view for `camera_id` out of the robot config.

    The config carries a single `fov-degrees`; the other axis is implied by the aspect ratio.
    Passing the same number for both -- the obvious mistake, since it is the only angle in the
    config -- stretches the narrow axis by nearly a third and walks every off-centre pixel
    away from its true ground point.

    Which axis `fov-degrees` describes is not stated in the Project AirSim config docs. It is
    taken as horizontal here, matching Unreal's `FOVAngle` and legacy AirSim's `FOV_Degrees`.
    If that is wrong the error will be strongly axis-dependent -- forward/aft markers off
    while left/right are fine, or the reverse -- which `--fov-axis vertical` then flips.
    """
    for sensor in robot_config.get("sensors", []):
        if sensor.get("id") != camera_id or sensor.get("type") != "camera":
            continue
        for capture in sensor.get("capture-settings", []):
            if capture.get("image-type") != int(ImageType.SCENE):
                continue
            width = int(capture["width"])
            height = int(capture["height"])
            fov = math.radians(float(capture["fov-degrees"]))
            if fov_axis == "vertical":
                v_fov = fov
                h_fov = 2.0 * math.atan(math.tan(fov / 2.0) * width / height)
            else:
                h_fov = fov
                v_fov = 2.0 * math.atan(math.tan(fov / 2.0) * height / width)
            return CameraIntrinsics(width, height, h_fov, v_fov)
    raise RuntimeError(
        f"camera '{camera_id}' has no scene capture-settings in the robot config. "
        f"Cameras present: {[s.get('id') for s in robot_config.get('sensors', [])]}"
    )


def pick_marker_asset(world: World) -> str:
    """Choose a marker asset that the running Unreal environment actually ships."""
    assets = world.list_assets(".*")
    if not assets:
        raise RuntimeError("the sim reported no spawnable assets, so no marker can be placed")

    for pattern in MARKER_ASSET_PREFERENCES:
        for asset in assets:
            if re.match(pattern, asset, flags=re.IGNORECASE):
                projectairsim_log().info(f"Using '{asset}' as the ground marker asset.")
                return asset
    raise RuntimeError(
        "none of the preferred marker assets are available in this environment. "
        f"Add one of these to MARKER_ASSET_PREFERENCES: {sorted(assets)[:40]}"
    )


def pose_at(north: float, east: float, down: float) -> Pose:
    """A Pose at a scene-NED point, unrotated."""
    return Pose(
        {
            "translation": Vector3({"x": north, "y": east, "z": down}),
            "rotation": Quaternion({"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}),
        }
    )


def read_ground_truth(drone: Drone) -> GroundTruth:
    """The drone's exact position and orientation, in one round trip.

    Position and orientation are read together because they are compared against a single
    photograph; two separate calls would straddle a physics step and disagree.
    """
    pose = drone.get_ground_truth_pose()
    translation = pose["translation"]
    rotation = pose["rotation"]
    roll, pitch, yaw = quaternion_to_rpy(
        float(rotation["w"]), float(rotation["x"]), float(rotation["y"]), float(rotation["z"])
    )
    return GroundTruth(
        north=float(translation["x"]),
        east=float(translation["y"]),
        down=float(translation["z"]),
        roll_deg=math.degrees(roll),
        pitch_deg=math.degrees(pitch),
        yaw_deg=math.degrees(yaw),
    )


def capture(drone: Drone, camera_id: str) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Grab a (scene, segmentation) pair. Either may be None if that type is not captured.

    Segmentation is only an aid to finding the markers, so a sim that refuses the request
    outright must not take the scene image down with it -- ask again for scene alone.
    """
    try:
        images = drone.get_images(camera_id, [int(ImageType.SCENE), int(ImageType.SEGMENTATION)])
    except Exception as err:  # noqa: BLE001 -- the fallback is the point, not the error type
        projectairsim_log().warning(
            f"Combined scene+segmentation capture failed ({err}); retrying scene only."
        )
        images = drone.get_images(camera_id, [int(ImageType.SCENE)])

    def unpack(image_type: ImageType) -> np.ndarray | None:
        message = images.get(int(image_type))
        if not message or not message.get("data"):
            return None
        return unpack_image(message)

    return unpack(ImageType.SCENE), unpack(ImageType.SEGMENTATION)


@dataclass
class Blob:
    """One connected region of pixels that changed between the baseline and the capture."""

    centroid: tuple[float, float]
    area: int


def find_blobs(baseline: np.ndarray, current: np.ndarray, threshold: int) -> list[Blob]:
    """Every blob of changed pixels between two frames, largest first."""
    if baseline.shape != current.shape:
        return []

    difference = cv2.absdiff(baseline, current)
    if difference.ndim == 3:
        difference = difference.max(axis=2)
    mask = (difference > threshold).astype(np.uint8)

    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    blobs = [
        Blob(
            (float(centroids[label][0]), float(centroids[label][1])),
            int(stats[label, cv2.CC_STAT_AREA]),
        )
        for label in range(1, count)  # label 0 is the unchanged background
    ]
    return sorted(blobs, key=lambda blob: blob.area, reverse=True)


def predict_pixel(
    marker_ned: tuple[float, float],
    truth: GroundTruth,
    ground_plane_z: float,
    intrinsics: CameraIntrinsics,
) -> tuple[float, float] | None:
    """Where a nadir camera would see `marker_ned`, for gating detections only.

    Roll and pitch are ignored -- this is a hover, and the gate is loose enough that a couple
    of degrees of tilt is irrelevant. It exists to answer "is this blob the marker?", never
    "is the conversion right?": using it for the latter would be the second-implementation
    trap the whole spawned-marker design avoids.
    """
    altitude = ground_plane_z - truth.down
    if altitude <= 0.0:
        return None

    yaw = math.radians(truth.yaw_deg)
    delta_north = marker_ned[0] - truth.north
    delta_east = marker_ned[1] - truth.east
    forward = delta_north * math.cos(yaw) + delta_east * math.sin(yaw)
    right = -delta_north * math.sin(yaw) + delta_east * math.cos(yaw)

    forward_extent, right_extent = intrinsics.footprint(altitude)
    px = intrinsics.width / 2.0 + right / right_extent * intrinsics.width
    py = intrinsics.height / 2.0 - forward / forward_extent * intrinsics.height
    return px, py


def select_marker_blob(
    blobs: list[Blob],
    expected_area_px: float,
    frame_area_px: int,
    predicted: tuple[float, float] | None,
    gate_px: float,
) -> tuple[Blob | None, str]:
    """Pick the blob that is actually the marker, or explain why none of them is.

    The marker's pixel area is known ahead of the capture, so plausibility is checked rather
    than assumed. The rejected candidates are summarised in the returned note: a run that
    finds nothing is only useful if it says what it did find instead.
    """
    if not blobs:
        return None, "no pixels changed between the baseline and the capture"

    candidates: list[Blob] = []
    for blob in blobs:
        if blob.area > frame_area_px * MAX_BLOB_FRAME_FRACTION:
            continue
        if not (
            expected_area_px * MARKER_AREA_MIN_RATIO
            <= blob.area
            <= expected_area_px * MARKER_AREA_MAX_RATIO
        ):
            continue
        if predicted is not None:
            offset = math.hypot(blob.centroid[0] - predicted[0], blob.centroid[1] - predicted[1])
            if offset > gate_px:
                continue
        candidates.append(blob)

    if not candidates:
        biggest = blobs[0]
        return None, (
            f"no blob looked like the marker (expected ~{expected_area_px:.0f} px); "
            f"largest of {len(blobs)} was {biggest.area} px at "
            f"({biggest.centroid[0]:.0f},{biggest.centroid[1]:.0f})"
        )

    # Closest to the expected size, not simply the largest: the marker's area is the one thing
    # known exactly, and drift blobs are what "largest" was picking up.
    best = min(candidates, key=lambda blob: abs(blob.area - expected_area_px))
    return best, ""


def latlon_to_ned(home_geo_point: dict, lat: float, lon: float) -> tuple[float, float]:
    """Horizontal scene-NED position of a lat/lon, for comparison in metres.

    Errors are judged in metres rather than degrees because a degree of longitude is not a
    degree of latitude, so a raw lat/lon difference hides which direction the miss was in.
    """
    north, east, _ = geo_to_ned_coordinates(home_geo_point, [lat, lon, home_geo_point["altitude"]])
    return float(north), float(east)


class FlightFailed(RuntimeError):
    """The flight-side process refused a command or went away."""


class FlightSide:
    """The flight half of the test, driven over a pipe in its own interpreter.

    The env container puts projectairsim in the system Python and the flight dependencies in
    the uv project venv, so no single interpreter can import both. Rather than requiring
    anyone to modify their container, this follows the split the repo already uses:
    interfaces/iarc.py runs on the system Python and spawns `uv run run.py` for flight. Here
    the child is coordinate_flight_helper.py, launched the same way, and the parent keeps
    control of sequencing so a marker is never photographed against a stale pose.
    """

    def __init__(
        self,
        repo_root: Path,
        address: str,
        startup_timeout: float,
        command: list[str] | None = None,
    ):
        # `uv run` is what puts dronekit on the child's path, matching how iarc.py starts
        # run.py. Overridable so the protocol can be exercised without a SITL.
        command = command or ["uv", "run", "python", str(HELPER_SCRIPT), "--address", address]
        projectairsim_log().info(f"Starting flight side: {' '.join(command)} (cwd {repo_root})")
        try:
            self._process = subprocess.Popen(
                command,
                cwd=str(repo_root),
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                # stderr is inherited so the child's progress lands on the console live.
                text=True,
                bufsize=1,
            )
        except FileNotFoundError as err:
            raise FlightFailed(
                "'uv' is not on PATH. The flight half runs under `uv run` so it picks up "
                "dronekit from the project venv, exactly as interfaces/iarc.py launches run.py."
            ) from err

        self._replies: queue.Queue = queue.Queue()
        self._reader = threading.Thread(target=self._pump_stdout, daemon=True)
        self._reader.start()

        # `uv run` may sync the project venv before the child starts, which on a cold
        # container is slow, so the first wait is the generous one.
        ready = self._await_reply(startup_timeout)
        if not ready.get("ok"):
            raise FlightFailed(ready.get("error", "flight side failed to start"))
        projectairsim_log().info("Flight side connected to the SITL.")

    def _pump_stdout(self) -> None:
        """Move replies onto the queue; anything else on stdout is the child's own chatter."""
        for line in self._process.stdout:
            line = line.strip()
            if line.startswith(REPLY_PREFIX):
                try:
                    self._replies.put(json.loads(line[len(REPLY_PREFIX) :]))
                except ValueError:
                    print(f"[flight] unparseable reply: {line}", file=sys.stderr)
            elif line:
                print(f"[flight] {line}", file=sys.stderr)
        # EOF: the child is gone. Unblock anyone waiting rather than hanging to the timeout.
        self._replies.put({"ok": False, "error": "flight side exited"})

    def _await_reply(self, timeout: float) -> dict:
        try:
            return self._replies.get(timeout=timeout)
        except queue.Empty as err:
            raise FlightFailed(f"flight side did not reply within {timeout:.0f}s") from err

    def request(self, cmd: str, timeout: float = COMMAND_TIMEOUT_SEC, **params) -> dict:
        if self._process.poll() is not None:
            raise FlightFailed(f"flight side exited with code {self._process.returncode}")
        self._process.stdin.write(json.dumps({"cmd": cmd, **params}) + "\n")
        self._process.stdin.flush()

        response = self._await_reply(timeout)
        if not response.get("ok"):
            raise FlightFailed(f"{cmd}: {response.get('error', 'refused')}")
        return response

    def pose(self) -> DronePose:
        """The pose the real flight code would be working from, ready for the conversion."""
        pose = self.request("pose")["pose"]
        return DronePose(
            lat=pose["lat"],
            lon=pose["lon"],
            altitude=pose["alt_m"],
            yaw=pose["yaw_deg"],
            pitch=pose["pitch_deg"],
            roll=pose["roll_deg"],
        )

    def close(self, land: bool) -> None:
        if self._process.poll() is not None:
            return
        try:
            if land:
                projectairsim_log().info("Landing...")
                self.request("land")
                time.sleep(2)
            self.request("quit")
        except (FlightFailed, OSError) as err:
            projectairsim_log().warning(f"Flight side did not shut down cleanly: {err}")
        finally:
            try:
                self._process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                self._process.kill()


def wait_until_grounded(drone: Drone) -> float:
    """Wait for the drone to come to rest on the ground, and return that scene-NED z.

    The scene spawns the drone 15 m up, so it is still falling when the SITL first comes
    alive. Reading the ground plane mid-fall would place every marker at the wrong height,
    and since the conversion intersects a ray with that plane, the whole run would be
    measuring against fiction. So poll until the height stops changing.
    """
    deadline = time.monotonic() + GROUNDING_TIMEOUT_SEC
    previous = read_ground_truth(drone).down
    stable_samples = 0

    while time.monotonic() < deadline:
        time.sleep(0.5)
        current = read_ground_truth(drone).down
        if abs(current - previous) < GROUNDING_TOLERANCE_M:
            stable_samples += 1
            if stable_samples >= GROUNDING_STABLE_SAMPLES:
                return current
        else:
            stable_samples = 0
        previous = current

    projectairsim_log().warning(
        f"Drone height was still changing after {GROUNDING_TIMEOUT_SEC:.0f}s "
        f"(scene NED z = {previous:.2f} m). Using it anyway; if the markers come out "
        "floating or buried, this is why."
    )
    return previous


def measure_marker(
    *,
    label: str,
    drone: Drone,
    world: World,
    flight: FlightSide,
    asset: str,
    marker_ned: tuple[float, float, float],
    marker_size_m: float,
    baseline_scene: np.ndarray | None,
    baseline_segmentation: np.ndarray | None,
    intrinsics: CameraIntrinsics,
    camera_id: str,
    ground_plane_z: float,
    home_geo_point: dict,
    expected_area_px: float,
) -> MarkerResult:
    """Spawn one marker, photograph it, and convert its pixel back to a ground position."""
    north, east, down = marker_ned
    marker_position = (north, east)
    object_name = f"coord_test_{label}"
    spawned = ""

    try:
        spawned = world.spawn_object(
            object_name,
            asset,
            pose_at(north, east, down),
            [marker_size_m, marker_size_m, MARKER_THICKNESS_M],
            False,
        )
        # The marker has to be rendered before it can be photographed.
        time.sleep(0.5)

        scene, segmentation = capture(drone, camera_id)

        # Sample the pose next to the capture, not once at the start: the drone drifts
        # between markers, and a stale pose would be charged to the conversion.
        drone_pose = flight.pose()
        truth = read_ground_truth(drone)
        truth_geo = drone.get_ground_truth_geo_location()

        predicted = predict_pixel(marker_position, truth, ground_plane_z, intrinsics)
        frame_area_px = intrinsics.width * intrinsics.height
        gate_px = (
            math.hypot(intrinsics.width, intrinsics.height) * DETECTION_GATE_DIAGONAL_FRACTION
        )

        # Segmentation first (flat-shaded, so the difference is exact), scene as the fallback.
        # Both are tried even when segmentation returns something implausible: a rejected
        # segmentation blob says nothing about whether the scene image holds the marker.
        found: Blob | None = None
        note = ""
        for baseline_frame, current_frame, threshold in (
            (baseline_segmentation, segmentation, 0),
            (baseline_scene, scene, SCENE_DIFF_THRESHOLD),
        ):
            if baseline_frame is None or current_frame is None:
                continue
            blobs = find_blobs(baseline_frame, current_frame, threshold)
            found, note = select_marker_blob(
                blobs, expected_area_px, frame_area_px, predicted, gate_px
            )
            if found is not None:
                break

        if found is None:
            return MarkerResult(
                label=label,
                marker_ned=marker_position,
                pixel=None,
                blob_pixels=0,
                computed_ned=None,
                computed_latlon=None,
                gt_pose_ned=None,
                predicted_pixel=predicted,
                note=note or "marker not visible in frame",
            )

        (px, py), area = found.centroid, found.area

        # The down camera is bolted to the frame at a -90 deg pitch, which is the mounting
        # that makes GimbalPose()'s zero rotation mean "straight down" in this model. The
        # drone's own attitude is applied separately, which is what a rigid mount does.
        gimbal_pose = GimbalPose()

        # h_fov/v_fov are consumed as radians here (tan(h_fov / 2), no conversion), unlike
        # the copy in tests/flight_day_3-7 which converts from degrees.
        computed_latlon = pixel_to_geocoord_gimbal(
            px=px,
            py=py,
            image_width=intrinsics.width,
            image_height=intrinsics.height,
            h_fov=intrinsics.h_fov_rad,
            v_fov=intrinsics.v_fov_rad,
            drone=drone_pose,
            gimbal=gimbal_pose,
        )

        # The same conversion again, but handed the pose the sim knows to be exact. If this
        # column is tight and the DroneKit one is not, the maths is sound and the autopilot's
        # position estimate is what moved.
        gt_pose_latlon = None
        if truth_geo:
            gt_pose_latlon = pixel_to_geocoord_gimbal(
                px=px,
                py=py,
                image_width=intrinsics.width,
                image_height=intrinsics.height,
                h_fov=intrinsics.h_fov_rad,
                v_fov=intrinsics.v_fov_rad,
                drone=DronePose(
                    lat=float(truth_geo["latitude"]),
                    lon=float(truth_geo["longitude"]),
                    # AGL measured to the plane the markers sit on, not the geoid.
                    altitude=ground_plane_z - truth.down,
                    yaw=truth.yaw_deg,
                    pitch=truth.pitch_deg,
                    roll=truth.roll_deg,
                ),
                gimbal=gimbal_pose,
            )

        return MarkerResult(
            label=label,
            marker_ned=marker_position,
            pixel=(px, py),
            blob_pixels=area,
            computed_ned=(
                latlon_to_ned(home_geo_point, *computed_latlon) if computed_latlon else None
            ),
            computed_latlon=computed_latlon,
            gt_pose_ned=(
                latlon_to_ned(home_geo_point, *gt_pose_latlon) if gt_pose_latlon else None
            ),
            predicted_pixel=predicted,
            note="" if computed_latlon else "ray did not intersect the ground plane",
        )
    finally:
        if spawned:
            world.destroy_object(spawned)
            time.sleep(0.2)


def annotate(image: np.ndarray, results: list[MarkerResult], path: Path) -> None:
    """Save the baseline frame with a crosshair where each marker was detected.

    The baseline is the marker-free capture, so the crosshairs show where the markers were
    found rather than sitting on top of them -- which is what you want when checking whether
    a detection landed on the tile's centre or drifted onto its shadow.
    """
    canvas = image.copy()
    if canvas.ndim == 2:
        canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)

    for result in results:
        if result.pixel is None:
            continue
        x, y = int(round(result.pixel[0])), int(round(result.pixel[1]))
        cv2.drawMarker(canvas, (x, y), (0, 0, 255), cv2.MARKER_CROSS, 12, 1)
        cv2.putText(
            canvas, result.label, (x + 6, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1
        )

    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), canvas)
    projectairsim_log().info(f"Annotated capture written to {path}")


def report(
    results: list[MarkerResult],
    tolerance_m: float,
    drone_gps_error_m: float,
    heading_deg: float,
) -> bool:
    """Print the per-marker table and return whether every marker met the tolerance."""
    header = (
        f"{'marker':<9} {'pixel':<13} {'expect px':<13} {'blob':>7} {'marker N,E':<16} "
        f"{'computed N,E':<18} {'error':>7} {'gt-pose':>8}"
    )
    print()
    print(header)
    print("-" * len(header))

    passed = True
    for result in results:
        pixel = f"{result.pixel[0]:.0f},{result.pixel[1]:.0f}" if result.pixel else "-"
        expected_pixel = (
            f"{result.predicted_pixel[0]:.0f},{result.predicted_pixel[1]:.0f}"
            if result.predicted_pixel
            else "-"
        )
        blob = f"{result.blob_pixels:7d}" if result.blob_pixels else f"{'-':>7}"
        actual = f"{result.marker_ned[0]:+.2f},{result.marker_ned[1]:+.2f}"
        computed = (
            f"{result.computed_ned[0]:+.2f},{result.computed_ned[1]:+.2f}"
            if result.computed_ned
            else "-"
        )
        error = result.error_m
        gt_error = result.gt_pose_error_m
        error_text = f"{error:7.2f}" if error is not None else f"{'-':>7}"
        gt_text = f"{gt_error:8.2f}" if gt_error is not None else f"{'-':>8}"
        print(
            f"{result.label:<9} {pixel:<13} {expected_pixel:<13} {blob} {actual:<16} "
            f"{computed:<18} {error_text} {gt_text}"
        )
        if result.note:
            print(f"{'':<9} -> {result.note}")
        if error is None or error > tolerance_m:
            passed = False

    print("-" * len(header))
    print(
        f"measured at heading {heading_deg:.0f} deg; tolerance {tolerance_m:.2f} m; DroneKit "
        f"position was {drone_gps_error_m:.2f} m from the sim's truth at capture time."
    )
    print(
        "'error' uses the DroneKit pose the flight code would have had; 'gt-pose' repeats the "
        "conversion with the sim's exact pose and is diagnostic only."
    )
    print(
        "'expect px' is where nadir geometry says the marker should have appeared; it gates "
        "detection only and is never what 'error' is measured against."
    )
    if drone_gps_error_m > tolerance_m:
        print(
            "NOTE: the autopilot's own position error already exceeds the tolerance, so no "
            "pixel can pass. Raise --tolerance-m or --alt-ft, or let the EKF settle longer."
        )
    print("RESULT:", "PASS" if passed else "FAIL")
    return passed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--alt-ft",
        type=float,
        default=10.0,
        help="hover altitude in feet AGL (default: %(default)s). The camera footprint scales "
        "with this, and so does the share of the frame that GPS error accounts for.",
    )
    parser.add_argument(
        "--tolerance-m",
        type=float,
        default=DEFAULT_TOLERANCE_M,
        help="maximum acceptable horizontal miss per marker, in metres (default: %(default)s). "
        "Must stay well under the camera footprint's half-diagonal or nothing in the frame can "
        "fail it; the run refuses to start otherwise and prints the ceiling for the altitude.",
    )
    parser.add_argument(
        "--ground-offset-m",
        type=float,
        default=0.0,
        help="added to the drone's resting height to get the plane markers sit on, positive "
        "down (default: %(default)s). The resting height is the drone's origin, which stands "
        "off the terrain by the height of its body and gear; raise this if markers spawn "
        "floating, lower it if they spawn buried.",
    )
    parser.add_argument(
        "--yaw-deg",
        type=float,
        default=DEFAULT_YAW_DEG,
        help="compass heading to hover at while measuring (default: %(default)s). Do not use "
        "0 unless you specifically want the nose-north case: at heading 0 an inverted yaw "
        "convention gives the same answers as a correct one, so the run proves nothing about "
        "yaw. Compare a run at 0 against one at 45 to isolate yaw handling.",
    )
    parser.add_argument(
        "--camera-id",
        default=CAMERA_ID,
        help="camera sensor to photograph the markers with (default: %(default)s)",
    )
    parser.add_argument(
        "--fov-axis",
        choices=("horizontal", "vertical"),
        default="horizontal",
        help="which axis the config's fov-degrees describes (default: %(default)s). Flip this "
        "if the misses are large along one image axis and small along the other.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=REPO_ROOT / "Logs" / "coordinate_accuracy",
        help="where to write the annotated capture (default: %(default)s)",
    )
    parser.add_argument(
        "--no-land",
        action="store_true",
        help="leave the drone hovering instead of landing, for repeated manual runs",
    )
    return parser.parse_args()


def run(args: argparse.Namespace) -> bool:
    client = ProjectAirSimClient(address=os.environ.get("PAS_HOST", "127.0.0.1"))
    flight = None
    altitude_m = args.alt_ft * FEET_TO_M

    try:
        projectairsim_log().info("Connecting to Project AirSim...")
        client.connect()

        # Same three-step handshake as interfaces/iarc.py: an empty scene gives the SITL
        # something to pull scene data from, and a drone that spawns before its SITL exists
        # softlocks with no way back.
        World(client, iarc.EMPTY_SCENE, delay_after_load_sec=2)
        iarc.wait_for_sitl_launch()

        world = CoordinateTestWorld(
            client,
            iarc.IARC_SCENE,
            delay_after_load_sec=2,
            num_drones=1,
            camera_id=args.camera_id,
        )
        drone = Drone(client, world, iarc.drone_name(1))
        iarc.wait_for_mavlink()

        intrinsics = read_camera_intrinsics(world.robot_config, args.camera_id, args.fov_axis)
        forward_extent, right_extent = intrinsics.footprint(altitude_m)
        projectairsim_log().info(
            f"{args.camera_id}: {intrinsics.width}x{intrinsics.height}, "
            f"h_fov {math.degrees(intrinsics.h_fov_rad):.1f} deg, "
            f"v_fov {math.degrees(intrinsics.v_fov_rad):.1f} deg -> ground footprint at "
            f"{altitude_m:.2f} m is {forward_extent:.2f} m fwd x {right_extent:.2f} m right"
        )

        # A tolerance that approaches the footprint's half-diagonal cannot be failed by any
        # pixel in the frame, so the run would report PASS without measuring anything.
        half_diagonal = math.hypot(forward_extent, right_extent) / 2.0
        tolerance_ceiling = half_diagonal * MAX_TOLERANCE_FOOTPRINT_FRACTION
        if args.tolerance_m > tolerance_ceiling:
            raise RuntimeError(
                f"--tolerance-m {args.tolerance_m:.2f} is too loose to mean anything at this "
                f"altitude: the camera only sees {forward_extent:.2f} x {right_extent:.2f} m, "
                f"so no pixel in the frame can miss by more than {half_diagonal:.2f} m and "
                f"every run would pass. Use --tolerance-m {tolerance_ceiling:.2f} or less, or "
                "raise --alt-ft to widen the footprint."
            )

        home_geo_point = world.home_geo_point
        asset = pick_marker_asset(world)
        marker_size = max(MARKER_MIN_SIZE_M, MARKER_FRAME_FRACTION * right_extent)
        marker_width_px = marker_size / right_extent * intrinsics.width
        marker_height_px = marker_size / forward_extent * intrinsics.height
        expected_area_px = marker_width_px * marker_height_px
        projectairsim_log().info(
            f"Markers will be {marker_size:.2f} m square "
            f"(~{marker_width_px:.0f} px across, ~{expected_area_px:.0f} px in area)"
        )

        # The ground plane, taken as the drone's own resting height. DroneKit's relative
        # altitude is measured from this same spot, so markers placed here sit at exactly
        # the "AGL = 0" plane the conversion assumes -- no separate terrain lookup needed.
        resting_z = wait_until_grounded(drone)
        ground_plane_z = resting_z + args.ground_offset_m
        projectairsim_log().info(
            f"Ground plane taken as scene NED z = {ground_plane_z:.2f} m "
            f"(drone rested at {resting_z:.2f} m, offset {args.ground_offset_m:+.2f} m). "
            "This is the drone's own origin, which sits a little above the terrain by however "
            "much its body and gear stand off the ground; --ground-offset-m corrects that if "
            "the markers come out floating or buried."
        )

        flight = FlightSide(
            REPO_ROOT,
            f"tcp:127.0.0.1:{iarc.SITL_MAVLINK_PORT}",
            startup_timeout=STARTUP_TIMEOUT_SEC,
        )

        projectairsim_log().info(f"Taking off to {altitude_m:.2f} m ({args.alt_ft} ft) AGL...")
        flight.request("takeoff", timeout=TAKEOFF_TIMEOUT_SEC, alt_m=altitude_m)

        projectairsim_log().info(f"Yawing to {args.yaw_deg:.0f} deg to expose yaw handling...")
        flight.request("yaw", timeout=YAW_TIMEOUT_SEC, heading_deg=args.yaw_deg)

        # Marker offsets are laid out in the body frame and rotated into NED by the drone's
        # true heading, so they land in the frame whatever heading the hover settled on.
        hover = read_ground_truth(drone)
        yaw = math.radians(hover.yaw_deg)

        baseline_scene, baseline_segmentation = capture(drone, args.camera_id)
        if baseline_segmentation is None:
            projectairsim_log().warning(
                "No segmentation frame available; falling back to differencing the scene "
                "image, which is more sensitive to lighting and shadow."
            )
        if baseline_scene is None and baseline_segmentation is None:
            raise RuntimeError(
                f"camera '{args.camera_id}' returned no images. Check that its scene capture "
                "is enabled in the robot config."
            )

        results: list[MarkerResult] = []
        for label, forward_fraction, right_fraction in MARKER_PLACEMENTS:
            forward = forward_fraction * forward_extent / 2.0
            right = right_fraction * right_extent / 2.0
            north = hover.north + forward * math.cos(yaw) - right * math.sin(yaw)
            east = hover.east + forward * math.sin(yaw) + right * math.cos(yaw)

            projectairsim_log().info(
                f"Marker '{label}' at scene NED ({north:.2f}, {east:.2f}) -- "
                f"{forward:+.2f} m fwd, {right:+.2f} m right of the drone"
            )
            result = measure_marker(
                label=label,
                drone=drone,
                world=world,
                flight=flight,
                asset=asset,
                marker_ned=(north, east, ground_plane_z),
                marker_size_m=marker_size,
                baseline_scene=baseline_scene,
                baseline_segmentation=baseline_segmentation,
                intrinsics=intrinsics,
                camera_id=args.camera_id,
                ground_plane_z=ground_plane_z,
                home_geo_point=home_geo_point,
                expected_area_px=expected_area_px,
            )

            # Markers metres apart cannot share a pixel. When they do, the detector is locked
            # onto something that is not the markers -- the drone's own shadow, or a drift
            # component whose centroid sits near the principal point -- and the conversion is
            # being handed the same input every time. Discarding the reading keeps that from
            # scoring as a near-miss instead of the failure it is.
            twin = next(
                (
                    other
                    for other in results
                    if other.pixel is not None
                    and result.pixel is not None
                    and math.hypot(
                        other.pixel[0] - result.pixel[0], other.pixel[1] - result.pixel[1]
                    )
                    < DUPLICATE_PIXEL_RADIUS
                ),
                None,
            )
            if twin is not None:
                result.computed_ned = None
                result.computed_latlon = None
                result.note = (
                    f"detected at the same pixel as '{twin.label}', which is impossible for "
                    "markers this far apart -- the detection is not tracking the markers"
                )

            results.append(result)

        # How far the autopilot thought it was from where it actually was. This is the floor
        # under every error in the table.
        reported = flight.pose()
        reported_north, reported_east = latlon_to_ned(home_geo_point, reported.lat, reported.lon)
        actual = read_ground_truth(drone)
        drone_gps_error = math.hypot(reported_north - actual.north, reported_east - actual.east)

        if baseline_scene is not None:
            annotate(baseline_scene, results, args.out_dir / "markers_annotated.png")

        return report(results, args.tolerance_m, drone_gps_error, actual.yaw_deg)

    finally:
        if flight is not None:
            flight.close(land=not args.no_land)
        client.disconnect()


def main() -> int:
    args = parse_args()
    try:
        return 0 if run(args) else 1
    except Exception as err:  # noqa: BLE001 -- a sim run should report, not traceback-dump
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
