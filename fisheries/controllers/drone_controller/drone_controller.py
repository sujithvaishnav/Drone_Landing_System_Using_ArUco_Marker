from controller import Robot

from sensors import Sensors
from motors import Motors
from aruco import ArucoDetector
from flight_controller import FlightController

from config import *
from utils import log, clamp


robot = Robot()

timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

log("Starting Drone Controller")

sensors = Sensors(robot, timestep)
motors = Motors(robot)
aruco = ArucoDetector(robot, timestep)
fc = FlightController()

TAKEOFF_HEIGHT = 2.0

# ✅ WAIT TIMER
wait_start_time = None

# ─────────────────────────────────────────────
# Snake Waypoints
# ─────────────────────────────────────────────
def generate_snake():
    waypoints = []
    for x in range(-5, 6):
        if (x + 5) % 2 == 0:
            ys = range(-5, 6)
        else:
            ys = range(5, -6, -1)

        for y in ys:
            waypoints.append((x, y, TAKEOFF_HEIGHT))

    return waypoints

waypoints = generate_snake()
wp_index = 0

# ─────────────────────────────────────────────
# STATES
# ─────────────────────────────────────────────
state = "TAKEOFF"

detected_markers = {}   # {id: (x, y)}
target_marker_id = None
target_position = None

marker_lost_counter = 0
MARKER_LOST_LIMIT = 10

prev_vx = 0
prev_vy = 0

# ─────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────
while robot.step(timestep) != -1:

    (
        roll, pitch, yaw,
        roll_rate, pitch_rate, yaw_rate,
        x, y, z,
        vel_x, vel_y, vel_z
    ) = sensors.read(dt)

    if not fc.yaw_initialized:
        fc.yaw_target = yaw
        fc.yaw_initialized = True

    # ─────────────────────────────
    # TAKEOFF
    # ─────────────────────────────
    if state == "TAKEOFF":

        error_z = TAKEOFF_HEIGHT - z
        fc.target_vel_z = clamp(0.6 * error_z, -MAX_VEL_Z, MAX_VEL_Z)

        if abs(error_z) < VERTICAL_TOLERANCE:
            fc.target_vel_z = 0
            state = "SCAN"
            log("Takeoff → SCAN")

    # ─────────────────────────────
    # SCAN
    # ─────────────────────────────
    elif state == "SCAN":

        detections = aruco.detect_all()

        for marker_id, mx, my in detections:

            if marker_id not in detected_markers:

                corrected_x = x + mx * 0.3
                corrected_y = y + my * 0.3

                detected_markers[marker_id] = (corrected_x, corrected_y)

                log(f"Stored Marker {marker_id} at ({corrected_x:.2f}, {corrected_y:.2f})")

        if wp_index >= len(waypoints):
            log("Scan complete → WAIT")
            log(f"Available markers: {list(detected_markers.keys())}")
            state = "WAIT"
            continue

        target_x, target_y, target_z = waypoints[wp_index]

        error_x = target_x - x
        error_y = target_y - y
        error_z = target_z - z

        dist = (error_x**2 + error_y**2)**0.5

        if dist < HORIZONTAL_TOLERANCE:
            wp_index += 1
        else:
            fc.target_vel_x = clamp(0.5 * error_x, -0.3, 0.3)
            fc.target_vel_y = clamp(0.5 * error_y, -0.3, 0.3)

        fc.target_vel_z = clamp(0.6 * error_z, -MAX_VEL_Z, MAX_VEL_Z)

    # ─────────────────────────────
    # WAIT (AUTO SELECT MARKER 10)
    # ─────────────────────────────
    elif state == "WAIT":

        fc.target_vel_x *= 0.8
        fc.target_vel_y *= 0.8
        fc.target_vel_z = 0

        # start timer
        if wait_start_time is None:
            wait_start_time = robot.getTime()

        # after 1 second → go to marker 10
        if robot.getTime() - wait_start_time > 1.0:

            target_marker_id = 10

            if target_marker_id in detected_markers:
                target_position = detected_markers[target_marker_id]
                state = "NAVIGATE"
                log("🚀 Selected Marker 10 → NAVIGATE")
            else:
                log("❌ Marker 10 not found")

            wait_start_time = None

    # ─────────────────────────────
    # NAVIGATE
    # ─────────────────────────────
    elif state == "NAVIGATE":

        target_x, target_y = target_position

        dx = target_x - x
        dy = target_y - y

        fc.target_vel_x = clamp(0.3 * dx, -0.2, 0.2)
        fc.target_vel_y = clamp(0.3 * dy, -0.2, 0.2)

        error_z = TAKEOFF_HEIGHT - z
        fc.target_vel_z = clamp(0.4 * error_z, -0.2, 0.2)

        marker_id, mx, my = aruco.detector(target_marker_id)

        if marker_id is not None:
            log("Marker visible → ALIGN")
            state = "ALIGN"
            continue

        if abs(dx) < 0.1 and abs(dy) < 0.1:
            state = "ALIGN"

    # ─────────────────────────────
    # ALIGN
    # ─────────────────────────────
    elif state == "ALIGN":

        marker_id, mx, my = aruco.detect(target_marker_id)

        if marker_id is None:
            marker_lost_counter += 1
            if marker_lost_counter > MARKER_LOST_LIMIT:
                state = "NAVIGATE"
            continue

        marker_lost_counter = 0

        if abs(mx) < 0.03: mx = 0
        if abs(my) < 0.03: my = 0

        vx = -0.2 * my
        vy = -0.4 * mx

        alpha = 0.7
        fc.target_vel_x = alpha * prev_vx + (1 - alpha) * vx
        fc.target_vel_y = alpha * prev_vy + (1 - alpha) * vy

        prev_vx = fc.target_vel_x
        prev_vy = fc.target_vel_y

        fc.target_vel_x = clamp(fc.target_vel_x, -0.08, 0.08)
        fc.target_vel_y = clamp(fc.target_vel_y, -0.15, 0.15)

        error_z = TAKEOFF_HEIGHT - z
        fc.target_vel_z = clamp(0.3 * error_z, -0.2, 0.2)

        if abs(mx) < 0.02 and abs(my) < 0.02:
            state = "HOVER"
            log("Aligned → HOVER")

    # ─────────────────────────────
    # HOVER
    # ─────────────────────────────
    elif state == "HOVER":

        fc.target_vel_x *= 0.8
        fc.target_vel_y *= 0.8

        if abs(fc.target_vel_x) < 0.01:
            fc.target_vel_x = 0
        if abs(fc.target_vel_y) < 0.01:
            fc.target_vel_y = 0

        error_z = TAKEOFF_HEIGHT - z
        fc.target_vel_z = clamp(0.3 * error_z, -0.2, 0.2)

    # ─────────────────────────────
    # MOTOR CONTROL
    # ─────────────────────────────
    fl, fr, rl, rr = fc.compute_motor_commands(
        roll, pitch, yaw,
        roll_rate, pitch_rate, yaw_rate,
        vel_x, vel_y, vel_z
    )

    motors.set(fl, fr, rl, rr)