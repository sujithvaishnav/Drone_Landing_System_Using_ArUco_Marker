from controller import Robot

from sensors import Sensors
from motors import Motors
from aruco import ArucoDetector
from navigation import Navigation
from flight_controller import FlightController
from logger import FlightLogger

from config import *
from utils import log, clamp


robot = Robot()

timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0


log("Starting Drone Controller")


sensors = Sensors(robot, timestep)
motors = Motors(robot)
aruco = ArucoDetector(robot, timestep)
nav = Navigation()
fc = FlightController()
logger = FlightLogger()

TAKEOFF_HEIGHT = 2.0


# ─────────────────────────────────────────────
# Snake Waypoint Generator
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

state = "TAKEOFF"

marker_lost_counter = 0
MARKER_LOST_LIMIT = 10
sim_time = 0.0
ALIGN_TOLERANCE = 0.03
LANDING_HEIGHT = 0.15
DESCENT_SPEED = -0.15


# ─────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────
while robot.step(timestep) != -1:
    sim_time += dt
    (
        roll, pitch, yaw,
        roll_rate, pitch_rate, yaw_rate,
        x, y, z,
        vel_x, vel_y, vel_z
    ) = sensors.read(dt)

    if not fc.yaw_initialized:
        fc.yaw_target = yaw
        fc.yaw_initialized = True

    marker_detected = 0
    mx = 0.0
    my = 0.0


    # ─────────────────────────────
    # TAKEOFF
    # ─────────────────────────────
    if state == "TAKEOFF":

        error_z = TAKEOFF_HEIGHT - z
        fc.target_vel_z = clamp(0.6 * error_z, -MAX_VEL_Z, MAX_VEL_Z)

        if abs(error_z) < VERTICAL_TOLERANCE:
            fc.target_vel_z = 0
            state = "SEARCH"
            log("Takeoff Complete → SEARCH")


    # ─────────────────────────────
    # SEARCH (Snake Movement)
    # ─────────────────────────────
    elif state == "SEARCH":

        # 🔍 Detect marker FIRST
        marker_id, detected_mx, detected_my = aruco.detect(TARGET_MARKER_IDS[0])

        if marker_id is not None:
            marker_detected = 1
            mx = detected_mx
            my = detected_my
            log("Marker detected → ALIGN")

            fc.target_vel_x = 0
            fc.target_vel_y = 0

            state = "ALIGN"
            continue

        # 🧭 Waypoint navigation
        if wp_index >= len(waypoints):
            log("Search complete - no marker found")
            state = "DONE"
            continue

        target_x, target_y, target_z = waypoints[wp_index]

        error_x = target_x - x
        error_y = target_y - y
        error_z = target_z - z

        distance = (error_x**2 + error_y**2)**0.5

        log(f"WP {wp_index} | Pos ({x:.2f},{y:.2f}) → Target ({target_x},{target_y})")

        if distance < HORIZONTAL_TOLERANCE:
            wp_index += 1
            log(f"Reached waypoint {wp_index}")
        else:
            fc.target_vel_x = clamp(0.5 * error_x, -0.4, 0.4)
            fc.target_vel_y = clamp(0.5 * error_y, -0.4, 0.4)

        # Maintain altitude
        fc.target_vel_z = clamp(0.6 * error_z, -MAX_VEL_Z, MAX_VEL_Z)


    # ─────────────────────────────
    # ALIGN
    # ─────────────────────────────
    elif state == "ALIGN":

        marker_id, detected_mx, detected_my = aruco.detect(TARGET_MARKER_IDS[0])

        if marker_id is not None:
            marker_detected = 1
            mx = detected_mx
            my = detected_my

        if marker_id is None:

            marker_lost_counter += 1

            if marker_lost_counter > MARKER_LOST_LIMIT:
                log("Marker lost → SEARCH")
                state = "SEARCH"

            continue

        marker_lost_counter = 0

        # Continuous alignment correction
        vx = -ALIGN_GAIN * my
        vy = -ALIGN_GAIN * mx

        fc.target_vel_x = clamp(vx, -0.2, 0.2)
        fc.target_vel_y = clamp(vy, -0.2, 0.2)

        # Maintain altitude during alignment
        error_z = TAKEOFF_HEIGHT - z
        fc.target_vel_z = clamp(0.6 * error_z, -MAX_VEL_Z, MAX_VEL_Z)

        log(f"ALIGN | mx:{mx:.3f}, my:{my:.3f}, z:{z:.2f}")

        # Check if centered above marker
        if abs(mx) < ALIGN_TOLERANCE and abs(my) < ALIGN_TOLERANCE:

            log("Alignment successful → DESCEND")

            state = "DESCEND"

        # ❗ DO NOT change state anymore
        # Stay in ALIGN forever = stable hoverw
    # ─────────────────────────────
    # DESCEND
    # ─────────────────────────────
    elif state == "DESCEND":

        marker_id, detected_mx, detected_my = aruco.detect(TARGET_MARKER_IDS[0])

        if marker_id is not None:
            marker_detected = 1
            mx = detected_mx
            my = detected_my

            # Continue correcting during descent
            vx = -ALIGN_GAIN * my
            vy = -ALIGN_GAIN * mx

            fc.target_vel_x = clamp(vx, -0.15, 0.15)
            fc.target_vel_y = clamp(vy, -0.15, 0.15)

        else:
            # If marker lost during descent
            fc.target_vel_x = 0
            fc.target_vel_y = 0

        # Slow descent
        fc.target_vel_z = DESCENT_SPEED

        log(f"DESCEND | z:{z:.2f}")

        # Landing condition
        if z <= LANDING_HEIGHT:

            log("Landing complete")

            state = "LAND"

    # ─────────────────────────────
    # LAND
    # ─────────────────────────────
    elif state == "LAND":

        fc.target_vel_x = 0
        fc.target_vel_y = 0
        fc.target_vel_z = 0

        motors.set(0, 0, 0, 0)

        log("Drone landed successfully")

        break


    # ─────────────────────────────
    # MOTOR CONTROL
    # ─────────────────────────────
    fl, fr, rl, rr = fc.compute_motor_commands(
        roll, pitch, yaw,
        roll_rate, pitch_rate, yaw_rate,
        vel_x, vel_y, vel_z
    )

    logger.log(
        sim_time,
        state,

        x, y, z,

        roll, pitch, yaw,

        vel_x, vel_y, vel_z,

        fc.target_vel_x,
        fc.target_vel_y,
        fc.target_vel_z,

        marker_detected,
        mx,
        my,

        fl,
        fr,
        rl,
        rr
        )

    motors.set(fl, fr, rl, rr)
logger.close()
