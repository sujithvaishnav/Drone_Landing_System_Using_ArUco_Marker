from controller import Robot

from sensors import Sensors
from motors import Motors
from aruco import ArucoDetector
from navigation import Navigation
from controller import FlightController

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


state = "TAKEOFF"


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


    target_x, target_y, target_z = nav.current()


    if state == "TAKEOFF":

        log(f"Takeoff Altitude {z:.2f}")

        error_z = target_z - z

        fc.target_vel_z = clamp(K_POS_P*error_z, -MAX_VEL_Z, MAX_VEL_Z)

        if abs(error_z) < VERTICAL_TOLERANCE:

            state = "NAVIGATE"

            log("Takeoff Complete")


    elif state == "NAVIGATE":

        error_x = target_x - x
        error_y = target_y - y

        distance = (error_x**2 + error_y**2)**0.5

        log(f"Waypoint Distance {distance:.2f}")

        if distance < HORIZONTAL_TOLERANCE:

            state = "SEARCH"

        fc.target_vel_x = clamp(K_POS_P*error_x, -MAX_VEL_XY, MAX_VEL_XY)
        fc.target_vel_y = clamp(K_POS_P*error_y, -MAX_VEL_XY, MAX_VEL_XY)


    elif state == "SEARCH":

        marker_id, mx, my = aruco.detect(TARGET_MARKER_IDS[nav.index])

        if marker_id is not None:

            state = "ALIGN"

        else:

            log("Marker not found")


    elif state == "ALIGN":

        marker_id, mx, my = aruco.detect(TARGET_MARKER_IDS[nav.index])

        if marker_id is None:

            state = "SEARCH"

        else:

            fc.target_vel_x = ALIGN_GAIN*(-my)
            fc.target_vel_y = ALIGN_GAIN*(mx)

            if abs(mx)<0.05 and abs(my)<0.05:

                state = "DESCEND"

                log("Aligned")


    elif state == "DESCEND":

        fc.target_vel_z = -0.3

        if z < LAND_HEIGHT:

            motors.stop()

            state = "DONE"

            log("Landing Complete")


    fl, fr, rl, rr = fc.compute_motor_commands(

        roll, pitch, yaw,
        roll_rate, pitch_rate, yaw_rate,
        vel_x, vel_y, vel_z
    )


    motors.set(fl, fr, rl, rr)
