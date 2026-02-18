import numpy as np

from config import *
from utils import clamp, normalize_angle


class FlightController:

    def __init__(self):

        self.target_vel_x = 0
        self.target_vel_y = 0
        self.target_vel_z = 0

        self.yaw_target = 0
        self.yaw_initialized = False

    def compute_motor_commands(
        self,
        roll, pitch, yaw,
        roll_rate, pitch_rate, yaw_rate,
        vel_x, vel_y, vel_z
    ):

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        vel_error_x = self.target_vel_x - vel_x
        vel_error_y = self.target_vel_y - vel_y
        vel_error_z = self.target_vel_z - vel_z

        body_vel_error_x = cos_yaw*vel_error_x + sin_yaw*vel_error_y
        body_vel_error_y = -sin_yaw*vel_error_x + cos_yaw*vel_error_y

        roll_input = K_ROLL_P*roll + roll_rate
        pitch_input = K_PITCH_P*pitch + pitch_rate

        roll_input += K_VEL_XY_P*body_vel_error_y
        pitch_input -= K_VEL_XY_P*body_vel_error_x

        roll_input = clamp(
            roll_input,
            -MAX_TILT*K_ROLL_P,
            MAX_TILT*K_ROLL_P
        )

        pitch_input = clamp(
            pitch_input,
            -MAX_TILT*K_PITCH_P,
            MAX_TILT*K_PITCH_P
        )

        vertical_input = K_VEL_Z_P*vel_error_z

        yaw_error = normalize_angle(self.yaw_target - yaw)

        yaw_input = K_YAW_P*yaw_error - K_YAW_D*yaw_rate

        fl = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input
        fr = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input
        rl = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
        rr = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input

        return (
            clamp(fl,0,100),
            clamp(fr,0,100),
            clamp(rl,0,100),
            clamp(rr,0,100)
        )
