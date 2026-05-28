import csv
import os


class FlightLogger:
    def __init__(self, filename="logs/flight_log.csv"):

        os.makedirs("logs", exist_ok=True)

        self.file = open(filename, mode="w", newline="")
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            "time",
            "state",

            "x", "y", "z",

            "roll", "pitch", "yaw",

            "vel_x", "vel_y", "vel_z",

            "target_vel_x",
            "target_vel_y",
            "target_vel_z",

            "marker_detected",
            "marker_x",
            "marker_y",

            "motor_fl",
            "motor_fr",
            "motor_rl",
            "motor_rr"
        ])

    def log(
        self,
        time,
        state,

        x, y, z,

        roll, pitch, yaw,

        vel_x, vel_y, vel_z,

        target_vel_x,
        target_vel_y,
        target_vel_z,

        marker_detected,
        marker_x,
        marker_y,

        fl, fr, rl, rr
    ):

        self.writer.writerow([
            time,
            state,

            x, y, z,

            roll, pitch, yaw,

            vel_x, vel_y, vel_z,

            target_vel_x,
            target_vel_y,
            target_vel_z,

            marker_detected,
            marker_x,
            marker_y,

            fl,
            fr,
            rl,
            rr
        ])

    def close(self):
        self.file.close()