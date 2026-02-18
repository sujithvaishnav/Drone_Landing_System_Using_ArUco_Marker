from utils import log


class Sensors:

    def __init__(self, robot, timestep):

        log("Initializing Sensors")

        self.imu = robot.getDevice("inertial unit")
        self.imu.enable(timestep)

        self.gps = robot.getDevice("gps")
        self.gps.enable(timestep)

        self.gyro = robot.getDevice("gyro")
        self.gyro.enable(timestep)

        self.prev_x = 0
        self.prev_y = 0
        self.prev_z = 0

    def read(self, dt):

        roll, pitch, yaw = self.imu.getRollPitchYaw()

        roll_rate, pitch_rate, yaw_rate = self.gyro.getValues()

        x, y, z = self.gps.getValues()

        vel_x = (x - self.prev_x) / dt
        vel_y = (y - self.prev_y) / dt
        vel_z = (z - self.prev_z) / dt

        self.prev_x, self.prev_y, self.prev_z = x, y, z

        return (
            roll, pitch, yaw,
            roll_rate, pitch_rate, yaw_rate,
            x, y, z,
            vel_x, vel_y, vel_z
        )
