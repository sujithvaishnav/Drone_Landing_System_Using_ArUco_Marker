from utils import log


class Motors:

    def __init__(self, robot):

        log("Initializing Motors")

        self.fl = robot.getDevice("front left propeller")
        self.fr = robot.getDevice("front right propeller")
        self.rl = robot.getDevice("rear left propeller")
        self.rr = robot.getDevice("rear right propeller")

        motors = [self.fl, self.fr, self.rl, self.rr]

        for m in motors:

            m.setPosition(float('inf'))
            m.setVelocity(0)

    def set(self, fl, fr, rl, rr):

        self.fl.setVelocity(fl)
        self.fr.setVelocity(-fr)
        self.rl.setVelocity(-rl)
        self.rr.setVelocity(rr)

    def stop(self):

        log("Stopping motors")

        self.set(0, 0, 0, 0)
