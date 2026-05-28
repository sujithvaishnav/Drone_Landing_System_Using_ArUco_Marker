from config import WAYPOINTS
from utils import log


class Navigation:

    def __init__(self):

        self.index = 0

        log("Navigation initialized")

    def current(self):

        return WAYPOINTS[self.index]

    def next(self):

        self.index += 1

        if self.index >= len(WAYPOINTS):

            log("Mission complete")

            return False

        log(f"Moving to waypoint {self.index+1}")

        return True
