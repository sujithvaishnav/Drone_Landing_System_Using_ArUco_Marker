import cv2
import numpy as np
from utils import log


class ArucoDetector:

    def __init__(self, robot, timestep):

        log("Initializing ArUco Detector")

        self.camera = robot.getDevice("camera")
        self.camera.enable(timestep)

        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()

        self.camera_pitch = robot.getDevice("camera pitch")
        self.camera_pitch.setPosition(1.2)

        dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_250
        )

        self.detector = cv2.aruco.ArucoDetector(dict)

    def detect(self, expected_id):

        image = self.camera.getImage()

        if image is None:
            return None, None, None

        img = np.frombuffer(
            image, np.uint8
        ).reshape((self.height, self.width, 4))

        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return None, None, None

        for i, marker_id in enumerate(ids):

            if marker_id[0] == expected_id:

                c = corners[i][0]

                cx = np.mean(c[:, 0])
                cy = np.mean(c[:, 1])

                error_x = (cx - self.width/2)/(self.width/2)
                error_y = (cy - self.height/2)/(self.height/2)

                log(f"Marker {marker_id[0]} detected")

                return marker_id[0], error_x, error_y

        return None, None, None
