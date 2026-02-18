import cv2
import numpy as np

aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

ids = [10, 25, 42]

marker_size = 400
padding = 100

total_size = marker_size + 2 * padding

for marker_id in ids:

    marker = cv2.aruco.generateImageMarker(
        aruco,
        marker_id,
        marker_size
    )

    padded = np.ones((total_size, total_size), dtype=np.uint8) * 255

    padded[
        padding:padding+marker_size,
        padding:padding+marker_size
    ] = marker

    cv2.imwrite(f"aruco_{marker_id}.png", padded)

print("Markers generated with white padding")
