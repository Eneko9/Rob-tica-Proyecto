import cv2
import numpy as np
import json

class NpEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super(NpEncoder, self).default(obj)
    
def cm2px(coords):
    aruco = 0.025
    px = abs(coords[0][0] - coords[1][0])
    
    return aruco / px if px != 0 else 0

def convert(point, factou):
    point_ref = (1001, 206)
    X_ref = point[0] - point_ref[0]
    Y_ref = point[1] - point_ref[1]
    
    return (X_ref * factou, Y_ref * factou)


cap = cv2.VideoCapture(2)
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters_create()

ret, frame = cap.read()
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

corners_aruco, ids_aruco, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
coordinates_aruco = []

if ids_aruco is not None:
    cv2.aruco.drawDetectedMarkers(frame, corners_aruco, ids_aruco)

    for marker_id, corner in zip(ids_aruco, corners_aruco):
        x, y = np.mean(corner[0], axis=0)
        coordinates_aruco.append((int(x), int(y)))

        x_bottom_left, y_bottom_left = corner[0][0]
        coordinates_aruco.append((int(x_bottom_left), int(y_bottom_left)))

        cv2.circle(frame, (int(x), int(y)), 5, (255, 0, 0), -1)
        cv2.circle(frame, (int(x_bottom_left), int(y_bottom_left)), 5, (255, 0, 0), -1)

circles = cv2.HoughCircles(
    gray,
    cv2.HOUGH_GRADIENT,
    dp=1,
    minDist=50,
    param1=50,
    param2=30,
    minRadius=5,
    maxRadius=30
)

coordinates_circles = []

if circles is not None:
    circles = np.uint16(np.around(circles))
    
    for i in circles[0, :]:
        x, y = i[0], i[1]
        coordinates_circles.append((x, y))
        cv2.circle(frame, (x, y), i[2], (0, 255, 0), 2)
        cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)

with open('pose_aruco.json', 'w') as json_file_aruco:
    json.dump(coordinates_aruco, json_file_aruco,cls=NpEncoder)

with open('pose_circles.json', 'w') as json_file_circles:
    json.dump(coordinates_circles, json_file_circles, cls=NpEncoder)

for coord_circles in coordinates_circles:
    if coordinates_aruco:  # Check if coordinates_aruco is not empty
        pixeles_centros = convert(coord_circles, cm2px(coordinates_aruco))
        #cv2.line(frame, coord_circles, coordinates_aruco[1], (0, 0, 255), 2)

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
corners_aruco, ids_aruco, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
coordinates_aruco = []

if ids_aruco is not None:
    cv2.aruco.drawDetectedMarkers(frame, corners_aruco, ids_aruco)

    for marker_id, corner in zip(ids_aruco, corners_aruco):
        x, y = np.mean(corner[0], axis=0)
        coordinates_aruco.append((int(x), int(y)))

        x_bottom_left, y_bottom_left = corner[0][0]
        coordinates_aruco.append((int(x_bottom_left), int(y_bottom_left)))

        cv2.circle(frame, (int(x), int(y)), 5, (255, 0, 0), -1)
        cv2.circle(frame, (int(x_bottom_left), int(y_bottom_left)), 5, (255, 0, 0), -1)

cv2.imshow('img', frame)
cv2.waitKey()
cv2.destroyAllWindows()
