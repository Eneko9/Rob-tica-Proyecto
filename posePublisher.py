import rospy
from geometry_msgs.msg import Point, PoseArray, Pose
from copy import deepcopy
import json
import cv2
import numpy as np

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

def leer_puntos_desde_json(ruta_json):
    with open(ruta_json, 'r') as archivo:
        datos = json.load(archivo)
    return datos

def publicar_puntos_desde_json(ruta_json):
    puntos_detectados = leer_puntos_desde_json(ruta_json)

    rospy.init_node('publicador_puntos', anonymous=True)
    pub = rospy.Publisher('/puntos_interes', PoseArray, queue_size=10)
    rospy.sleep(2)
    rate = rospy.Rate(10)  # 10 Hz

    puntos_array = PoseArray() # Nuevo array para almacenar los puntos

    for punto in puntos_detectados:
        msg = Pose()
        msg.position.x = punto[0]
        msg.position.y = punto[1]
        msg.position.z = 0  # Si tus puntos son 2D
        puntos_array.poses.append(deepcopy(msg))
        
    pub.publish(puntos_array)
    rate.sleep()

def getPoints():
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
    pixeles_centros = []
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        
        for i in circles[0, :]:
            x, y = i[0], i[1]
            coordinates_circles.append((x, y))
            cv2.circle(frame, (x, y), i[2], (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)

    for coord_circles in coordinates_circles:
        if coordinates_aruco:  # Check if coordinates_aruco is not empty
            pixeles_centros.append(convert(coord_circles, cm2px(coordinates_aruco)))
            #cv2.line(frame, coord_circles, coordinates_aruco[1], (0, 0, 255), 2)
            
    with open('circles_norm.json', 'w') as json_file_circles:
        json.dump(pixeles_centros, json_file_circles, cls=NpEncoder)
            
    cv2.imwrite("peru.jpg",frame)
if __name__ == '__main__':
    # Especifica la ruta de tu archivo JSON
    ruta_circles = 'circles_norm.json'
    getPoints()
    # Llama a la función para publicar puntos desde el archivo JSON
    publicar_puntos_desde_json(ruta_circles)
    
