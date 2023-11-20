import cv2
import numpy as np
import yaml

def cm2px(coords):
    aruco = 0.025
    px = abs(coords[0][0] - coords[1][0])
    
    return aruco/px

def convert(point, factou):
    point_ref=(1001, 206)
    X_ref = point[0] - point_ref[0]
    Y_ref = point[1] - point_ref[1]
    
    return (X_ref*factou, Y_ref*factou)

# Lee la imagen
img = cv2.imread("memi.jpeg")

# Redimensionar la imagen (opcional)
nuevas_dimensiones = (int(img.shape[1]/2), int(img.shape[0]/2))
img = cv2.resize(img, nuevas_dimensiones)

# Convierte la imagen a escala de grises
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Carga el diccionario de marcadores
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

# Inicializa el detector de marcadores
parameters = cv2.aruco.DetectorParameters_create()

# Detecta los marcadores en la imagen
corners_aruco, ids_aruco, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

# Almacena las coordenadas de los marcadores ArUco en un array
coordinates_aruco = []

# Si se detectan marcadores ArUco
if ids_aruco is not None:
    # Dibuja los contornos y los ID de los marcadores en la imagen
    cv2.aruco.drawDetectedMarkers(img, corners_aruco, ids_aruco)

    # Extrae las coordenadas de los marcadores ArUco
    for marker_id, corner in zip(ids_aruco, corners_aruco):
        x, y = np.mean(corner[0], axis=0)
        coordinates_aruco.append((int(x), int(y)))

        # Agrega las coordenadas del punto de abajo a la izquierda
        x_bottom_left, y_bottom_left = corner[0][0]
        coordinates_aruco.append((int(x_bottom_left), int(y_bottom_left)))

        # Dibuja un círculo en la esquina inferior derecha (punto de arriba a la derecha)
        cv2.circle(img, (int(x), int(y)), 5, (255, 0, 0), -1)

        # Dibuja un círculo en la esquina inferior izquierda (punto de arriba a la izquierda)
        cv2.circle(img, (int(x_bottom_left), int(y_bottom_left)), 5, (255, 0, 0), -1)



# Detecta círculos en la imagen
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

# Almacena las coordenadas de los círculos en un array
coordinates_circles = []

# Si se detectan círculos
if circles is not None:
    circles = np.uint16(np.around(circles))
    
    # Extrae las coordenadas de los círculos en el mismo array
    for i in circles[0, :]:
        x, y = i[0], i[1]
        coordinates_circles.append((x, y))
        # Dibujar el círculo externo
        cv2.circle(img, (x, y), i[2], (0, 255, 0), 2)
        # Dibujar el centro del círculo
        cv2.circle(img, (x, y), 2, (0, 0, 255), 3)

# Guardar las coordenadas en archivos YAML separados
with open('pose_aruco.yaml', 'w') as yaml_file_aruco:
    yaml.dump(coordinates_aruco, yaml_file_aruco)

with open('pose_circles.yaml', 'w') as yaml_file_circles:
    yaml.dump(coordinates_circles, yaml_file_circles)

# Imprimir las coordenadas por separado
print("Coordenadas de Marcadores ArUco:")
for coord_aruco in coordinates_aruco:
    print(coord_aruco)

print("\nCoordenadas de Círculos:")
for coord_circles in coordinates_circles:
    pixeles_centros = convert(coord_circles,cm2px(coordinates_aruco))
    cv2.line(img,coord_circles, coordinates_aruco[1], (0, 0, 255), 2)
    print(pixeles_centros)
    
print(cm2px(coordinates_aruco))
# Mostrar la imagen con los marcadores ArUco y círculos detectados
cv2.imshow("Marcadores ArUco y Circulos detectados", img)
cv2.waitKey(0)
cv2.destroyAllWindows()