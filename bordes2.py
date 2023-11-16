import cv2
import numpy as np
import yaml

# Lee la imagen
img = cv2.imread("yes2.png")

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
    print(coord_circles)

# Mostrar la imagen con los marcadores ArUco y círculos detectados
cv2.imshow("Marcadores ArUco y Circulos detectados", img)
cv2.waitKey(0)
cv2.destroyAllWindows()