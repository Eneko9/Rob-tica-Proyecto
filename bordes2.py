"""import cv2
import numpy as np

img = cv2.imread(r"C:\Master MUCSI\computerVision\Vision-Robotica\cap.jpeg")
# Inicializar la captura de video desde la cámara

# Redimensionar el fotograma capturado
#frame = cv2.pyrDown(frame)

x1, y1 = 248, 130
x2, y2 = 457, 280

# Recortar la imagen
imagen_recortada = img[y1:y2, x1:x2]

nuevas_dimensiones = (int(img.shape[1]/2), int(img.shape[0]/2))

# Redimensionar la imagen
img = cv2.resize(img, nuevas_dimensiones)

ret, thresh = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 5, 255, 1)
contours, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
black = np.zeros_like(img)

for cnt in contours:
    epsilon = 0.02 * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    hull = cv2.convexHull(cnt)
    area = cv2.contourArea(cnt)
    cv2.drawContours(black, [cnt], -1, (0, 255, 0), 2)
    # cv2.drawContours(black, [approx], -1, (255, 255, 0), 2)
    # cv2.drawContours(black, [hull], -1, (0, 0, 255), 2)

cv2.imshow("bordes", black)
cv2.imshow("img", img)


cv2.waitKey(0)
cv2.destroyAllWindows()


"""


""""import cv2
import numpy as np

img = cv2.imread(r"C:\Master MUCSI\computerVision\Vision-Robotica\cap.jpeg")

x1, y1 = 248, 130
x2, y2 = 457, 280

imagen_recortada = img[y1:y2, x1:x2]

nuevas_dimensiones = (int(img.shape[1]/2), int(img.shape[0]/2))
img = cv2.resize(img, nuevas_dimensiones)

ret, thresh = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 5, 255, 1)
contours, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
black = np.zeros_like(img)

# ...

# Almacenar las coordenadas de los puntos más pequeños
min_point_coordinates = []

for cnt in contours:
    epsilon = 0.02 * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    hull = cv2.convexHull(cnt)
    area = cv2.contourArea(cnt)

    if area < 50:  # Puedes ajustar este umbral según tus necesidades
        # Obtener las coordenadas del rectángulo delimitador
        x, y, w, h = cv2.boundingRect(cnt)
        min_point_coordinates.append((x, y))

        # Dibujar un círculo en el punto más pequeño
        cv2.circle(black, (x, y), 5, (0, 0, 255), -1)

    cv2.drawContours(black, [cnt], -1, (0, 255, 0), 2)

# ...

# Imprimir las coordenadas de los puntos más pequeños
for coord in min_point_coordinates:
    print("Coordenadas del punto más pequeño:", coord)

cv2.imshow("bordes", black)
cv2.imshow("img", img)

cv2.waitKey(0)
cv2.destroyAllWindows()
"""
import cv2
import numpy as np
import yaml

img = cv2.imread(r"C:\Master MUCSI\computerVision\Vision-Robotica\cap.jpeg")

x1, y1 = 248, 130
x2, y2 = 457, 280

imagen_recortada = img[y1:y2, x1:x2]

nuevas_dimensiones = (int(img.shape[1]/2), int(img.shape[0]/2))
img = cv2.resize(img, nuevas_dimensiones)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray = cv2.medianBlur(gray, 5)

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

if circles is not None:
    circles = np.uint16(np.around(circles))
    
    # Almacenar las coordenadas en un array
    coordinates_array = []

    for i in circles[0, :]:
        x, y = i[0], i[1]
        coordinates_array.append((x, y))

        # Dibujar el círculo externo
        cv2.circle(img, (x, y), i[2], (0, 255, 0), 2)
        # Dibujar el centro del círculo
        cv2.circle(img, (x, y), 2, (0, 0, 255), 3)

    # Guardar las coordenadas en un archivo YAML
    with open('pose.yaml', 'w') as yaml_file:
        yaml.dump(coordinates_array, yaml_file)

cv2.imshow("Círculos detectados", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
