import cv2
import numpy as np

# Inicializar la captura de video desde la cámara
cap = cv2.VideoCapture(2)  # El argumento 0 indica la cámara principal (puede variar dependiendo de tu configuración)

while True:
    # Capturar un fotograma de la cámara
    ret, frame = cap.read()

    if not ret:
        break

    # Redimensionar el fotograma capturado
    #frame = cv2.pyrDown(frame)
    
    x1, y1 = 248, 130
    x2, y2 = 457, 280

    # Recortar la imagen
    imagen_recortada = frame[y1:y2, x1:x2]

    ret, thresh = cv2.threshold(cv2.cvtColor(imagen_recortada, cv2.COLOR_BGR2GRAY), 170, 255, 1)
    contours, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    black = np.zeros_like(imagen_recortada)

    for cnt in contours:
        epsilon = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        hull = cv2.convexHull(cnt)
        area = cv2.contourArea(cnt)
        cv2.drawContours(black, [cnt], -1, (0, 255, 0), 2)
        # cv2.drawContours(black, [approx], -1, (255, 255, 0), 2)
        # cv2.drawContours(black, [hull], -1, (0, 0, 255), 2)

    cv2.imshow("bordes", black)

    if cv2.waitKey(1) & 0xFF == 27:  # Presiona la tecla 'Esc' para salir
        break

cap.release()
cv2.destroyAllWindows()
