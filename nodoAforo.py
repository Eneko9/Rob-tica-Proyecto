import cv2
import numpy as np
from ultralytics import YOLO
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Bool
model = YOLO('src/model/yolov8n.pt')

# Variables globales para gestionar los puntos
point1, point2 = None, None
drawing = False
line_drawn = False
slope = None
region_mask = None
paused = False

STOP = False

# Diccionario para realizar seguimiento de personas
person_tracker = {}

# Contador para asignar ID único a cada persona
person_id_counter = 1

# Umbral de solapamiento para considerar que es la misma persona
overlap_threshold = 0.1

# Función para calcular la superposición entre dos cuadros delimitadores
def calculate_overlap(box1, box2):
    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2
    area1 = w1 * h1
    area2 = w2 * h2

    x_left = max(x1, x2)
    y_top = max(y1, y2)
    x_right = min(x1 + w1, x2 + w2)
    y_bottom = min(y1 + h1, y2 + h2)

    if x_right < x_left or y_bottom < y_top:
        return 0.0

    intersection_area = (x_right - x_left) * (y_bottom - y_top)
    union_area = area1 + area2 - intersection_area
    overlap = intersection_area / union_area
    return overlap

def draw_line(event, x, y, flags, param):
    global point1, point2, drawing, line_drawn, slope, region_mask

    if event == cv2.EVENT_LBUTTONDOWN:
        if not line_drawn:
            if point1 is None:
                point1 = (x, y)
                drawing = True
            elif point2 is None:
                point2 = (x, y)
                drawing = False
                line_drawn = True
                # Calcular la pendiente de la línea
                if point2[0] - point1[0] != 0:
                    slope = (point2[1] - point1[1]) / (point2[0] - point1[0])

    if event == cv2.EVENT_RBUTTONDOWN:
        point1, point2 = None, None
        line_drawn = False
        slope = None
        region_mask = None

def show_movement_histogram(frame_shape):
    global person_tracker

    movement_data = []  # Lista para almacenar los datos de movimiento

    for tracked_positions in person_tracker.values():
        for x, y, _, _ in tracked_positions:
            movement_data.append((x, y))

    if movement_data:
        movement_data = np.array(movement_data)
        plt.figure(figsize=(frame_shape[1] / 100, frame_shape[0] / 100))  # Ajustar el tamaño en base al ancho y alto del frame
        plt.hist2d(movement_data[:, 0], movement_data[:, 1], bins=(50, 50), cmap='gray_r')
        plt.colorbar(label='Frecuencia de Movimiento')
        plt.title('Histograma de Movimiento')
        plt.xlabel('Coordenada X')
        plt.ylabel('Coordenada Y')
        plt.grid(visible=True, linestyle='--', alpha=0.5)
        plt.gca().set_facecolor('black')  # Fondo negro
        plt.gca().tick_params(axis='x', colors='white')  # Color de los ejes X
        plt.gca().tick_params(axis='y', colors='white')  # Color de los ejes Y
        plt.gca().xaxis.label.set_color('white')  # Color de la etiqueta del eje X
        plt.gca().yaxis.label.set_color('white')  # Color de la etiqueta del eje Y
        plt.show()

def publicarStop():
    rospy.init_node('publicador_stop_aforo', anonymous=True)
    pub = rospy.Publisher('/parada_aforo', Bool, queue_size=10)
    rospy.sleep(2)
    rate = rospy.Rate(10)  # 10 Hz

    pub.publish(STOP)
    rate.sleep()
    
cap = cv2.VideoCapture(8)

colors = {}

while True:
    # Leer el primer fotograma
    ret, frame = cap.read()
    
    # Mostrar el primer fotograma aunque el video comience en pausa
    cv2.imshow('Video', frame)
    cv2.setMouseCallback('Video', draw_line)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('o') and paused:  # Reanudar la reproducción con una sola pulsación de 'o'
        paused = False

    if key == ord('p') and not paused:  # Pausar la reproducción con una sola pulsación de 'p'
        paused = True

    if key == ord('h'):  # Mostrar el mapa de calor cuando se presiona la tecla 'h'
        show_movement_histogram(frame.shape)

    if ret:  # Verificar si se ha leído un fotograma correctamente
        
        if point1 is not None and point2 is not None:
            # Calcular los puntos de intersección de la línea con los bordes del marco
            x1, y1 = point1
            x2, y2 = point2
            x1_new = 0
            y1_new = int(y1 - (x1 - x1_new) * slope)
            x2_new = frame.shape[1]
            y2_new = int(y2 - (x2 - x2_new) * slope)

            # Completar la línea hacia los bordes de la imagen
            cv2.line(frame, (x1_new, y1_new), (x2_new, y2_new), (0, 255, 0), 2)

            # Crear una máscara del mismo tamaño que el marco (255 en la región por encima de la línea, 0 en la región por debajo)
            mask = np.zeros_like(frame)
            cv2.fillPoly(mask, np.array([[(0, 0), (0, y1_new), (x1, y1), (x2, y2), (x2_new, y2_new), (frame.shape[1], y2_new), (frame.shape[1], 0)]], np.int32), (255, 255, 255))

            # Invertir la máscara para bloquear la región superior
            inverted_mask = cv2.bitwise_not(mask)

            # Obtener los resultados de la detección de personas en la región que no está bloqueada por la máscara
            results = model(source=cv2.bitwise_and(frame, inverted_mask), show=True, conf=0.4, save=False, classes=0)

            num_predictions = results[0].boxes.shape[0]
            text = f"Total de predicciones: {num_predictions}"

            if num_predictions <= 2:
                cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
            else:
                cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                STOP = True
                publicarStop()

            detected_people = []

            for res in results:
                boxes = res.boxes.cpu().numpy()
                xyxys = boxes.xyxy
                for xyxy in xyxys:
                    x1, y1, x2, y2 = map(int, xyxy)
                    bbox = (x1, y1, x2 - x1, y2 - y1)
                    matched_person_id = None

                    for person_id, tracked_positions in person_tracker.items():
                        last_position = tracked_positions[-1]
                        overlap = calculate_overlap(bbox, last_position)
                        if overlap >= overlap_threshold:
                            matched_person_id = person_id
                            break

                    if matched_person_id is None:
                        matched_person_id = person_id_counter
                        person_id_counter += 1

                    if matched_person_id not in person_tracker:
                        person_tracker[matched_person_id] = [bbox]
                    else:
                        person_tracker[matched_person_id].append(bbox)

                    detected_people.append(matched_person_id)

                    if matched_person_id not in colors:
                        # Asignar un color aleatorio al ID si no tiene uno asignado previamente
                        colors[matched_person_id] = np.random.randint(0, 255, size=3).tolist()

                    # Dibujar el rectángulo con el color asignado al ID
                    color = colors[matched_person_id]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, f'ID: {matched_person_id}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Eliminar personas no detectadas en este fotograma
            disappeared_people = set(person_tracker.keys()) - set(detected_people)
            for person_id in disappeared_people:
                del person_tracker[person_id]

        # Mostrar el video con la línea dibujada y la información de detección
        if not paused:
            cv2.imshow('Video', frame)
        
    if key == 27 or not ret:  # Salir con la tecla 'Esc' o si no se puede leer más fotogramas
        break

    # Leer el siguiente fotograma si el video no está pausado
    if not paused:
        ret, frame = cap.read()

cap.release()

cv2.destroyAllWindows()
