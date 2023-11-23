import rospy
from geometry_msgs.msg import Point
import json

def leer_puntos_desde_json(ruta_json):
    with open(ruta_json, 'r') as archivo:
        datos = json.load(archivo)
    return datos['puntos']  # Supongo que el JSON tiene una clave 'puntos' que contiene la lista de puntos

def publicar_puntos_desde_json(ruta_json):
    puntos_detectados = leer_puntos_desde_json(ruta_json)

    rospy.init_node('publicador_puntos', anonymous=True)
    pub = rospy.Publisher('/puntos_interes', Point, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    for punto in puntos_detectados:
        msg = Point()
        msg.x = punto[0]
        msg.y = punto[1]
        msg.z = 0  # Si tus puntos son 2D
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    # Especifica la ruta de tu archivo JSON
    ruta_json = 'ruta/a/tu/archivo.json'

    # Llama a la función para publicar puntos desde el archivo JSON
    publicar_puntos_desde_json(ruta_json)
    
