#!/usr/bin/env python

# Importamos este modulo que nos permite obtener la lista de argumentos de la linea de comandos
import sys
# Importamos la libreria cliente ROS para Python
import rospy
# Importamos la declaracion de mensaje "String"
from std_msgs.msg import String

class MessagePublisher():
    def __init__(self, topic_name):
        # Inicializamos un nodo ROS
        rospy.init_node("publisher_node", anonymous=False)
        # Creamos una instancia para publicar
        self.__publisher=rospy.Publisher(topic_name, String, queue_size=10)

    def __publish(self, message):
        # Publicamos el mensaje 
        self.__publisher.publish(String(message))
        rospy.loginfo_once("\nPublishing message")

    # Definimos la manera de publicar
    def publish(self, message, rate_second=0):
        # Publicar una vez o a una cierta tasa 
        if rate_second==0:
            self.__publish(message)
        else:
            # rate=rospy.Rate(10) # En Hz
            while not rospy.is_shutdown():
                self.__publish(message)
                rospy.sleep(rate_second)
                # rate.sleep()

def main():
    # Creamos una instancia de la clase
    message_publisher=MessagePublisher(topic_name="message")
    # Guardamos el valor de la entrada al ejecutar el script (si es que el usuario especifico uno)
    message="No hay mensaje :(" if len(sys.argv)==1 else sys.argv[1]
    rate_second=0 if len(sys.argv)<=2 else float(sys.argv[2])
    # Realizamos la tarea de publicar
    message_publisher.publish(message=message, rate_second=rate_second)

if __name__=="__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass