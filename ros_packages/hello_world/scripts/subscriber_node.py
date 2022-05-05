#!/usr/bin/env python

# Importamos la libreria cliente ROS para Python
import rospy
# Importamos la declaracion de mensaje "String"
from std_msgs.msg import String

class MessageSubscriber():
    def __init__(self):
        # Inicializamos un nodo ROS
        rospy.init_node("subscriber_node", anonymous=False)

    def __callback(self, data):
        print(data)

    def subscribe(self, topic_name):
        rospy.loginfo_once("\nSubscribing to message")
        # Nos suscribimos al tema
        rospy.Subscriber(topic_name, String, self.__callback)
        # Mantenemos ejecutando el nodo hasta que se le de una senal de apagado
        rospy.spin()

def main():
    # Creamos una instancia de la clase
    message_subscriber=MessageSubscriber()
    # Nombre del tema al que nos suscribiremos
    topic_name="message"
    # Realizamos la tarea de suscribirnos
    message_subscriber.subscribe(topic_name=topic_name)

if __name__=="__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass