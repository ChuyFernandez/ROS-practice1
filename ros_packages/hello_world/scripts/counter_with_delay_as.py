#!/usr/bin/env python

# Importamos la libreria cliente ROS para Python
import rospy
import actionlib
# Importamos la declaracion de accion "CounterWithDelayAction" y los mensajes de retroalimentacion y resultado
from hello_world_msgs.msg import CounterWithDelayAction, CounterWithDelayFeedback, CounterWithDelayResult 

class CounterWithDelayAS():
    def __init__(self):
        # Inicializamos un nodo ROS
        rospy.init_node("counter_with_delay_as", anonymous=False)
        self.__feedback=CounterWithDelayFeedback()
        self.__result=CounterWithDelayResult()
    
    def createSimpleActionServer(self, action_name):
        # Creamos el servidor de accion simple
        self.__as=actionlib.SimpleActionServer(action_name, CounterWithDelayAction, execute_cb=self.__execute_cb, auto_start=False)
        self.__as.start()
        rospy.loginfo_once("\nSimple action server created")
        # Mantenemos ejecutando el nodo hasta que se le de una senal de apagado
        rospy.spin()
    
    # Recibimos un objetivo
    def __execute_cb(self, goal):
        rate_second=1
        success=True
        self.__feedback.counts_elapsed=0
        for i in range(goal.num_counts):
            # Se comprueba si el cliente ha solicitado la preferencia
            if self.__as.is_preempt_requested():
                self.__as.set_preempt()
                success=False
                break
            self.__feedback.counts_elapsed=i
            self.__as.publish_feedback(self.__feedback)
            # Dormir por 1 segundo
            rospy.sleep(rate_second)
        if success:
            self.__result.result_message="Succesfully completed counting"
            self.__as.set_succeeded(self.__result)

def main():
    # Creamos una instancia de la clase
    counter_with_delay_as=CounterWithDelayAS()
    # Creamos el servicio
    counter_with_delay_as.createSimpleActionServer(action_name="counter")

if __name__=='__main__':
    main()

