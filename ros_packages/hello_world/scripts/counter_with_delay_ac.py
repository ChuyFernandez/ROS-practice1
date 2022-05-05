#!/usr/bin/env python

# Importamos este modulo que nos permite obtener la lista de argumentos de la linea de comandos
import sys
# Importamos la libreria cliente ROS para Python
import rospy
import actionlib
# Importamos la declaracion de accion "CounterWithDelayAction" y los mensajes de retroalimentacion y resultado
from hello_world_msgs.msg import CounterWithDelayAction, CounterWithDelayGoal, CounterWithDelayResult 

class CounterWithDelayAC():
    def __init__(self):
        # Inicializamos un nodo ROS
        rospy.init_node("counter_with_delay_ac", anonymous=False)
    
    def createSimpleActionClient(self, action_name):
        # Creamos un cliente de accion simple
        self.__ac=actionlib.SimpleActionClient(action_name, CounterWithDelayAction)

    def sendGoal(self, num_counts):
         # Esperamos hasta que el servidor de acciones se haya iniciado
        self.__ac.wait_for_server()
        goal=CounterWithDelayGoal(num_counts=num_counts)
        # Enviamos el objeto al servidor de acciones
        self.__ac.send_goal(goal)
        rospy.loginfo("Goal has been sent to the action server")

        # NOTA: Aqui es donde podemos incluir la funcionalidad de bloqueo en las acciones
        # Esperamos a que el servidor termine de realizar la accion
        #self.__ac.wait_for_server()

        # Podemos hacer otras cosas mientras el servidor de acciones procesa la accion
        # ...

        self.__ac.wait_for_result()
        
        return self.__ac.get_result()

def main():
    try:
        # Creamos una instancia de la clase
        counter_with_delay_ac=CounterWithDelayAC()
        # Creamos el cliente
        counter_with_delay_ac.createSimpleActionClient(action_name="counter")
        # Enviamos un objetivo
        num_counts=3 if len(sys.argv)==1 else int(sys.argv[1])
        result=counter_with_delay_ac.sendGoal(num_counts)
        print(result.result_message)

        # Recordemos que podemos ver los mensajes de retroalimentacion que se publican
        # mientras se procesa el objetivo en el lado del servidor de acciones, esto con el
        # siguiente comando:
        #   rostopic echo /counter_with_delay_as/feedback
        # Esto solo cuando se esta procesando el objetivo, ya que una vez finalizado, ya no se
        # publicara mas en este tema.
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")

if __name__=='__main__':
    main()