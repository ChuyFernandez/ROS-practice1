#!/usr/bin/env python

# Importamos este modulo que nos permite obtener la lista de argumentos de la linea de comandos
import sys
# Importamos la libreria cliente ROS para Python
import rospy
import actionlib
# Importamos la declaracion de accion "MoveCubeAction" y los mensajes de objetivo, retroalimentacion y resultado
from practice_msgs.msg import MoveCubeAction, MoveCubeGoal, MoveCubeFeedback, MoveCubeResult 
# Importamos la declaracion de mensaje "PoseOriginTarget" para el mensaje objetivo de la accion
# (se trata de un mensaje personalizado)
from practice_msgs.msg import PoseOriginTarget
# Importamos otras declaraciones de mensaje ya predefinidas
from geometry_msgs.msg import Pose, Point, Quaternion

"""
rosmsg show geometry_msgs/Pose
    geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
    geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
"""

class MoveCubeAC():
    def __init__(self):
        # Inicializamos un nodo ROS
        rospy.init_node("move_cube_ac_node", anonymous=False)
        # Indicamos el nombre del tema de retroalimentacion
        # Inicia con el nombre del servidor de accion
        self.__feedback_topic_name="/move_cube/feedback"
    
    def createSimpleActionClient(self, action_name):
        # Creamos un cliente de accion simple
        self.__ac=actionlib.SimpleActionClient(action_name, MoveCubeAction)

    def sendGoal(self):
        # Esperamos hasta que el servidor de acciones se haya iniciado
        self.__ac.wait_for_server()
        # Creamos un mensaje de objetivo
        goal=MoveCubeGoal(
            PoseOriginTarget(
                pose_origin=Pose(
                    Point(0,0,0),
                    Quaternion(0,0,0,0)
                ),
                pose_target=Pose(
                    Point(10,0,0),
                    Quaternion(0,0,0,0)
                )
            )
        )
        # Enviamos el objeto al servidor de acciones
        self.__ac.send_goal(goal)
        rospy.loginfo("Goal has been sent to the action server")

        # NOTA: Aqui es donde podemos incluir la funcionalidad de bloqueo en las acciones
        # Esperamos a que el servidor termine de realizar la accion
        #self.__ac.wait_for_server()

        # Podemos hacer otras cosas mientras el servidor de acciones procesa la accion
        # ...

        # Nos suscribimos al tema de retroalimentacion
        self.__subscribe_feeback()
        # La linea de codigo anterior es bloqueante si se usa "rospy.spin()", por lo que
        # para poder continuar necesitamos presionar Ctrl + c
        print("After subscriber")

        # Esperamos a que termine el servidor de accion antes de devolver el resultado
        self.__ac.wait_for_result()
        
        # Devolvemos el resultado ya finalizada la accion
        return self.__ac.get_result()

    def __subscribe_feeback(self):
        rospy.loginfo_once("\nSubscribing to feedback message")
        # Nos suscribimos al tema
        rospy.Subscriber(self.__feedback_topic_name, MoveCubeFeedback, self.__callback_feedback)
        # Mantenemos ejecutando el nodo hasta que se le de una senal de apagado
        #rospy.spin()

    def __callback_feedback(self, data):
        print(data)

def main():
    try:
        # Creamos una instancia de la clase
        move_cube_ac=MoveCubeAC()
        # Creamos el cliente de accion simple
        move_cube_ac.createSimpleActionClient(action_name="move_cube")
        # Enviamos un objetivo
        # sys.argv[1]
        result=move_cube_ac.sendGoal()
        print("Result message: "+result.result_message)

        # Recordemos que podemos ver los mensajes de retroalimentacion que se publican
        # mientras se procesa el objetivo en el lado del servidor de acciones, esto con el
        # siguiente comando:
        #   rostopic echo /move_cube/feedback
        # Esto solo cuando se esta procesando el objetivo, ya que una vez finalizado, ya no se
        # publicara mas en este tema.
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")

if __name__=='__main__':
    main()