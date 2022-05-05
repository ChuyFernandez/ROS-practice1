#!/usr/bin/env python

# Importamos la libreria cliente ROS para Python
import rospy
import actionlib
# Importamos la declaracion de servicio "CallMoveCubeAS" y los mensajes de solicitud y respuesta
from practice_msgs.srv import CallMoveCubeAS, CallMoveCubeASRequest, CallMoveCubeASResponse
# Importamos la declaracion de accion "MoveCubeAction" y los mensajes de objetivo, retroalimentacion y resultado
from practice_msgs.msg import MoveCubeAction, MoveCubeGoal, MoveCubeFeedback, MoveCubeResult 
# Importamos la declaracion de mensaje "PoseOriginTarget" para el mensaje objetivo de la accion
# (se trata de un mensaje personalizado)
from practice_msgs.msg import PoseOriginTarget

class CallMoveCubeASService():
    def __init__(self):
        # Inicializamos un nodo ROS
        rospy.init_node("call_move_cube_as_service_node", anonymous=False)

    def createSimpleActionClient(self, action_name):
        # Creamos un cliente de accion simple
        self.__ac=actionlib.SimpleActionClient(action_name, MoveCubeAction)

    def createService(self, srv_name):
        # Creamos el servicio
        self.__srv=rospy.Service(srv_name, CallMoveCubeAS, self.__handler)
        rospy.loginfo_once("\nService 'call_move_cube_as' created")
        # Mantenemos ejecutando el nodo hasta que se le de una senal de apagado
        rospy.spin()
    
    def __sendGoal(self, goal):
        # Esperamos hasta que el servidor de acciones se haya iniciado
        self.__ac.wait_for_server()
        # Enviamos el objeto al servidor de acciones
        self.__ac.send_goal(goal)
        rospy.loginfo("Goal has been sent to the action server")

    # Recibimos una solicitud y devolvemos una respuesta
    def __handler(self, req):
        # Recibimos el objetivo de la solicitud
        goal=req
        # Enviamos el objeto al servidor de acciones
        self.__sendGoal(goal)
        # Devolvemos una respuesta de que ya se ha llamado al servidor de accion
        res=CallMoveCubeASResponse(success=True, message="Action server called")
        return res

def main():
    # Creamos una instancia de la clase
    call_move_cube_as_service=CallMoveCubeASService()
    # Creamos el cliente de accion simple
    call_move_cube_as_service.createSimpleActionClient(action_name="move_cube")
    # Creamos el servicio
    call_move_cube_as_service.createService(srv_name="call_move_cube_as")

if __name__=='__main__':
    try:
        main()
    except rospy.ServiceException as exception:
        pass

