#!/usr/bin/env python

# importamos este modulo que nos permite obtener la lista de argumentos de la linea de comandos
import sys
# Importamos la libreria cliente ROS para Python
import rospy
import actionlib
# Importamos la declaracion de servicio "CallMoveCubeAS" y los mensajes de solicitud y respuesta
from practice_msgs.srv import CallMoveCubeAS, CallMoveCubeASRequest, CallMoveCubeASResponse
# Importamos la declaracion de mensaje "PoseOriginTarget" para el mensaje objetivo de la accion
# (se trata de un mensaje personalizado)
from practice_msgs.msg import PoseOriginTarget
# Importamos otras declaraciones de mensaje ya predefinidas
from geometry_msgs.msg import Pose, Point, Quaternion

class CallMoveCubeASClient():
    def __init__(self):
        pass

    # Llamamos al servicio
    def callService(self,  service_name):
        # Esperamos hasta que el servicio este disponible
        rospy.wait_for_service(service_name)
        # Creamos una instancia del servicio (la cual podemos invocar)
        self.__srv=rospy.ServiceProxy(service_name, CallMoveCubeAS)
        # Creamos nuestro objeto de solicitud
        req=CallMoveCubeASRequest(
            pose_origin_target=PoseOriginTarget(
                pose_origin=Pose(
                    position=Point(0,0,0),
                    orientation=Quaternion(0,0,0,0)
                ),
                pose_target=Pose(
                    position=Point(10,0,0),
                    orientation=Quaternion(0,0,0,0)
                )
            )
        )
        # Llamamos al servicio y obtenemos un objeto de respuesta
        res=self.__srv(req)
        rospy.loginfo_once("\nCall made")
        return res

def main():
    # Creamos una instancia de la clase
    srv=CallMoveCubeASClient()
    # Llamamos al servicio
    res=srv.callService("call_move_cube_as")
    if res.success:
        print(res.message)
    else:
        print("There has been a problem")

if __name__=='__main__':
    try:
        main()
    except rospy.ServiceException as exception:
        pass

