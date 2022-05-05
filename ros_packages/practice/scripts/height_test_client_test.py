#!/usr/bin/env python

# importamos este modulo que nos permite obtener la lista de argumentos de la linea de comandos
import sys
# Importamos la libreria cliente ROS para Python
import rospy
# Importamos la declaracion de servicio "HeightTest" y los mensajes de solicitud y respuesta
from practice_msgs.srv import HeightTest, HeightTestRequest, HeightTestResponse

class HeightTestClient():
    def __init__(self):
        pass

    # Llamamos al servicio
    def callService(self,  service_name, height_cube_mts):
        # Esperamos hasta que el servicio este disponible
        rospy.wait_for_service(service_name)
        # Creamos una instancia del servicio (la cual podemos invocar)
        self.__srv=rospy.ServiceProxy(service_name, HeightTest)
        # Creamos nuestro objeto de solicitud
        req=HeightTestRequest(height_cube_mts=height_cube_mts)
        # Llamamos al servicio y obtenemos un objeto de respuesta
        res=self.__srv(req)
        rospy.loginfo_once("\nCall made")
        return res

def main():
    if len(sys.argv)==1:
         print("%s [height_cube_mts]"%(sys.argv[0]))
         sys.exit(1)
    else:
        # Guardamos la medida en metros
        height_cube_mts=float(sys.argv[1])
        # Creamos una instancia de la clase
        srv=HeightTestClient()
        # Llamamos al servicio
        res=srv.callService("height_test", height_cube_mts)
        if res.result:
            print("Test passed")
        else:
            print("Test not passed")

if __name__=='__main__':
    try:
        main()
    except rospy.ServiceException as exception:
        pass

