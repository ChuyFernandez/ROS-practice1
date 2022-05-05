#!/usr/bin/env python

# importamos este modulo que nos permite obtener la lista de argumentos de la linea de comandos
import sys
# Importamos la libreria cliente ROS para Python
import rospy
# Importamos la declaracion de servicio "ConvertMetresToFeet" y los mensajes de solicitud y respuesta
from practice_msgs.srv import ConvertMetresToFeet, ConvertMetresToFeetRequest, ConvertMetresToFeetResponse

class ConvertMetresToFeetClient():
    def __init__(self):
        pass

    # Llamamos al servicio
    def callService(self,  service_name, measurement_metres):
        # Esperamos hasta que el servicio este disponible
        rospy.wait_for_service(service_name)
        # Creamos una instancia del servicio (la cual podemos invocar)
        self.__srv=rospy.ServiceProxy(service_name, ConvertMetresToFeet)
        # Creamos nuestro objeto de solicitud
        req=ConvertMetresToFeetRequest(measurement_metres=measurement_metres)
        # Llamamos al servicio y obtenemos un objeto de respuesta
        res=self.__srv(req)
        rospy.loginfo_once("\nCall made")
        return res

def main():
    if len(sys.argv)==1:
         print("%s [measurement_metres]"%(sys.argv[0]))
         sys.exit(1)
    else:
        # Guardamos la medida en metros
        measurement_metres=float(sys.argv[1])
        # Creamos una instancia de la clase
        srv=ConvertMetresToFeetClient()
        # Llamamos al servicio
        res=srv.callService("convert_metres_to_feet", measurement_metres)
        if res.success:
            print("Response: {} ft".format(res.measurement_feets))

if __name__=='__main__':
    try:
        main()
    except rospy.ServiceException as exception:
        pass

