#!/usr/bin/env python

# Importamos la libreria cliente ROS para Python
import rospy
# Importamos la declaracion de servicio "ConvertMetresToFeet" y los mensajes de solicitud y respuesta
from hello_world_msgs.srv import ConvertMetresToFeet, ConvertMetresToFeetRequest, ConvertMetresToFeetResponse

class ConvertMetresToFeetService():
    def __init__(self):
        # Inicializamos un nodo ROS
        rospy.init_node("metres_to_feet_server", anonymous=False)
        self.__conversion_factor_metres_to_feet=3.281
    
    def createService(self, srv_name):
        # Creamos el servicio
        self.__srv=rospy.Service(srv_name, ConvertMetresToFeet, self.__handler)
        rospy.loginfo_once("\nService created")
        # Mantenemos ejecutando el nodo hasta que se le de una senal de apagado
        rospy.spin()
    
    # Recibimos una solicitud y devolvemos una respuesta
    def __handler(self, req):
        # Mostramos el contenido de la solicitud
        print("Request: {} mt".format(req))
        # Hacemos la conversion de metros a pies
        feets=req.measurement_metres*self.__conversion_factor_metres_to_feet
        # Devolvemos una respuesta 
        return ConvertMetresToFeetResponse(measurement_feets=feets, success=True)

def main():
    # Creamos una instancia de la clase
    convert_metres_to_feet_service=ConvertMetresToFeetService()
    # Creamos el servicio
    convert_metres_to_feet_service.createService(srv_name="metres_to_feet")

if __name__=='__main__':
    try:
        main()
    except rospy.ServiceException as exception:
        pass

