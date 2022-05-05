#!/usr/bin/env python

# Importamos la libreria cliente ROS para Python
import rospy
# Importamos la declaracion de servicio "HeightTest" y los mensajes de solicitud y respuesta
from practice_msgs.srv import HeightTest, HeightTestRequest, HeightTestResponse
# Importamos la declaracion de servicio "ConvertMetresToFeet" y los mensajes de solicitud y respuesta
from practice_msgs.srv import ConvertMetresToFeet, ConvertMetresToFeetRequest, ConvertMetresToFeetResponse

class HeightTestService():
    def __init__(self):
        # Inicializamos un nodo ROS
        rospy.init_node("height_test_service_node", anonymous=False)
        self.__service_name_convert_metres_to_feet="convert_metres_to_feet"
    
    def createService(self, srv_name):
        # Creamos el servicio
        self.__srv=rospy.Service(srv_name, HeightTest, self.__handler)
        rospy.loginfo_once("\nService 'height_test' created")
        # Mantenemos ejecutando el nodo hasta que se le de una senal de apagado
        rospy.spin()
    
    # Recibimos una solicitud y devolvemos una respuesta
    def __handler(self, req):
        # Mostramos el contenido de la solicitud
        #print("Request: {} mts".format(req.height_cube_mts))

        # Llamamos al servicio que convierte de metros a pies y guardamos la respuesta en una variable
        res=self.__callServiceConvertMetresToFeet(req.height_cube_mts)

        if res.success:
            # Indicamos si pasa la prueba de altura o no
            # Si la altura es mayor o igual a 6.56 ft (aprox 2.0 mts) no pasa la prueba, de lo contrario, si pasa la prueba
            result=False if res.measurement_feets>=6.56 else True

            # Devolvemos una respuesta 
            return HeightTestResponse(result=result)
        else:
            rospy.logerr("\nThere has been an error in the conversion")

    # Llamamos al servicio que convierte de metros a pies
    def __callServiceConvertMetresToFeet(self, height_cube_mts):
        # Esperamos hasta que el servicio este disponible
        rospy.wait_for_service(self.__service_name_convert_metres_to_feet)
        # Creamos una instancia del servicio (la cual podemos invocar)
        self.__srv=rospy.ServiceProxy(self.__service_name_convert_metres_to_feet, ConvertMetresToFeet)
        # Creamos nuestro objeto de solicitud
        req=ConvertMetresToFeetRequest(measurement_metres=height_cube_mts)
        # Llamamos al servicio y obtenemos un objeto de respuesta
        res=self.__srv(req)
        rospy.loginfo_once("\nCall made")
        return res

def main():
    # Creamos una instancia de la clase
    height_test_service=HeightTestService()
    # Creamos el servicio
    height_test_service.createService(srv_name="height_test")

if __name__=='__main__':
    try:
        main()
    except rospy.ServiceException as exception:
        pass

