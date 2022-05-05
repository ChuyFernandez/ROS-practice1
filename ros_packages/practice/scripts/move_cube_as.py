#!/usr/bin/env python

import numpy as np
# Importamos la libreria cliente ROS para Python
import rospy
import actionlib
# Importamos la declaracion de accion "MoveCubeAction" y los mensajes de objetivo, retroalimentacion y resultado
from practice_msgs.msg import MoveCubeAction, MoveCubeGoal, MoveCubeFeedback, MoveCubeResult 
# Importamos otras declaraciones de mensaje ya predefinidas
from geometry_msgs.msg import Pose, Point, Quaternion

class MoveCubeAS():
    def __init__(self):
        # Inicializamos un nodo ROS
        rospy.init_node("move_cube_as_node", anonymous=False)
        # Inicializamos las variables para los mensajes de retroalimentacion y resultado
        self.__feedback=MoveCubeFeedback()
        self.__result=MoveCubeResult()
    
    def createSimpleActionServer(self, action_name):
        # Creamos el servidor de accion simple
        self.__as=actionlib.SimpleActionServer(action_name, MoveCubeAction, execute_cb=self.__execute_cb, auto_start=False)
        self.__as.start()
        rospy.loginfo_once("\nSimple action server 'move_cube' created")
        # Mantenemos ejecutando el nodo hasta que se le de una senal de apagado
        rospy.spin()
    
    # Recibimos un objetivo
    def __execute_cb(self, goal):
        # NOTA: Recordemos que el sistema de coordenadas Unity y ROS no son las mismas, por lo que
        # es importante tener en cuenta (hay que definir con que sistema de coordenadas trabajar y
        # como hacer las respectivas conversiones entre sistemas de coordenadas).
        # Unity (x,y,z) = ROS (z,-x,y)
        # Posicion origen del cubo (donde se encuentra)
        pose_origin=goal.pose_origin_target.pose_origin
        # Posicion destino del cubo (donde se tiene que dirigir)
        pose_target=goal.pose_origin_target.pose_target
        # En la vida real para planear una trayectoria es muy importante conocer el entorno
        # en donde se movera el robot (en este caso es un cubo), donde toda la informacion de su
        # entorno puede estar dada mediante los sensores que incorpora el robot (sensores de distancia,
        # camaras, etc.).
        # Una trayectoria puede variar si el robot se encuentra en un mundo donde los obstaculos
        # no son estaticos, si no dinamicos, es decir, pueden cambiar de posicion mediante el trayecto
        # del robot.
        # Nos preocuparemos por mover al objeto solo en un movimiento en linea recta,
        # esto con el motivo de entender con algo simple como funciona toda esta interaccion.
        success=True
        duration=5
        rate_second=1
        cont=0
        # Aqui usamos un poco de calculo vectorial 
        p0=np.array([pose_origin.position.x, pose_origin.position.z])
        p=np.array([pose_target.position.x, pose_target.position.z])
        u=p-p0
        magnU=np.sqrt(u.dot(u))
        v=u/magnU
        while(cont<duration):
            cont=cont+1
            # Se comprueba si el cliente ha solicitado la preferencia
            if self.__as.is_preempt_requested():
                self.__as.set_preempt()
                success=False
                break
            # Calculamos la posicion hacia donde se movera el cubo
            t=(magnU/duration)*cont
            pos=p0+t*v
            self.__feedback.current_pose=Pose(
                position=Point(pos[0],pose_origin.position.y,pos[1]),
                orientation=Quaternion(0,0,0,0),
            )
            self.__as.publish_feedback(self.__feedback)
            # Dormir por 1 segundo
            rospy.sleep(rate_second)

        if success:
            self.__result.result_message="Succesfully completed move cube"
            self.__as.set_succeeded(self.__result)

def main():
    # Creamos una instancia de la clase
    move_cube_as=MoveCubeAS()
    # Creamos el servicio
    move_cube_as.createSimpleActionServer(action_name="move_cube")

if __name__=='__main__':
    main()

