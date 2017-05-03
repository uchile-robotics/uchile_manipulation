#!/usr/bin/env python
import rospy

def main():

    #Iniciar Nodo ROS
    rospy.init_node('new_bender_planning')

    #Obtener argumentos

    #Cargar planning scene monitor

    #Modificar planning scene monitor para obtener el estado del robot

    #Interface para move_group l_arm y r_arm

    #Capa de servicios

    rospy.spin()


if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass
    