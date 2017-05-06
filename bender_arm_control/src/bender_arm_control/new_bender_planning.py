#!/usr/bin/env python
import rospy

from bender_moveit_python import MoveGroupInterface

JOINT_STATE_TOPIC = "/bender/joint_states"
ROBOT_DESCRIPTION = "robot_description"
MONITORED_PLANNING_SCENE_TOPIC = "/move_group/monitored_planning_scene"
ATTACHED_COLLISION_OBJECT_TOPIC = "/attached_collision_object"
COLLISION_OBJECT_TOPIC = "/collision_object"
PLANNING_SCENE_TOPIC = "/planning_scene"
PLANNING_SCENE_WORLD_TOPIC = "/planning_scene_world"





def main():

    #Iniciar Nodo ROS
    rospy.init_node('new_bender_planning')

    #Obtener argumentos (POR AHORA PARA l_arm)
    planning_group ="l_arm"

    #Cargar planning scene monitor

    #Modificar planning scene monitor para obtener el estado del robot

    psi = PlanningSceneInterface("bender/l_arm_base")

    

    rospy.loginfo("Publishing planning scene on "+PLANNING_SCENE_TOPIC)

    #Interface para move_group l_arm y r_arm

    l_arm = MoveGroupInterface(planning_group, "bender/l_arm_base")
    #r_arm = MoveGroupInterface("planning_group_name", "bender/r_arm_base")
    #Capa de servicios

    rospy.spin()


if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass
    