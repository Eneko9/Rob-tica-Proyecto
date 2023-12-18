from moveit_commander import MoveGroupCommander
from moveit_commander import PlanningSceneInterface
from moveit_commander import RobotCommander
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandActionGoal
import rospy
import yaml 

class ControlRobot:
    def __init__(self) -> None:
        rospy.init_node("mi_primer_nodo",anonymous=True)
        rospy.sleep(2)
        self.move_group = MoveGroupCommander("robot")
        self.planning_scene = PlanningSceneInterface()
        self.robot_commander = RobotCommander()        

        pose_suelo = PoseStamped()
        pose_suelo.header.frame_id = self.robot_commander.get_planning_frame()
        pose_suelo.pose.position.z = -0.022
        self.planning_scene.add_box("suelo",pose_suelo,(3,3,0.02))
        
        pose_luz = PoseStamped()
        pose_luz.header.frame_id = self.robot_commander.get_planning_frame()
        pose_luz.pose.position.x = 0.64
        pose_luz.pose.position.y = 0.05
        pose_luz.pose.position.z = -0.21
        self.planning_scene.add_box("luz",pose_luz,(0.742211,0.742211,0.742211))

        self.move_group.set_planning_time(10)
        self.move_group.set_num_planning_attempts(5)

        self.publicador_pinza = rospy.Publisher("/rg2_action_server/goal",
                                                 GripperCommandActionGoal,
                                                 queue_size=10)
        rospy.Subscriber('/puntos_interes', Point, self.callback_puntos_interes)

    def mover_articulaciones(self, valores_articulaciones: list) -> bool:
        return self.move_group.go(valores_articulaciones)

    def mover_a_punto_interes(self, punto: Point) -> None:
        # Mover el robot al punto de interés recibido
        lista_pose = [punto.x, punto.y, 0, 0, 0, 0]
        self.mover_a_pose(lista_pose)

    def callback_puntos_interes(self, msg):
         if not self.puntos_recibidos:
            print("Punto de interés recibido: ({}, {})".format(msg.x, msg.y))
            #self.mover_a_punto_interes(msg)
            self.puntos_recibidos = True  # Marcar que se han recibido puntos
        
    def mover_a_pose(self, lista_pose: list) -> bool:
        orientacion_quaternion = quaternion_from_euler(lista_pose[3],
                                                       lista_pose[4],
                                                       lista_pose[5])        
        
        pose_meta = PoseStamped()
        pose_meta.header.frame_id = self.robot_commander.get_planning_frame()
        pose_meta.pose.position.x = lista_pose[0]
        pose_meta.pose.position.y = lista_pose[1]
        pose_meta.pose.position.z = lista_pose[2]
        pose_meta.pose.orientation.w = orientacion_quaternion[3]
        pose_meta.pose.orientation.x = orientacion_quaternion[0]
        pose_meta.pose.orientation.y = orientacion_quaternion[1]
        pose_meta.pose.orientation.z = orientacion_quaternion[2]

        self.move_group.set_pose_target(pose_meta)
        for i in range(5):
            success, trajectory, _, _ =self.move_group.plan()
            if success:
                break
        else:
            return False
        
        if not success:
            return False

        return self.move_group.execute(trajectory)

    def manejar_pinza(self, anchura: float, fuerza: float) -> None:
        msg_pinza = GripperCommandActionGoal()
        msg_pinza.goal.command.position = anchura
        msg_pinza.goal.command.max_effort = fuerza

        self.publicador_pinza.publish(msg_pinza)

if __name__ == '__main__':
    control_robot = ControlRobot()
    rospy.spin()
    # pose = control_robot.move_group.get_current_pose()
    # angulos = control_robot.move_group.get_current_joint_values()
    # # with open("poses","w+") as f:
    # #     yaml.dump(pose,f)
    # with open("poses","r+") as f:
    #     mi_pose = yaml.load(f, yaml.Loader)
    # #control_robot.mover_a_pose([1.4394619464874268, -0.4464119237712403, -0.4029938280582428, -0.7764907044223328, -1.5569680372821253, -0.09305268922914678])
    # #control_robot.mover_a_pose([0, 0.4, 0, 0, 0, 0])
    # control_robot.move_group.set_pose_target(mi_pose)
    # control_robot.move_group.go()
 
    # print(control_robot.move_group.get_current_joint_values())
    # pass
 


























# rospy.init_node("nodo_pruebas",anonymous=True)
# move_group = MoveGroupCommander("robot")
# print(move_group.get_end_effector_link())

# class PruebasRobot:
#     def __init__(self) -> None:
#         rospy.init_node("nodo_pruebas",anonymous=True)
#         self.move_group = MoveGroupCommander("robot")
#         pose_caja = PoseStamped()
#         pose_caja.header.frame_id = self.move_group.get_planning_frame()
#         pose_caja.pose.position.x = 0.0
#         pose_caja.pose.position.y = 0.0
#         pose_caja.pose.position.z = -0.011

#         escena = PlanningSceneInterface()
#         escena.add_box("suelo",pose_caja,(2,2,0.02))

#         self.publisher_pinza = rospy.Publisher("/rg2_action_server/goal",GripperCommandActionGoal,queue_size=10)
        

#     def control_pinza(self, width: float, fuerza: float) -> None:
#         msg_pinza = GripperCommandActionGoal()
#         msg_pinza.goal.command.position = width
#         msg_pinza.goal.command.max_effort = fuerza

#         self.publisher_pinza.publish(msg_pinza)

# if __name__ == '__main__':
#     from time import sleep
#     test = PruebasRobot()
#     sleep(2)
#     test.control_pinza(100.0, 40.0)
#     test.control_pinza(100.0, 40.0)
#     rospy.spin()
