from moveit_commander import MoveGroupCommander
from moveit_commander import PlanningSceneInterface
from moveit_commander import RobotCommander
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, PoseArray
from std_msgs.msg import Bool, Int64
from tf.transformations import quaternion_from_euler
import rospy
from copy import deepcopy

class ControlRobot:
    def __init__(self) -> None:
        rospy.init_node("mi_primer_nodo",anonymous=True)
        rospy.sleep(2)
        self.move_group = MoveGroupCommander("robot")
        self.planning_scene = PlanningSceneInterface()
        self.robot_commander = RobotCommander()      

        self.array_poses = PoseArray()
        self.gesto = int
        self.msg = None
        self.arucoReference = None
        self.stop = False
        
        pose_suelo = PoseStamped()
        pose_suelo.header.frame_id = self.robot_commander.get_planning_frame()
        pose_suelo.pose.position.y = 1.4
        pose_suelo.pose.position.z = -0.022
        self.planning_scene.add_box("suelo",pose_suelo,(3,3,0.02))
        
        pose_palo_vertical = PoseStamped()
        pose_palo_vertical.header.frame_id = self.robot_commander.get_planning_frame()
        pose_palo_vertical.pose.position.z = 0
        pose_palo_vertical.pose.position.x = 0.30
        pose_palo_vertical.pose.position.y = -0.05
        self.planning_scene.add_box("palo_vertical",pose_palo_vertical,(0.1,0.1,3))
        
        pose_palo_horizontal = PoseStamped()
        pose_palo_horizontal.header.frame_id = self.robot_commander.get_planning_frame()
        pose_palo_horizontal.pose.position.z = 0.77
        pose_palo_horizontal.pose.position.x = 0.30
        pose_palo_horizontal.pose.position.y = 0
        self.planning_scene.add_box("palo_horizontal",pose_palo_horizontal,(0.1,0.1,3))
        
        pose_palo2 = PoseStamped()
        pose_palo2.header.frame_id = self.robot_commander.get_planning_frame()
        pose_palo2.pose.position.z = 0.75
        pose_palo2.pose.position.x = 0.35
        pose_palo2.pose.position.y = 1.4

        self.planning_scene.add_box("palo_horizontal",pose_palo2,(0.2,3,0.05))
        
        # pose_luz = PoseStamped()
        # pose_luz.header.frame_id = self.robot_commander.get_planning_frame()
        # pose_luz.pose.position.x = 0.64
        # pose_luz.pose.position.y = 0.05
        # pose_luz.pose.position.z = -0.21
        # self.planning_scene.add_box("luz",pose_luz,(0.742211,0.742211,0.742211))

        self.move_group.set_planning_time(5)
        self.move_group.set_num_planning_attempts(2)

        rospy.Subscriber('/puntos_interes', PoseArray, self.__callback)
        rospy.Subscriber('/gestos_interes', Int64, self.__callback_gestos)
        rospy.Subscriber('/parada_aforo', Bool, self.__callback_aforo)

    def mover_articulaciones(self, valores_articulaciones: list) -> bool:
        return self.move_group.go(valores_articulaciones)

    def mover_a_punto_interes(self, punto: Point) -> None:
        # Mover el robot al punto de interés recibido
        lista_pose = [punto.x, punto.y, 0.75, 0, 0, 0]
        self.mover_a_pose(lista_pose)
        
    def __callback(self, msg: PoseArray) -> None:
        self.array_poses = msg
        
    def __callback_gestos(self, msg: Int64) -> None:
        self.gesto = msg.data
    
    def __callback_aforo(self, msg: Bool) -> None:
        self.stop = msg

    def mover_a_pose(self, pose_meta: PoseStamped) -> bool:

        self.move_group.set_pose_target(pose_meta)
        for i in range(5):
            success, trajectory, _, _ =self.move_group.plan()
            if success:
                break
        else:
            return False

        return self.move_group.execute(trajectory)
    
    def stop_robot(self):
        while self.gesto != 1 :
            print("Robot parado! Realiza el gesto 1 para reanudar")
        self.stop = False
        
    def mover_a_pose2(self, lista_pose: list) -> bool:   
        pose_meta = PoseStamped()
        pose_meta.header.frame_id = self.robot_commander.get_planning_frame()
        pose_meta.pose.position.x = lista_pose[0]
        pose_meta.pose.position.y = lista_pose[1]
        pose_meta.pose.position.z = lista_pose[2]
        pose_meta.pose.orientation.w = lista_pose[6]
        pose_meta.pose.orientation.x = lista_pose[3]
        pose_meta.pose.orientation.y = lista_pose[4]
        pose_meta.pose.orientation.z = lista_pose[5]

        print(pose_meta.pose)
        self.move_group.set_pose_target(pose_meta)
        # success, trajectory, _, _ =self.move_group.plan()
        for i in range(5):
            success, trajectory, _, _ =self.move_group.plan()
            if success:
                break
        else:
            return False

        return self.move_group.execute(trajectory)
    
    def mover_a_home(self) -> bool:
        self.move_group.set_named_target("home")
        return self.move_group.go()
            
    def empezar_rutina(self) -> None:
        poses: PoseArray = deepcopy(self.array_poses)
        print(poses)
        posesStamped = []
        
        for i, pose in enumerate(poses.poses):
            p = PoseStamped()
            pose: PoseStamped
            p.pose.position.x = self.arucoReference.pose.position.x + pose.position.x + 0.005
            p.pose.position.y = self.arucoReference.pose.position.y - pose.position.y - 0.044
            p.pose.position.z = self.arucoReference.pose.position.z
            p.pose.orientation = self.arucoReference.pose.orientation
            p.header.frame_id = self.robot_commander.get_planning_frame()
            
            posesStamped.append(deepcopy(p))
            print(posesStamped)
        while not rospy.is_shutdown():
            print(self.gesto)
            poses = deepcopy(self.array_poses)
            
            if self.stop or self.gesto == 0: 
                self.stop_robot()
            
            for poseStamped in posesStamped:
                if self.stop or self.gesto == 0: 
                    self.stop_robot()
                self.mover_a_pose(poseStamped)
                poseZ = deepcopy(poseStamped)
                poseZ.pose.position.z = poseZ.pose.position.z - 0.02
                self.mover_a_pose(poseZ)
                self.mover_a_pose(poseStamped)
    
if __name__ == '__main__':
    
    control_robot = ControlRobot()
    control_robot.mover_a_home()

    control_robot.move_group.set_planner_id("RRTstar")
    control_robot.move_group.set_planning_time(10.0)
    posicionAruco = [0.3224506750598117,0.23143119743294796,0.39466705828974225,0.713275847818445,-0.7008834173127139,1.1856567911481408e-05,1.0693572173820851e-05]
    control_robot.mover_a_pose2(posicionAruco)
        
    control_robot.move_group.set_planning_time(5.0)
    control_robot.move_group.set_planner_id("RRTConnect")
    if control_robot.arucoReference is None:
            control_robot.arucoReference = control_robot.move_group.get_current_pose()
            print("POSE GUARDADA!")
            print(control_robot.arucoReference)
    
    control_robot.mover_a_home()
    
    input("Mueve el robot a la posicion inicial y dale a ENTER...")
    
    control_robot.empezar_rutina()
