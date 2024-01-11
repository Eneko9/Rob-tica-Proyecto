from moveit_commander import MoveGroupCommander
from moveit_commander import PlanningSceneInterface
from moveit_commander import RobotCommander
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, PoseArray
from std_msgs.msg import String
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
        self.msg = None
        self.arucoReference = None
        
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

    def mover_articulaciones(self, valores_articulaciones: list) -> bool:
        return self.move_group.go(valores_articulaciones)

    def mover_a_punto_interes(self, punto: Point) -> None:
        # Mover el robot al punto de interés recibido
        lista_pose = [punto.x, punto.y, 0.75, 0, 0, 0]
        self.mover_a_pose(lista_pose)
        
    def __callback(self, msg: PoseArray) -> None:
        self.array_poses = msg

    def mover_a_pose(self, pose_meta: PoseStamped) -> bool:

        self.move_group.set_pose_target(pose_meta)
        for i in range(5):
            success, trajectory, _, _ =self.move_group.plan()
            if success:
                break
        else:
            return False

        return self.move_group.execute(trajectory)
    
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
    
    # def empezar_rutina(self) -> None:
    #     poses = deepcopy(self.array_poses)
    #     print(poses)
    #     posesStamped = []
    #     #print(poses.poses)
        
    #     poses = [[-0.10625,-0.085],[-0.13625,-0.12375],[-0.14125000000000001,-0.051250000000000004]]
        
    #     for i in range(len(poses.poses)):
    #         p = PoseStamped()

    #         p.pose.position.x = poses.poses[i].position.x + self.arucoReference.pose.position.x
    #         p.pose.position.y = poses.poses[i].position.y + self.arucoReference.pose.position.y
    #         p.pose.position.z = poses.poses[i].position.z + self.arucoReference.pose.position.z
    #         p.pose.orientation.w = self.arucoReference.pose.orientation.w
    #         p.pose.orientation.x = self.arucoReference.pose.orientation.x
    #         p.pose.orientation.y = self.arucoReference.pose.orientation.y
    #         p.pose.orientation.z = self.arucoReference.pose.orientation.z
            
            
    #         posesStamped.append(deepcopy(p))
    #         print(posesStamped)
    #     while not rospy.is_shutdown():
    #         poses = deepcopy(self.array_poses)
    #         for poseStamped in posesStamped:
    #             self.mover_a_pose(poseStamped)
    #         # movimientos robot   

    def mover_a_home(self) -> bool:
        self.move_group.set_named_target("home")
        return self.move_group.go()
            
    def empezar_rutina(self) -> None:
        poses: PoseArray = deepcopy(self.array_poses)
        print(poses)
        posesStamped = []
        #print(poses.poses)
        
        #poses = [[0.0985,0],[0.0661,0]]
        
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
            poses = deepcopy(self.array_poses)
            for poseStamped in posesStamped:
                self.mover_a_pose(poseStamped)
            # movimientos robot   
        

    
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
    
    # input("Mueve el robot a la posicion inicial y dale a ENTER...")
    
    # punto_objetivo = control_robot.move_group.get_current_pose()
    # print(punto_objetivo)
    
    input("Mueve el robot a la posicion inicial y dale a ENTER...")
    
    control_robot.empezar_rutina()
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
