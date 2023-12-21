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
        
        pose_luz = PoseStamped()
        pose_luz.header.frame_id = self.robot_commander.get_planning_frame()
        pose_luz.pose.position.x = 0.64
        pose_luz.pose.position.y = 0.05
        pose_luz.pose.position.z = -0.21
        self.planning_scene.add_box("luz",pose_luz,(0.742211,0.742211,0.742211))

        self.move_group.set_planning_time(10)
        self.move_group.set_num_planning_attempts(5)

        rospy.Subscriber('/puntos_interes', PoseArray, self.callback)

    def mover_articulaciones(self, valores_articulaciones: list) -> bool:
        return self.move_group.go(valores_articulaciones)

    def mover_a_punto_interes(self, punto: Point) -> None:
        lista_pose = [punto.x, punto.y, 0.75, 0, 0, 0]
        self.mover_a_pose(lista_pose)
        
    def callback(self, msg: PoseArray):
        self.array_poses = msg

    def mover_a_pose(self, pose_meta: PoseStamped) -> bool:

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
    
    def empezar_rutina(self) -> None:
        poses = deepcopy(self.array_poses)
        posesStamped = []
        print(poses.poses)
        for i in range(len(poses.poses)):
            p = PoseStamped()
            p.pose.position.x = poses.poses[i].position.x + self.arucoReference.pose.position.x
            p.pose.position.y = poses.poses[i].position.y + self.arucoReference.pose.position.y
            p.pose.position.z = poses.poses[i].position.z + self.arucoReference.pose.position.z
            p.pose.orientation.w = self.arucoReference.pose.orientation.w
            p.pose.orientation.x = self.arucoReference.pose.orientation.x
            p.pose.orientation.y = self.arucoReference.pose.orientation.y
            p.pose.orientation.z = self.arucoReference.pose.orientation.z
            
            posesStamped.append(deepcopy(p))
            print(posesStamped)
        while not rospy.is_shutdown():
            poses = deepcopy(self.array_poses)
        
            # movimientos robot   
        

    
if __name__ == '__main__':
    control_robot = ControlRobot()

    if control_robot.arucoReference is None:
            control_robot.arucoReference = control_robot.move_group.get_current_pose()
    
    input("Presiona Enter para continuar...")
    
    control_robot.empezar_rutina()
