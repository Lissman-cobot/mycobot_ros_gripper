#! /usr/bin/python3

# Python 2/3 compatibility imports
from __future__ import print_function
#from six.moves import input

#Include the necessary libraries 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry, AllowedCollisionMatrix
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_commander import PlanningSceneInterface, RobotCommander, MoveGroupCommander

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi
    
    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class myCobot:

    # Default Constructor
    def __init__(self, Group_Name, Grasping_Group_Name):

        #Initialize the moveit_commander and rospy node
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_set_predefined_pose', anonymous=True)

        
        #Instantiate a RobotCommander object. This object is the outer-level interface to the robot
        self._robot = moveit_commander.RobotCommander()
        #Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
        self._scene = moveit_commander.PlanningSceneInterface()
        
        #define the movegoup for the robotic 
        #Move group name is taken as input when initializing an object from the myCobot Class
        self._planning_group = Group_Name
        #Enf effector planning group name
        self._grasping_group = Grasping_Group_Name
        #Instantiate a MoveGroupCommander Object. This Object is an interface to the one group of joints. this interface can be used to plan and execute the motions on the robotic arm
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._group.set_goal_tolerance(0.01)  # Set goal tolerance
        self._group.set_goal_joint_tolerance(0.01)  # Set goal joint tolerance
        #Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        #Create action client for the "Execute Trajectory" action server
        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        #Get the planning frame, end effector link and the robot group names
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        self.box_name = ""
        self.get_planning_scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.apply_planning_scene_srv = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)

        #print the info
        #here the '\033[95m' represents the standard colour "LightMagenta" in terminals. For details, refer: https://pkg.go.dev/github.com/whitedevops/colors
        #The '\033[0m' is added at the end of string to reset the terminal colours to default
        rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> myCobot initialization is done." + '\033[0m')   

    def set_pose(self, arg_pose_name):
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        
        #for moveit_commander member functions in Python 3 (For Noetic), please refer: https://docs.ros.org/en/noetic/api/moveit_commander/html/functions_func.html
        #for moveit_commander member functions in Python 2, please refer(For Kinetic or Melodic): https://docs.ros.org/en/kinetic/api/moveit_commander/html/functions_func.html
        #Python file with function definitions: https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
        #Python file with function definitions (for Noetic): https://docs.ros.org/en/noetic/api/moveit_commander/html/move__group_8py_source.html
        #Python file with function definitions (for Kinetic or Melodic): https://docs.ros.org/en/kinetic/api/moveit_commander/html/move__group_8py_source.html

        #Set the predefined position as the named joint configuration as the goal to plan for the move group. The predefined positions are defined in the Moveit Setup Assistant
        self._group.set_named_target(arg_pose_name)
        
        #Plan to the desired joint-space goal using the default planner
        #The below line of code is different for melodic and noetic
        plan_success, plan, planning_time, error_code = self._group.plan()
        
        #Create a goal message object for the action server
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        #Update the trajectory in the goal message
        goal.trajectory = plan
        #Send the goal to the action server
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        #Print the current pose
        rospy.loginfo('\033[32m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    
    #Start of code for box
    def wait_for_state_update(self,name, box_is_known=False, box_is_attached=False, timeout=4):
       #box_name = self.box_name
       box_name = name
       scene = self._scene
       start = rospy.get_time()
       seconds = rospy.get_time()
       
       while (seconds - start < timeout) and not rospy.is_shutdown():
           # Test if the box is in attached objects
           attached_objects = scene.get_attached_objects([box_name])
           is_attached = len(attached_objects.keys()) > 0
           
           # Test if the box is in the scene.
           # Note that attaching the box will remove it from known_objects
           is_known = box_name in scene.get_known_object_names()
           # Test if we are in the expected state
           
           if (box_is_attached == is_attached) and (box_is_known == is_known):
              return True
           
           # Sleep so that we give other threads time on the processor
           rospy.sleep(0.1)
           seconds = rospy.get_time()
           
       # If we exited the while loop without returning then we timed out
       return False
	    
    def add_box(self,name,x,y,z,sx,sy,sz, timeout=4):
        box_name = name
        scene = self._scene

        ## First, create an message object of type pose to deifne box positions
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_footprint"
	    
	    #set the position massage arguments
        box_pose.pose.position.x = x # above the panda_hand frame
        box_pose.pose.position.y = y # above the panda_hand frame
        box_pose.pose.position.z = z # above the panda_hand frame
        box_pose.pose.orientation.x = 0
        box_pose.pose.orientation.y = 0
        box_pose.pose.orientation.x = 0
        box_pose.pose.orientation.w = 1.0
        #Add the box in the scean
        scene.add_box(box_name, box_pose, size=(sx, sy, sz))
        #https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
        #https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html

        #Set the box name for box_name of the class
        self.box_name=box_name
        #Check if the box is added sucessfully
        return self.wait_for_state_update(name, box_is_known=True, timeout=timeout)

    def attach_box(self,name, timeout=4):
	    #box_name = self.box_name
        box_name = name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link

        grasping_group = self._grasping_group
        touch_links = robot.get_link_names(group=grasping_group)
        rospy.loginfo('\033[95m' + "Touch Link: {}".format(touch_links) + '\033[0m')
        scene.attach_box(eef_link , box_name, touch_links=touch_links)
        #https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
        #https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html

        # We wait for the planning scene to update.
        return self.wait_for_state_update(name,box_is_known=False, box_is_attached=True,  timeout=timeout)

    def detach_box(self,name, timeout=4):
	    #box_name = self.box_name
        box_name = name
        scene = self._scene
        eef_link = self._eef_link
        
        scene.remove_attached_object(eef_link, name=box_name)
        #https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
        #https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html

        return self.wait_for_state_update(name, box_is_known=True, box_is_attached=False, timeout=timeout)
	
    def remove_box(self,name, timeout=4):
        box_name = name
        scene = self._scene
        scene.remove_world_object(box_name)
       #https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
       #https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html
       
        return self.wait_for_state_update(name, box_is_known=False, box_is_attached=False,  timeout=timeout)

    # Class Destructor
    def __del__(self):
        #When the actions are finished, shut down the moveit commander
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[95m' + "Object of class myCobot Deleted." + '\033[0m')


def main():

    #Create a new arm object from the myCobot class
    arm = myCobot("arm_group", "gripper_group")
    hand =  myCobot("gripper_group", "gripper_group")
    robotArm = myCobot("arm_group", "gripper_group")

    #call the function to set the position to "zero_pose"
    arm.set_pose("init_pose")
    #Wait for 2 seconds
    rospy.sleep(3)
    
    hand.set_pose("gripper_opened")
    rospy.sleep(3)
    
    #Open the gripper or end effector
    

    #Here, we will repeat the cycle of setting to various positions, simulating the pick and place action
    while not rospy.is_shutdown():
        #task = input("Enter add, attach, detach, remove, pick, close, lift, place, open, init: ")
    
        #if task == "add":
        robotArm.add_box("package$1",0.3,-0.09,0.05,0.04,0.04,0.04)
        rospy.loginfo('\033[32m' + "Added Object in Rviz: {}".format(robotArm.box_name) + '\033[0m')
        rospy.sleep(2)
        #Go to Pick Object Pose
        #if task == "pick":
        arm.set_pose("pick_object")
        rospy.sleep(2)

        hand.set_pose("gripper_closed")
        rospy.sleep(2)
        #if task == "attach":
        robotArm.attach_box("package$1")
        rospy.loginfo('\033[32m' + "Attached Object in Rviz: {}".format(robotArm.box_name) + '\033[0m')      
        #hand.set_pose("gripper_closed")
        rospy.sleep(2)
               
        #if task == "lift":
        arm.set_pose("lift_object")
        rospy.sleep(2)    
        
        #Go to drop Object Pose
        #if task == "place":
        arm.set_pose("place_object")
        rospy.sleep(2) 
        
        #Open the gripper or end effector
        #if task == "open":
        hand.set_pose("gripper_opened")
        rospy.sleep(2) 

        #if task == "detach":
        robotArm.detach_box("package$1")
        rospy.loginfo('\033[32m' + "Detached Object from Rviz: {}".format(robotArm.box_name) + '\033[0m')
        rospy.sleep(2) 
        #if task == "remove":
        robotArm.remove_box("package$1")
        rospy.loginfo('\033[32m' + "Removed Object from Rviz: {}".format(robotArm.box_name) + '\033[0m')
        rospy.sleep(2) 

        #if task == "init":
        arm.set_pose("init_pose")
        rospy.sleep(2) 
     
        #if task == "x":
            #break

    
    #delete the arm object at the end of code
    del arm
    del hand
    del robotArm
	


if __name__ == '__main__':
    main()



