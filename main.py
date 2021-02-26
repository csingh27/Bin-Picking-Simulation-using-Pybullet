## --IMPORT LIBRARIES-- # 
print("Importing Libraries ...")
import pybullet as p
import time
import pybullet_data
import os
import sys
import numpy as np
import time
import random
from collections import namedtuple
from attrdict import AttrDict
import math 
from datetime import datetime
print(os.environ['DISPLAY'])
# --LOAD PYBULLET GUI-- #
print("Loading PyBullet ...")
FILE_PATH = os.path.dirname(os.path.realpath(__file__))
print(FILE_PATH)
os.chdir(FILE_PATH)
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
# --SETTING REALTIME SIMULATION-- #
print("Setting Realtime Simulation ...")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setRealTimeSimulation(True)
p.setGravity(0,0,-10)

import definitions as d

# --DEFINE THE CONTROL FUNCTIONS-- #   
def initialize_robot(choice):
            if choice==0:
                control=0
                motors_arm(control,choice,0)
            if choice==1:
                control=end_effector_add_sliders()
                print("End Effector control activated ...")
                motors_arm(control,choice,0)
            if choice==2:
                control=joint_add_sliders()
                print("Joint control activated ...")
                motors_arm(control,choice,0)
            elif choice==3:
                print("Grasp control activated ...")
                control=0
                grasp_user_input()        
  
time.sleep(2)    
if d.mode==9:
	print("EXITING ...")
	exit()
else :        
	# --IMPORT UR5 ROBOT WITH GRIPPER-- #
	print("------------------------------------- \n")
	print("CHOOSE A ROBOT-GRIPPER CONFIGURATION ... \n")
	print("1: UR5 with virtual Suction \n")
	print("2: UR5 with Basic Gripper \n")
	print("3: UR5 with Robotiq Gripper \n")
	print("4: Diana with virtual Suction \n")
	print("------------------------------------- \n")
	Flags=p.URDF_USE_SELF_COLLISION
	FILE_PATH=os.path.join(FILE_PATH,"Configurations")
	os.chdir(FILE_PATH)
	sys.path.append(FILE_PATH)
	robotinput=input()
	if robotinput==1:
    		d.ROBOT_URDF_MODEL=d.ROBOT_URDF_MODEL_SUCTION
    		print("Virtual Suction gripper selected \n") 
    		from Simulation_UR5_SuctionGripper import end_effector_add_sliders
    		from Simulation_UR5_SuctionGripper import end_effector_control
    		from Simulation_UR5_SuctionGripper import motors_arm
    		from Simulation_UR5_SuctionGripper import joint_add_sliders    
    		from Simulation_UR5_SuctionGripper import grasp_user_input    
    		from Simulation_UR5_SuctionGripper import grasp_control  
	elif robotinput==2:
    		d.ROBOT_URDF_MODEL=d.ROBOT_URDF_MODEL_BASIC_GRIPPER
    		print("Basic gripper selected \n")
    		from Simulation_UR5_BasicGripper import end_effector_add_sliders
    		from Simulation_UR5_BasicGripper import end_effector_control
    		from Simulation_UR5_BasicGripper import motors_arm
    		from Simulation_UR5_BasicGripper import joint_add_sliders    
    		from Simulation_UR5_BasicGripper import grasp_user_input    
    		from Simulation_UR5_BasicGripper import grasp_control  
	elif robotinput==3:
    		d.ROBOT_URDF_MODEL=d.ROBOT_URDF_MODEL_ROBOTIQ_GRIPPER
    		print("Robotiq gripper selected \n")
    		from Simulation_UR5_RobotiqGripper import end_effector_add_sliders
    		from Simulation_UR5_RobotiqGripper import end_effector_control
    		from Simulation_UR5_RobotiqGripper import motors_arm
    		from Simulation_UR5_RobotiqGripper import joint_add_sliders    
    		from Simulation_UR5_RobotiqGripper import grasp_user_input    
    		from Simulation_UR5_RobotiqGripper import grasp_control  
	elif robotinput==4:
    		d.ROBOT_URDF_MODEL=d.ROBOT_URDF_MODEL_DIANA_SUCTION_GRIPPER
    		print("Diana robot with virtual suction selected \n")
    		from Simulation_Diana_SuctionGripper import end_effector_add_sliders
    		from Simulation_Diana_SuctionGripper import end_effector_control
    		from Simulation_Diana_SuctionGripper import motors_arm
    		from Simulation_Diana_SuctionGripper import joint_add_sliders    
    		from Simulation_Diana_SuctionGripper import grasp_user_input    
    		from Simulation_Diana_SuctionGripper import grasp_control 
	print("Importing robot ... \n")        
	# --INITIALIZE THE ROBOT-- #
	print("Initializing the robot ...\n")
	initialize_robot(0)
	print("------------------------------------- \n")
	print("CHOOSE A ROBOT CONTROL MODE ... \n")
	print("1: End-effector control1 \n")
	print("2: Joint control \n")
	print("3: Grasp control \n")
	print("------------------------------------- \n")
	choice=input()
	if choice==1 or choice==2 or choice==3:
    		print("In the loop...")
    		initialize_robot(choice)
	else:
    		print("Please select 1,2 or 3")
    		choice  

