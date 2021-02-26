## THIS FILE DEFINES THE COMMON FUNCTIONS FOR ALL CONFIGURATIONS
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
from PIL import Image
from collections import namedtuple
from attrdict import AttrDict
import math 
from datetime import datetime
import matplotlib.pyplot as plt
os.environ['DISPLAY']=':0'
print(os.environ['DISPLAY'])
# --LOAD PYBULLET GUI-- #
print("Loading PyBullet ...")
FILE_PATH = os.path.dirname(os.path.realpath(__file__))
print(FILE_PATH)
os.chdir(FILE_PATH)
imgCount=0
# --URDF FILES-- #
TABLE_URDF_MODEL=os.path.join(pybullet_data.getDataPath(), "table/table.urdf")
ROBOT_URDF_MODEL_SUCTION=os.path.join(FILE_PATH,"Models/Robots/UR5/urdf/ur5e.urdf")
ROBOT_URDF_MODEL_BASIC_GRIPPER=os.path.join(FILE_PATH,"Models/Robots/UR5/urdf/ur5e_camera_gripper.urdf")
ROBOT_URDF_MODEL_ROBOTIQ_GRIPPER=os.path.join(FILE_PATH,"Models/Robots/UR5/urdf/ur5e_camera_robotiq_gripper.urdf")
ROBOT_URDF_MODEL_DIANA_SUCTION_GRIPPER=os.path.join(FILE_PATH,"Models/Robots/dianaV1_description/urdf/DianaV1_robot.urdf")
GRIPPER_URDF_MODEL=os.path.join(FILE_PATH,"Models/Robots/robotiq_arg85_description/robots/robotiq_arg85_description.URDF")

# --IMPORTING TABLE MODEL-- #
print("Importing Table ...")
table = p.loadURDF(TABLE_URDF_MODEL, [0.5, 0, -0.6300], [0, 0, 0, 1])
OBJ_DIR=os.path.join(FILE_PATH,"Models/Tray Objects")
os.chdir(OBJ_DIR)

# --IMPORT TRAY-- #
print("Importing Tray ...")
TRAY_POSITION=0.65
trayStartPosition=[TRAY_POSITION,0,0]
trayStartOrientation=p.getQuaternionFromEuler([0,0,0])
tray=p.loadURDF("tray/traybox.urdf",trayStartPosition,trayStartOrientation)  
print("Position and orientation of Tray")
print(p.getBasePositionAndOrientation(tray))

# --IMPORT EXTERNAL CAMERA-- #
print("Initializing camera ...")
viewMatrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition = [0,0,0], distance = 0.3, yaw = 90, pitch = -90, roll = 0, upAxisIndex = 2) 
projectionMatrix = p.computeProjectionMatrixFOV(fov = 40,aspect = 1,nearVal = 0.01,farVal = 1) # Far Value is for background, Near value is for objects
image_renderer = p.ER_BULLET_HARDWARE_OPENGL
pos = [TRAY_POSITION,0,0.80] 
ori = [0,0.7071067811865475,0,0.7071067811865476] # Quarternion for rotation around y axis
rot_matrix = p.getMatrixFromQuaternion(ori)
rot_matrix = np.array(rot_matrix).reshape(3, 3)
print(rot_matrix)
# Initial vectors
init_camera_vector = (1, 0, 0) # z-axis
init_up_vector = (0, 1, 0) # y-axis
# Rotated vectors
camera_vector = rot_matrix.dot(init_camera_vector)
up_vector = rot_matrix.dot(init_up_vector)
view_matrix_gripper = p.computeViewMatrix(pos, pos + 0.1 * camera_vector, up_vector)
def get_camera_image(itera):
	img = p.getCameraImage(200, 200, view_matrix_gripper, projectionMatrix,shadow=0, flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX, renderer=image_renderer)
	#plt.imshow(img[2])
	#plt.show()    
	FILE_PATH = os.path.dirname(os.path.realpath(__file__))
	directory='Cameraoutput/Dataset/images'
	path=os.path.join(FILE_PATH, directory)	
	imgName='image-'+str(itera)+'.png'
	path=os.path.join(path , imgName)
	print(path)
	im = Image.fromarray(img[2]) 
	im.save(path, '')



# --IMPORT RANDOM TRAY OBJECTS-- #
def random_object_import(NO_OBJ):
    for i in range(0,NO_OBJ):
        DEXNET_DIR=os.path.join(OBJ_DIR,"3dnet")
        os.chdir(DEXNET_DIR)
        random_obj=random.choice(os.listdir(DEXNET_DIR))
        drop_height=0.25
        drop_location_x=random.uniform(TRAY_POSITION-0.20, TRAY_POSITION+0.20)
        drop_location_y=random.uniform(-0.1, 0.1)
        random_color_r=random.uniform(0,1)
        random_color_g=random.uniform(0,1)
        random_color_b=random.uniform(0,1)
        shift = [0, -0.002, 0]
        meshScale = [1, 1, 1]
        visualShapeId=p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName=random_obj,
                                    rgbaColor=[random_color_r, random_color_g, random_color_b, 1],
                                    specularColor=[0.4, 0.4, 0],
                                    visualFramePosition=shift,
                                    meshScale=meshScale)
        collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName=random_obj,
                                          collisionFramePosition=shift,
                                          meshScale=meshScale)    
        body.append(p.createMultiBody(baseMass=1,
                          baseInertialFramePosition=[0, 0, 0],
                          baseCollisionShapeIndex=collisionShapeId,
                          baseVisualShapeIndex=visualShapeId,
                          basePosition=[drop_location_x,drop_location_y,drop_height],
                          useMaximalCoordinates=True))
        time.sleep(0.2)
def remove_objects():
   	for i in range(0,NO_OBJ):
		p.removeBody(body[i])

print("------------------------------------- \n")
print("CHOOSE THE OBJECT TYPES TO IMPORT ... \n")
print("1: Five Lego cubes \n")
print("2: Five Small soccer balls \n")
print("ANY KEY : Random Dex-Net objects \n")
print("------------------------------------- \n")
body=[]
NO_OBJ=5
inp=input()
if inp==1:
    for i in range(0,NO_OBJ):
        body.append(p.loadURDF("lego/lego.urdf",[0.5+0.05*i,0.15-0.1*i,0.5]))
	mode=0
elif inp==2:
    for i in range(0,NO_OBJ):
        body.append(p.loadURDF("sphere_small.urdf",[0.5+0.05*i,0.2-0.1*i,0.5]))
	mode=0
else:
    print("0: Test Mode \n")
    print("9: Dataset Generation Mode \n")
    mode=input()
    print("How many random Dex-net objects do you want to import in the tray ? \n")
    obj_num=input()
    NO_OBJ=int(obj_num)
    if mode==0:
    	random_object_import(NO_OBJ)
	get_camera_image(0)
    elif mode==9:
	print("How many images do you want to generate?\n")
	num=input()
	for i in range(1,num+1):
		random_object_import(NO_OBJ)
		# Heap settling time
		time.sleep(5) 
		get_camera_image(i)
		remove_objects()
	print("Dataset Generation complete ...")
    else :
	print("Wrong mode. Exiting ...")




def dataset_generation(num):
	for i in range(0,num):	
      		dataset_image = p.getCameraImage(200, 200, view_matrix_gripper, projectionMatrix,shadow=0, flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX, renderer=image_renderer)
		FILE_PATH = os.path.dirname(os.path.realpath(__file__))
	        directory='Cameraoutput/Dataset/images'
		path=os.path.join(FILE_PATH, directory)
		imgName='image-'+str(i+1)+'.png'
		path=os.path.join(path , imgName)
		print(path)
		im = Image.fromarray(img[2]) 
		im.save(path, '')


# --DEFINE THE CONTROL FUNCTIONS-- #   
def initialize_robot(choice):
            if choice==0:
                print("Initializing robot ...")
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
