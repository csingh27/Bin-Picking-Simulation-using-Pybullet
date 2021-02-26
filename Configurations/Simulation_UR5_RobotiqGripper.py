import pybullet as p
import definitions as d
from attrdict import AttrDict
from collections import namedtuple
from datetime import datetime
import numpy as np
import os
import definitions as d
import matplotlib.pyplot as plt
FILE_PATH = os.path.dirname(os.path.realpath(__file__+"/../"))
Flags=p.URDF_USE_SELF_COLLISION
from PIL import Image
# --SETUP THE ROBOT-- #
print("Setting up the robot ...")
robot=p.loadURDF(d.ROBOT_URDF_MODEL,[0,0,0],[0, 0, 1, 0],flags=Flags) 
end_effector_index = 7
num_joints = p.getNumJoints(robot)
print(num_joints)
control_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint","finger_joint","right_outer_knuckle_joint"]
gripper_joints = ["left_inner_knuckle_joint", "right_inner_knuckle_joint", "left_inner_finger_joint" ,"right_inner_finger_joint"]
#control_joints_old_gripper = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint","left_gripper_motor","right_gripper_motor"]
joint_type_list = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
joint_info = namedtuple("jointInfo", ["id", "name", "type", "lowerLimit", "upperLimit", "maxForce", "maxVelocity", "controllable"])
joints = AttrDict()
for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    jointID = info[0]
    jointName = info[1].decode("utf-8")
    jointType = joint_type_list[info[2]]
    jointLowerLimit = info[8]
    jointUpperLimit = info[9]
    jointMaxForce = info[10]
    jointMaxVelocity = info[11]
    controllable = True if jointName in control_joints else False
    info = joint_info(jointID, jointName, jointType, jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity, controllable)
    if info.type == "REVOLUTE":
        p.setJointMotorControl2(robot, info.id, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    joints[info.name] = info
    print("No. of joints ...")
    print(num_joints)
    print("Joint info for you ...")
    print(info)
    
def motors_arm(sliders,choice,joint_grasp):
    while True: 
            img = p.getCameraImage(200, 200, d.view_matrix_gripper, d.projectionMatrix,shadow=0, flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX, renderer=d.image_renderer)
            #plt.imshow(img[2])
            #plt.show()  
            FILE_PATH = os.path.dirname(os.path.realpath(__file__+"/../"))
	    directory='Cameraoutput/RobotiqGripper'
	    path=os.path.join(FILE_PATH, directory)
            d.imgCount=d.imgCount+1	
            imgName='RobotiqGripper-'+str(d.imgCount)+'.png'
	    path=os.path.join(path , imgName)
            print(path)
            im = Image.fromarray(img[2]) 
	    im.save(path, '')
	    poses = []
            griposes=[]
            indexes = []
            grindexes = []
            forces = []
            griforces = []
            gripper_angles=[]
            #init_poses = [0, -d.math.pi/2, -d.math.pi/2, -d.math.pi/2, -d.math.pi/2, 0,0,0,0,0,0,0]
            init_poses = [0, -d.math.pi/2, -d.math.pi/2, -d.math.pi/2, -d.math.pi/2, 0,0,0]
            init_grip=[0,0,0.1,0.1]
            if choice==0:
                joint_angles=init_poses
                gripper_angles=init_grip
                mult=[1,1,1,1]
                d.time.sleep(2)
            if choice==1:
                joint_angles=end_effector_control(sliders)
                mult=[1,1,1,1]
                gripper_angles=init_grip
            elif choice==2:
                joint_angles=joint_control(sliders)
                joint_angles[7]=-1*joint_angles[6]
                gripper_angles=[joint_angles[6],joint_angles[7],joint_angles[6],joint_angles[7]]
                print("Gripper angles")
                print(gripper_angles)
                mult=[1,-1,-1,1]
                gripper_angles=np.multiply(mult,gripper_angles)
            elif choice==3:
                joint_angles=joint_grasp
                gripper_angles=init_grip
                mult=[1,1,1,1]
            joint_angles=list(joint_angles)
            print(joint_angles)
            for i, name in enumerate(control_joints):
                joint = joints[name]
                print(joint)
                indexes.append(joint.id)
                forces.append(joint.maxForce)
                poses.append(joint_angles[i])
            p.setJointMotorControlArray(
                        robot, indexes,
                        p.POSITION_CONTROL,
                        targetPositions=joint_angles,
                        targetVelocities=[0.1]*len(poses),
                        positionGains=[0.04]*len(poses), 
                        forces=forces)
            print(joints)
            print(gripper_angles)
            for j,name in enumerate(gripper_joints):
                joint = joints[name]
                grindexes.append(joint.id)
                griforces.append(joint.maxForce)
                griposes.append(gripper_angles[j])
            griforces[2]=24
            griforces[3]=24
            p.setJointMotorControlArray(
                        robot, grindexes,
                        p.POSITION_CONTROL,
                        targetPositions=gripper_angles,
                        targetVelocities=[0.1]*len(griposes),
                        positionGains=[0.04]*len(griposes), 
                        forces=griforces)
            collisions = p.getContactPoints()
            if len(collisions) > 0:
                print("[Collision detected!] {}".format(datetime.now()))   
            if choice==0 or choice==3:
                break
     
def end_effector_add_sliders():     
    print("Adding Sliders for end-effector control ...")
    sliders = []
    sliders.append(p.addUserDebugParameter("X", 0, 1, 0.4))
    sliders.append(p.addUserDebugParameter("Y", -1, 1, 0))
    sliders.append(p.addUserDebugParameter("Z", 0.3, 1, 0.4))
    sliders.append(p.addUserDebugParameter("Rx", -d.math.pi/2, d.math.pi/2, 0))
    sliders.append(p.addUserDebugParameter("Ry", -d.math.pi/2, d.math.pi/2, 0))
    sliders.append(p.addUserDebugParameter("Rz", -d.math.pi/2, d.math.pi/2, 0))
    return sliders
    
def end_effector_control(sliders):
    print("Reading Sliders for end-effector control ...")
    x = p.readUserDebugParameter(sliders[0])
    y = p.readUserDebugParameter(sliders[1])
    z = p.readUserDebugParameter(sliders[2])
    Rx = p.readUserDebugParameter(sliders[3])
    Ry = p.readUserDebugParameter(sliders[4])
    Rz = p.readUserDebugParameter(sliders[5])
    position=[x, y, z]
    orientation=[Rx, Ry, Rz]
    quaternion = p.getQuaternionFromEuler(orientation)
    lower_limits = [-d.math.pi]*6
    upper_limits = [d.math.pi]*6
    joint_ranges = [2*d.math.pi]*6
    #rest_poses = [0, -d.math.pi/2, -d.math.pi/2, -d.math.pi/2, -d.math.pi/2, 0,0,0,0,0,0,0]
    rest_poses = [0, -d.math.pi/2, -d.math.pi/2, -d.math.pi/2, -d.math.pi/2, 0,0,0]
    joint_angles = p.calculateInverseKinematics(
                    robot, end_effector_index, position, quaternion, 
                    jointDamping=[0.01]*6, upperLimits=upper_limits, 
                    lowerLimits=lower_limits, jointRanges=joint_ranges, 
                    restPoses=rest_poses
                    )
    joint_angles=list(joint_angles)
    joint_angles=joint_angles[:8]
    print(joint_angles) 
    return joint_angles

def joint_add_sliders():     
    print("Adding Sliders for joint control ...")
    sliders=[]
    sliders.append(p.addUserDebugParameter("Robot Joint1", -d.math.pi/2, d.math.pi/2, 0))
    sliders.append(p.addUserDebugParameter("Robot Joint2",-d.math.pi/2, d.math.pi/2, -d.math.pi/2))
    sliders.append(p.addUserDebugParameter("Robot Joint3", -d.math.pi/2, d.math.pi/2, -d.math.pi/2))
    sliders.append(p.addUserDebugParameter("Robot Joint4", -d.math.pi/2, d.math.pi/2, -d.math.pi/2))
    sliders.append(p.addUserDebugParameter("Robot Joint5", -d.math.pi/2, d.math.pi/2, -d.math.pi/2))
    sliders.append(p.addUserDebugParameter("Robot Joint6", -d.math.pi/2, d.math.pi/2, 0))
    sliders.append(p.addUserDebugParameter("Close Gripper", 0, 0.75, 0))    
    return sliders

def joint_control(sliders):
    print("Reading Sliders for joint control ...")
    j1 = p.readUserDebugParameter(sliders[0])
    j2 = p.readUserDebugParameter(sliders[1])
    j3 = p.readUserDebugParameter(sliders[2])
    j4 = p.readUserDebugParameter(sliders[3])
    j5 = p.readUserDebugParameter(sliders[4])
    j6 = p.readUserDebugParameter(sliders[5])
    j7 = p.readUserDebugParameter(sliders[6])
    j8=-1*j7    
    joint_angles=[j1,j2,j3,j4,j5,j6,j7,j8]
    #joint_angles=[j1,j2,j3,j4,j5,j6,j7,j8,j9,j10,j11,j12]
    return joint_angles   

def grasp_user_input():
                print("Getting updated Object positions ...")
                while True:
                    obj_pose=[]
                    for i in range(0,d.NO_OBJ):
                        obj_pose.append(p.getBasePositionAndOrientation(d.body[i]))
                        print("OBJECT %",i+1)
                        print(p.getBasePositionAndOrientation(d.body[i]))   
                    print("Enter the object number you want to move ... ")
                    j=input()
                    try:
                        num=int(j)
                    except ValueError:
                        print("That's not an integer!")
                        j=input()
                    if num in range(1,50):
                        # Moving towards Object- Gripper
                        d.time.sleep(2)
                        print("Moving towards object %",j)
                        pos1=list(obj_pose[num-1][0])
                        pos1[2]=0.5
                        quat1 = [0,0.7071067811865475,0,0.7071067811865476] # Quarternion for rotation around y axis
                        #quat1=list(obj_pose[num-1][1]) #orientation
                        print(pos1)
                        print(quat1)
                        print("\n")
                        grip_status=1
                        j=grasp_control(pos1,quat1,grip_status)
                        motors_arm(0,3,j)
                        
                        # Moving down with Closed Gripper
                        d.time.sleep(2)
                        pos1[2]=0.3
                        print("Moving down \n")
                        j=grasp_control(pos1,quat1,grip_status)
                        motors_arm(0,3,j)
                        
                        # Opening Gripper
                        d.time.sleep(2)
                        print("Opening Gripper \n")
                        grip_status=0
                        j=grasp_control(pos1,quat1,grip_status)
                        motors_arm(0,3,j)
                        
                        # Moving down with Open Gripper
                        d.time.sleep(2)
                        pos1[2]=0.15
                        print("Moving down \n")
                        j=grasp_control(pos1,quat1,grip_status)
                        motors_arm(0,3,j)
                        
                        # Closing Gripper
                        d.time.sleep(2)
                        print("Closing Gripper \n")
                        grip_status=1
                        j=grasp_control(pos1,quat1,grip_status)
                        motors_arm(0,3,j)
                        
                        # Moving up with Closed Gripper
                        d.time.sleep(2)
                        pos1[2]=0.5
                        print("Moving down \n")
                        j=grasp_control(pos1,quat1,grip_status)
                        motors_arm(0,3,j)
                        
                        # Moving the robout outside
                        d.time.sleep(2)
                        pos1[0]=0.25
                        pos1[1]=0.2
                        print("Moving outside the tray \n")
                        j=grasp_control(pos1,quat1,grip_status)
                        motors_arm(0,3,j)
                        
                        # Moving down with Closed Gripper
                        d.time.sleep(2)
                        pos1[2]=0.2
                        print("Moving down \n")
                        j=grasp_control(pos1,quat1,grip_status)
                        motors_arm(0,3,j)
                        
                        # Opening Gripper
                        d.time.sleep(2)
                        print("Opening Gripper \n")
                        grip_status=0
                        j=grasp_control(pos1,quat1,grip_status)
                        motors_arm(0,3,j)
                    else:
                        print("Enter a number from 1 to 50 \n")

def grasp_control(position,quaternion,grip_status):

         
        # Get updated positons and orientations of the bodies
        lower_limits = [-d.math.pi]*6
        upper_limits = [d.math.pi]*6
        joint_ranges = [2*d.math.pi]*6
        #rest_poses = [0, -d.math.pi/2, -d.math.pi/2, -d.math.pi/2, -d.math.pi/2, 0,0,0,0,0,0,0]
        rest_poses = [0, -d.math.pi/2, 0, -d.math.pi/2, -d.math.pi/2, 0,0,0]
        # IK is generated in favour of restPoses
        joint_angles = p.calculateInverseKinematics(
                        robot, end_effector_index, position, quaternion, 
                        jointDamping=[0.01]*6, upperLimits=upper_limits, 
                        lowerLimits=lower_limits, jointRanges=joint_ranges, 
                        restPoses=rest_poses
                    )
        joint_angles=list(joint_angles)
        joint_angles=joint_angles[:8]
        print(joint_angles)
        if grip_status==0:
            joint_angles[6]=-0.3
        elif grip_status==1:
            joint_angles[6]=0.75
        return joint_angles
         
