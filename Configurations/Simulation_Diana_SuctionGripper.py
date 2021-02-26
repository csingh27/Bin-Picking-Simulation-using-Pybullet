import pybullet as p
from datetime import datetime
import definitions as d
from attrdict import AttrDict
from collections import namedtuple
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
d.ROBOT_URDF_MODEL_SUCTION=os.path.join(FILE_PATH,"Models/Robots/dianaV1_description/urdf/DianaV1_joint_limited_robot.urdf")
d.ROBOT_URDF_MODEL=d.ROBOT_URDF_MODEL_DIANA_SUCTION_GRIPPER
num_joints = p.getNumJoints(robot)
#control_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint","finger_joint","right_outer_knuckle_joint"]
control_joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6","joint7"]
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
	    FILE_PATH = os.path.dirname(os.path.realpath(__file__+"/../"))
	    directory='Cameraoutput/virtualSuction'
	    path=os.path.join(FILE_PATH, directory)
            d.imgCount=d.imgCount+1	
            imgName='SuctionGripper-'+str(d.imgCount)+'.png'
	    path=os.path.join(path , imgName)
            print(path)
            im = Image.fromarray(img[2]) 
	    #plt.imshow(im)
            #plt.show()  
	    im.save(path, '')
            poses = []
            indexes = []
            forces = []
            #init_poses = [0, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0,0,0,0,0,0,0]
            init_poses = [0, -d.math.pi/2, -d.math.pi/2, -d.math.pi/2, -d.math.pi/2, 0,0]
            if choice==0:
                joint_angles=init_poses
                d.time.sleep(2)
            if choice==1:
                joint_angles=end_effector_control(sliders)
            elif choice==2:
                joint_angles=joint_control(sliders)
            elif choice==3:
                joint_angles=joint_grasp
            #joint_angles=list(joint_angles)
            #joint_angles[7]=-1*joint_angles[6]
            print(joint_angles)
            for i, name in enumerate(control_joints):
                print(name,"\n")
                print(i,"\n")
                joint = joints[name]
                poses.append(joint_angles[i])
                indexes.append(joint.id)
                forces.append(joint.maxForce)
            #forces[6]=24
            #forces[7]=24
            p.setJointMotorControlArray(
                        robot, indexes,
                        p.POSITION_CONTROL,
                        targetPositions=joint_angles,
                        targetVelocities=[0.1]*len(poses),
                        positionGains=[0.04]*len(poses), 
                        forces=forces)           
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
    #rest_poses = [0, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0,0,0,0,0,0,0]
    rest_poses = [0, -d.math.pi/2, -d.math.pi/2, -d.math.pi/2, -d.math.pi/2, 0]
    joint_angles = p.calculateInverseKinematics(
                    robot, end_effector_index, position, quaternion, 
                    jointDamping=[0.01]*6, upperLimits=upper_limits, 
                    lowerLimits=lower_limits, jointRanges=joint_ranges, 
                    restPoses=rest_poses
                    )
    joint_angles=list(joint_angles)
    print(joint_angles)
    return joint_angles

def joint_add_sliders():     
    print("Adding Sliders for joint control ...")
    sliders=[]
    sliders.append(p.addUserDebugParameter("Joint1", -d.math.pi/2, d.math.pi/2, 0))
    sliders.append(p.addUserDebugParameter("Joint2",-d.math.pi/2, d.math.pi/2, -d.math.pi/2))
    sliders.append(p.addUserDebugParameter("Joint3", -d.math.pi/2, d.math.pi/2, -d.math.pi/2))
    sliders.append(p.addUserDebugParameter("Joint4", -d.math.pi/2, d.math.pi/2, -d.math.pi/2))
    sliders.append(p.addUserDebugParameter("Joint5", -d.math.pi/2, d.math.pi/2, -d.math.pi/2))
    sliders.append(p.addUserDebugParameter("Joint6", -d.math.pi/2, d.math.pi/2, 0))
    sliders.append(p.addUserDebugParameter("Joint7", -d.math.pi/2, d.math.pi/2, 0))
    
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
    #j8=-1*j7    
    joint_angles=[j1,j2,j3,j4,j5,j6,j7]
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
                    if num in range(1,6):
                        # Moving towards Object- Gripper
                        d.time.sleep(2)
                        print("Moving towards object %",j)
                        pos1=list(obj_pose[num-1][0])
                        pos1[2]=0.1
                        quat1 = [0,0.7071067811865475,0,0.7071067811865476] # Quarternion for rotation around y axis
                        #quat1 = [0,0.7071067811865475,0,0.7071067811865476] # Quarternion for rotation around y axis
                        #quat1=list(obj_pose[num-1][1]) #orientation
                        print(pos1)
                        print(quat1)
                        print("\n")
                        j=grasp_control(pos1,quat1)
                        motors_arm(0,3,j)
                        
                        # Moving down with Closed Gripper
                        d.time.sleep(2)
                        pos1[2]=0.05
                        print("Moving down \n")
                        j=grasp_control(pos1,quat1)
                        motors_arm(0,3,j)
                        
                        d.time.sleep(2)
                        # Picking Object
                        
                        #pos=p.getJointInfo()
                        pos=list(obj_pose[num-1][0])
                        quat=list(obj_pose[num-1][1]) #orientation                           
                        end_pos=[]
                        end_ori=[]    
                        for i in range(0,8):
                            a=[]
                            a=p.getLinkState(robot,i)
                            print("Pose ",i,"\n")
                            a=list(a)
                            print(a)
                            print("\n")
                            a0=list(a[0])
                            a1=list(a[1])
                            end_pos.append(a0)
                            end_ori.append(a1)
                            print(end_pos[i])
                            print(end_ori[i])
                        
                        p.addUserDebugLine(end_pos[1],end_pos[2])
                        p.addUserDebugLine(end_pos[2],end_pos[3])
                        p.addUserDebugLine(end_pos[3],end_pos[4])
                        p.addUserDebugLine(end_pos[4],end_pos[5])
                        p.addUserDebugLine(end_pos[5],end_pos[6])
                        p.addUserDebugLine(end_pos[6],end_pos[7])


                        ori1eul=p.getEulerFromQuaternion(end_ori[1])
                        ori2eul=p.getEulerFromQuaternion(end_ori[2])
                        ori3eul=p.getEulerFromQuaternion(end_ori[3])
                        ori4eul=p.getEulerFromQuaternion(end_ori[4])
                        ori5eul=p.getEulerFromQuaternion(end_ori[5])
                        ori6eul=p.getEulerFromQuaternion(end_ori[6])
                        ori7eul=p.getEulerFromQuaternion(end_ori[7])
                        ori0eul=p.getEulerFromQuaternion(end_ori[0])
                        
                        print(ori0eul)
                        print(ori1eul)
                        print(ori2eul)
                        print(ori3eul)
                        print(ori4eul)
                        print(ori5eul)
                        print(ori6eul)
                        print(ori7eul)
                        print("Object position : ",pos)
                        print("End Effector Position w.r.t. ground link: ",end_pos[7],"\n")
                        pos=np.subtract(end_pos[7],pos)
                        print("Object position w.r.t. end effector ",pos,"\n")
                        pos[1]=pos[1]+0.11
                        pos[2]=0
                        pos[0]=0
                        print("Object orientation : ",quat)
                        print("End Effector Orientation w.r.t. ground link : ",end_ori[7],"\n")
                        # End Effector position is w.r.t. the previous link. Need to multiply all Transformation matrices.
                        #end_ori=[0,0,1,0]
                        end_ori_Eul=list(p.getEulerFromQuaternion(end_ori[7]))
                        #end=end_ori_Eul
                        #x=end[0]
                        #y=end[1]
                        #z=end[2]
                        #end_ori_Eul=[z,-1*x,-1*y]
                        #quat_Eul=list(p.getEulerFromQuaternion(quat))
                        #quat_Eul=np.add(quat_Eul,[0,0,1.57])
                        #sum_Eul=np.subtract(quat_Eul,end_ori_Eul)
                        #quat=p.getQuaternionFromEuler(sum_Eul)
                        eTgpos,eTgori=p.invertTransform(end_pos[7],end_ori[7])
                        possi,quat=p.multiplyTransforms(eTgpos,eTgori,pos,quat)
                        quat=list(quat)
                        print("Object orientation w.r.t. end effector  : ",quat,"\n")
                        print("Activating suction ...")
                        c = p.createConstraint(
                                robot,
                                7,
                                d.body[num-1],
                                -1,
                                jointType=p.JOINT_FIXED,
                                jointAxis=[0, 0, 0],
                                parentFramePosition=pos,
                                childFramePosition=[0,0,0],
                                parentFrameOrientation=quat
                                )
                        d.time.sleep(2)
                        
                        # Moving up with Closed Gripper
                        d.time.sleep(2)
                        pos1[2]=0.5
                        print("Moving down \n")
                        j=grasp_control(pos1,quat1)
                        motors_arm(0,3,j)
                        
                        # Moving the robot outside
                        d.time.sleep(2)
                        pos1[0]=0.25
                        pos1[1]=0.2
                        print("Moving outside the tray \n")
                        j=grasp_control(pos1,quat1)
                        motors_arm(0,3,j)
                        
                        # Moving down with Closed Gripper
                        d.time.sleep(2)
                        pos1[2]=0.05
                        print("Moving down \n")
                        j=grasp_control(pos1,quat1)
                        motors_arm(0,3,j)
                        d.time.sleep(2)                        
                        p.removeConstraint(c)
                        
                    else:
                        print("Enter a number from 1 to 50 \n")

def grasp_control(position,quaternion):
        #obj_pose_nxt=obj_pose[num][0]+1
        #print(obj_pose_nxt)
        print("In the grasp function ...") 
        # Get updated positons and orientations of the bodies
        lower_limits = [-d.math.pi]*7
        upper_limits = [d.math.pi]*7
        joint_ranges = [d.math.pi]*7
        #rest_poses = [0, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0,0,0,0,0,0,0]
        rest_poses = [0, d.math.pi/2, 0, -d.math.pi/2, -d.math.pi/2, 0,0]
        # IK is generated in favour of restPoses
        joint_angles = p.calculateInverseKinematics(
                        robot, end_effector_index, position, quaternion, 
                        jointDamping=[0.01]*7, upperLimits=upper_limits, 
                        lowerLimits=lower_limits, jointRanges=joint_ranges, 
                        restPoses=rest_poses,maxNumIterations=10
                    )
        joint_angles=list(joint_angles)
        print(joint_angles)
        # --OLD GRIPPER-- #
        #if grip_status==1:

        #    joint_angles[6]=-0.00005
        #elif grip_status==0:
        #    if c:
        #        

        #        joint_angles[6]=0.05
        #return joint_angles
        return joint_angles
        
            
    
