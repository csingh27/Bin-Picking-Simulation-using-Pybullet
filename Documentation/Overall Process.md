**Given** : Multiple objects are randomly arranged in a heap  

**Objective** : Sequentially grasp and transport each into a packing bin  

**Challenges** :  
* Reliable Robotic Grasping is challenging due to imprecision in sensing and actuation.   
* Difficult to infer object shapes from point clouds due to sensor noise, obstructions and occlusions when objects are placed in heaps  
* For deep learning model, time cost of collecting physical data too high  

**Ideas**: 
* We model bin picking with Partially Observable Markov Decision process : States of heaps, point cloud observations, rewards  
* Train on synthetic datasets of grasps and point clouds  
* It is possible to grasp a diverse set of objects from clutter using Deep CNNs.
* Consider modeling uncertainty during dataset generation to learn a policy for rapid bin picking from a single view point  
* Modelling bin picking as a sequence of 3D objects in heap with noisy point cloud observations using POMDP  
* Deep neural networks trained on large datasets can predict grasp positions to a very high accuracy  
* DexNet - Dataset of 6.7 million poin cloud objects  
* GQCNNs- Grasp Quality Convolutional Neural Networks : Rapidly predicts the success of grasps from depth images
* Grasps are specified as planar position, angle and depth of a gripper relative to and RGB-D Sensor  
* For grasping since the distance by which the gripper moves down and the opening of jaws and end effector pose of gripper jaws depend upon object position  
, orientation and dimensions, we can feed the object poses and dimensions + end effector poses to a network to predict new end effector poses and jaw opening  
* Robotiq Gripper has different modes of gripping depending on the dimensions of the object  

**Assumptions** :  
* Camera is mounted on a stationary position to capture the top view of the heap  
* Gripper motion can occlude camera view  
* Point cloud of one object selected as X for CNN
* Gripping position selected as y for CNN  
* We perform 3D Point Cloud Segmentation  

**Process** :  

*Simulation* , Tools : Gazebo, Isaac-Sim, iGibson, Neurorobotics, **Pybullet** 

1. Simulate the bin picking process  (http://git.ar.int/c.singh/simulation-bin-picking-pybullet)

*Machine Learning model* , Tools : Dex-Net 


2. Selecting a suitable Machine Learning model  

*Training dataset generation*  
Tools :  
NDDS (https://github.com/NVIDIA/Dataset_Synthesizer)    
Dex-Net (https://berkeley.app.box.com/s/w6bmvvkp399xtjpgskwq1cytkndmm7cn)  
CGD (http://pr.cs.cornell.edu/grasping/rect_data/data.php)  
Fraunhoffer (http://www.bin-picking.ai/en/dataset.html, https://owncloud.fraunhofer.de/index.php/s/AacICuOWQVWDDfP, https://arxiv.org/abs/1912.12125)  

3. Synthetic dataset generation (NDDS/ Dex-Net)  
    * Sampling from 3D CAD Models : Dex-Net  
    * Sampling from 3D CAD Models : NDDS  
    * Procedurally generated random shape models  
    * Generate heaps of models using Dynamic simulation of dropping action  
    Dynamic Simulation options : Pybullet, Newton Dynamics, Open Dynamics Engine (ODE), Vortex Dynamics (Dex-Net used this)  
    * Data collection using Dex-Net  
        I. Camera data  
        II. Robot/end effector data  
        III. Grasp feedback 
    * Domain randomization (random camerapositions, lighting conditions, object positions, and non-realistic textures.)  
        I. Physical parameter (end effector, object, collision, friction etc.)  
        II. Rendering (later)  
    OR 
3. Use Real-world dataset  
    * CGD (Cornell grasp Dataset)  

*Target Specification* , Tools : NDDS, Dex-Net   
4. 3D Point cloud
5. 3D Point cloud segmentation  
6. Specify target object to pick from the heap   

*Grasp Generation* , Tools : Pybullet, Isaac-Sim, iGibson, Gazebo    
7. Define a trajectory that reaches the object without occlusions  
8. Gripping action so as not to damage it  
9. Policy between Gripping action and point cloud  
10. Reward if successful transfer of object. 
11. Choose the heap state/ point of object as X and gripping position as y for a CNN only for which reward is 1  

*Feeding the heap and grasp dataset to a CNN* , Tools : Dex-Net (GQCNN used)  
12. Feed to CNN  
13. Predict trajectories for new heaps  

