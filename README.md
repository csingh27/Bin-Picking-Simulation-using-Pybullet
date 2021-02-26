# Simulation- Bin Picking- Pybullet  

To run the Simulation : Run "python3 main.py"  

To navigate the PyBullet GUI :  
G- Hide all open windows  
Mouse Scrool- Zoom In/ Zoom Out  
CTRL+ Left Mouse Hold - Rotate  

STEPS :  
* Add UR5 Model with controls : ✔  
* Add Tray Model✔  
* Add Table Model✔  
* Add PyBullet Objects - 1 Type✔  
* Do heap simulation of objects✔  
* Add Gripper to UR5✔  
* Write code from scratch to control the UR5 Robot with sliders✔  
* Import OBJ Models to generate heap✔  
* Add Camera model external✔  
* Import Robotiq Gripper with UR5 Model✔  
* Perform Grasping Simulation on UR5 robot with Basic Gripper using Pybullet Objects✔  
* Perform Grasping Simulation on UR5 robot with Schmalz Gripper using Dex-Net Objects✔  
* Perform Grasping Simulation on UR5 robot with Robotiq Gripper using Dex-Net Objects  

DEPENDENCIES :  
1. Pybullet https://github.com/bulletphysics/bullet3  
```
Steps :   
pip3 install --user gym  
pip3 install pybullet --upgrade --user
pip install attrdict
*Testing the installation* :  
python3 -m pybullet_envs.examples.enjoy_TF_AntBulletEnv_v0_2017may  
python3 -m pybullet_envs.examples.enjoy_TF_HumanoidFlagrunHarderBulletEnv_v1_2017jul  
python3 -m pybullet_envs.deep_mimic.testrl --arg_file run_humanoid3d_backflip_args.txt  
In case the installation is not successful using the above steps, please follow the steps below :  
git clone https://github.com/Microsoft/vcpkg.git  
cd vcpkg  
./bootstrap-vcpkg.sh  
./vcpkg integrate install  
vcpkg install bullet3  
```  
2. Attrdict  
Steps :  
* pip3 install attrdict  
NOTE : Clone the following repo in /home/cs/GitHub/simulation-bin-picking-pybullet/Models/Robots before starting  
https://github.com/a-price/robotiq_arg85_description  

Running Examples from Reference Repositories :  

**Running ur5pybullet (Uses Kuka Model- With Gripper)**  
* cd ur5pybullet in the terminal  
* Run "python arm.py --mode xyz" in the terminal  
Or "python arm.py --mode motors"  
NOTE : Run this at the root if you add another repo inside it "rm -rf -d .git rmdir .git"  

**Running UR5Bullet (Uses UR5 Model- Without Gripper)**
* python3 UR5Sim.py

Running Demo Simulation :  
* Navigate to '/Demo Simulation'  
**To run KUKA Demo**  
* python3 enjoy_kuka_diverse_object_grasping.py  
**To run Panda Demo**  
* python3 loadpanda_grasp.py
