**INSTALLATION**  

**Pybullet https://github.com/bulletphysics/bullet3**  
&#x2611;   
* pip3 install --user gym    
* pip3 install pybullet --upgrade --user  
* python3 -m pybullet_envs.examples.enjoy_TF_AntBulletEnv_v0_2017may  
* python3 -m pybullet_envs.examples.enjoy_TF_HumanoidFlagrunHarderBulletEnv_v1_2017jul  
* python3 -m pybullet_envs.deep_mimic.testrl --arg_file run_humanoid3d_backflip_args.txt  
* git clone https://github.com/Microsoft/vcpkg.git  
* cd vcpkg  
* ./bootstrap-vcpkg.sh  
* ./vcpkg integrate install  
* vcpkg install bullet3  
 
**NDDS https://github.com/NVIDIA/Dataset_Synthesizer**  
&#9746; No space for Unreal Engine  
Installation :  
a) Install Git-LFS Dependencies   
* *sudo apt install git*  
* *git --version*
* Install mergetool kdiff3: *sudo apt install kdiff3*  
* *kdiff3 --version*  
* Install mono to run git extensions (https://github.com/gitextensions/gitextensions/wiki/How-To:-run-Git-Extensions-on-Linux)  
* *sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF*
* *echo "deb http://download.mono-project.com/repo/debian wheezy main" | sudo tee /etc/apt/sources.list.d/mono-xamarin.list*  
* *sudo apt update*  
* *sudo apt install mono-complete*  
* *mono --version*  
* Install GitExtension  from https://sourceforge.net/projects/gitextensions/files/latest/download  

b) Install Git-LFS  
* *curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash*  
* *sudo apt-get install git-lfs*

c) Clone the repository  
* *git lfs clone https://github.com/NVIDIA/Dataset_Synthesizer.git*  
* *cd Dataset_Synthesizer*  
* *git lfs install*  
NOTE : LFS files not working. Download the repository from this link instead : https://github.com/NVIDIA/Dataset_Synthesizer/releases/download/1.2.2/ndds_1.2.2.zip  

d) Install Unreal Engine  
* Go to www.unrealengine.com and sign up  
* Go to personal in your name menu, select connections, connect GitHub account  
* Go to UnrealEngine Repository in EpicGames on GitHub  
* git clone https://github.com/EpicGames/UnrealEngine.git  
* In the root directory run *./Setup.sh*
* Then run *./GenerateProjectFiles.sh*  
* make  
* Run UE4  using *UE4Editor*  
https://docs.unrealengine.com/en-US/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/index.html  

*) Install
https://github.com/NVIDIA/Dataset_Utilities  

**https://berkeleyautomation.github.io/dex-net/#dexnet_4**  
&#9746;  
Test failed :  
FAILED (errors=1)  
Test failed: <unittest.runner.TextTestResult run=15 errors=1 failures=0>  
error: Test failed: <unittest.runner.TextTestResult run=15 errors=1 failures=0>  

* git clone https://github.com/BerkeleyAutomation/dex-net.git  
* sudo sh install.sh cpu python  
* sudo python setup.py test  

**http://gazebosim.org/** &#x2611;    
* Download Gazebo http://gazebosim.org/  
* http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo.sh
* sh gazebo.sh  
* Run gazebo by typing *gazebo* in the command line  

**https://berkeleyautomation.github.io/gqcnn/tutorials/tutorial.html

**GQCNN**  
https://berkeleyautomation.github.io/gqcnn/tutorials/tutorial.html
&#x2611; or &#9746;  

**Neurorobotics https://www.neurorobotics.net/**  
&#x2611;   
The Neurorobotics Platform is an Internet-accessible simulation system that allows the simulation of robots controlled by spiking neural networks.  

Using a docker container the entire package with all its dependencies can be downloaded  
https://www.neurorobotics.net/local_install.html
1. Install docker  
* sudo apt-get remove docker docker-engine docker.io containerd runc  
* sudo apt-get update  
* sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common  
* curl -fsSL https://download.docker.com/linux/debian/gpg | sudo apt-key add -  
* sudo apt-key fingerprint 0EBFCD88  
* sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/debian \
   $(lsb_release -cs) \
   stable"  
* sudo apt-get update  
* sudo apt-get install docker-ce docker-ce-cli containerd.io 
oR apt-cache madison docker-ce  
   sudo apt-get install docker-ce=18.06.3~ce~3-0~ubuntu docker-ce-cli=18.06.3~ce~3-0~ubuntu containerd.io  
* sudo docker run hello-world  
* Download the script from https://neurorobotics-files.net/index.php/s/HQJzj8fywKN8oxZ  
* chmod 755 nrp_installer.sh  
* ./nrp_install.sh install  
* http://localhost:9000/#/esv-private Or http://172.19.0.2:9000/#/esv-private   
* Login : nrpuser, Password : password  
* Clone "HoLLie arm manipulation demo experiment" 

**iGibson http://svl.stanford.edu/igibson/**  &#9746;   
* pip install gibson2  
* pip install PyYAML==5.1  
* python -m gibson2.envs.demo_interactive  

Segmentation fault (core dumped)  

OR  

* Download anaconda https://medium.com/@menuram1126/how-to-install-anaconda-on-ubuntu-16-04-538009ca7936  
* sha256sum Anaconda3-2020.02-Linux-x86_64.sh -u   
* bash /home/chandandeep/Documents/Agile\ Robots/Installations/Anaconda3-2020.02-Linux-x86_64.sh -u   
* Type *anaconda-navigator* in the command prompt  
* git clone https://github.com/StanfordVL/iGibson --recursive  
* cd iGibson  
* conda create -n py3-igibson python=3.6 anaconda  
* source activate py3-igibson  
* pip install -e .  
http://svl.stanford.edu/igibson/docs/installation.html  

**Isaac Sim : https://developer.nvidia.com/isaac-sim#man**  &#9746; 
*Install Nvidia-docker-container package*
* distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
* curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
* curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
* sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
* sudo systemctl restart docker
*Install Isaac-sim docker*
* https://developer.nvidia.com/isaac-sim/download  
* login to ngc.nvidia.com  
* Go to setup under your name menu  
* Click on get API key and "Generate API Key"   
* Copy username and password  
* ngc config set  
* sudo docker login nvcr.io  
* Enter the copied username and password  
* sudo docker pull nvcr.io/nvidia/isaac-sim:2020.1_preview  
* sudo docker run --gpus all --rm -e "ACCEPT_EULA=Y" -v /var/lib/isaac-sim/data:/root/.local/share/omniverse/Kit/2019.3 -v samples:/omniverse-kit/_build/linux-x86_64/release/extensions/extensions-other/omni/isaac/samples -p 47995-48012:47995-48012/udp -p 47995-48012:47995-48012/tcp -p 49000-49007:49000-49007/tcp -p 49000-49007:49000-49007/udp nvcr.io/nvidia/isaac-sim:2020.1_preview  