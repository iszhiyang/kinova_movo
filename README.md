# Zhiyang Simulation:
The propose of this README is to do simulation with MOVO.
It is intended to be used with a remote PC that has ubuntu 16.04 LTS cleanly installed and has native hardware capable of running Gazebo simulation. It will configure the system and install all necessary updates to interface with MoVo. This process may change the environment if there is one already setup. Have a wonderful journey!

**Environment Requirement:**
1. Clean Ubuntu 16.04
2. ROS Kinetic desktop-full
## Installation and Use:
1. Install ROS and setup Kinova-movo
```
sudo apt-get install git
mkdir ~/movo_ws && cd movo_ws
git clone https://github.com/ALAN-NUS/kinova_movo.git src/
cd ~/movo_ws/src/movo_pc_setup 
chmod +x setup_remote_pc
./setup_remote_pc
```
**NOTE:** As the `setup_remote_pc` script runs, it will prompt you at different points. Pay attention to the question when it asks if you only want to run in simulation. This will take the setup procedure in a different direction. You may need to wait for several minutes during this stage.

2. Change the DISPLAY path
```
echo 'export DISPLAY=:1.0' >> .bashrc
```
3. Run simulation
```
cd ~/movo_ws/
catkin_make
source ~/.bashrc
roslaunch movo_demos zhiyang_demo.launch
```
After that, in the Gazebo GUI, there are MOVO robot and a table, then you can choose components of the Gearbox and put them on the table.

4. Launch torque sensor
```
rosrun movo_gazebo cartesianforce.py
```
to read the rostopic '/sim/...' and publish to '/movo/...'

Now the simulator and force sensor has been launched, you can move the robot in simulation by running any moveit command.

5. Demo
```
cd /home/*/movo_ws/src/movo_common/si_utils/src/si_utils
./yw_lx_demo_v5_py2servers
./yw_lx_demo_v5_py3clients
```
