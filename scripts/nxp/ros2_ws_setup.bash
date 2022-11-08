STARTING_PATH=${pwd}
cd /home/${USER}
wget https://swdownloads.analog.com/cse/aditof/tof_ros2_build_nxp/ros2_nxp_2022_10_12.tar
tar -xvf ros2_nxp_2022_10_12.tar
rm ros2_nxp_2022_10_12.tar
cd ros2_foxy/src
git clone https://github.com/analogdevicesinc/tof-ros2.git
cd ..
source install/local_setup.bash
colcon build 
cd ${STARTING_PATH}