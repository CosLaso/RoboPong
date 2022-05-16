******************************************************************************************************************************************************

Setup of Working Envrionment

******************************************************************************************************************************************************

For the setup of this project there are quite a few packages that need to be installed so everything can work. Firsty you will need Ubuntu Environment 
18.01 with ROS melodic, and how this can be installed can be found at the following websites:

Installing Ubuntu DualBoot - https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/

Installing Ubuntu VMWare (used for this project) - https://ubuntu.com/tutorials/install-ubuntu-desktop-1804#1-overview

Setting up ROS Melodic - http://wiki.ros.org/melodic/Installation/Ubuntu

Creation of a catkin_ws - http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

After setting this up there are certain workspaces and packages that need to be installed by the terminal to get the software communicating to the 
hardware. Below wwill outline the setup for each.

******************************************************************************************************************************************************

Setup for Real Sense Camera

******************************************************************************************************************************************************

Install the ROS Distribution:

sudo apt-get install ros-$ROS_DISTRO-realsense2-camera

Install the Packages:

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

sudo apt-get install librealsense2-dkms

sudo apt-get install librealsense2-utils

Then reconnect the Intel Real Sense Camera and run: realsense-viewer to verify the installation

-	When launching the module run: roslaunch realsense2_camera rs_camera.launch

******************************************************************************************************************************************************

Setup for Dobot Workspace

******************************************************************************************************************************************************

Install the Packages:

sudo apt update

sudo apt upgrade

sudo apt install g++ git autogen autoconf build-essential cmake graphviz \ libboost-dev libboost-test-dev libgtest-dev libtool \ python3-sip-dev doxygen python3-sphinx pkg-config \ python3-sphinx-rtd-theme

git clone https://github.com/crayzeewulf/libserial

cd libserial

sudo ./compile.sh

cd build

sudo make install

sudo apt install libserialport0

cd

cd ~/catkin_ws/src

Clone the Dobot workspace repository:

git clone https://github.com/gapaul/dobot_magician_driver.git

cd ..

catkin_make

source devel/setup.bash

Setting up roscore ip address (in new terminal):

nano ~/.bashrc

export ROS_HOSTNAME=localhost				*add this line and the next line to the bottom of the script then exit and save

export ROS_MASTER_URI=http://localhost:11311

source ~/.bashrc

Setting up the udev rules:

sudo cp -i  ~/catkin_ws/src/dobot_magician_driver/supporting/43-dobot_magician.rules /etc/udev/rules.d

-	When launching the module run: roslaunch dobot_magician_driver dobot_magician.launch


