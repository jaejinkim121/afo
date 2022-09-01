# afo

# Before running
# Make sure tty port user mode

## Change user mode
sudo chown srbl /dev/ttyUSB0
sudo chown srbl /dev/ttyACM0
sudo chown srbl /dev/ttyACM1
sudo chown srbl /dev/ttyACM2

# Need to make .sh file
#

# How to run 

source /home/srbl/catkin_ws/devel/setup.bash
roslaunch afo_launch afo_launch.launch

#
