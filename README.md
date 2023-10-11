# airground_apm
iris_airground with apm
#无人机仿真用的是PX4-Atupilot的包，魔改加了一个iris带wheels的模型，一个从solidworks导入的空地车模型（添加apm固件链接）
#在zshrc或者bashrc中的设置（根据自己文件夹的实际位置）
source ~/px4_ws/src/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/px4_ws/src/PX4-Autopilot ~/px4_ws/src/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/px4_ws/src/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/px4_ws/src/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ardupilot_gazebo
#启动apm固件仿真，有些参数可以自己修改，这里是针对iris型无人机，console允许交互
 sim_vehicle.py -v ArduCopter -f gazebo-iris --console
#apm使环境变量生效
. ~/.profile
#启动gazebo无人机
roslaunch px4 apm_mavros_wheel.launch
#启动apm和mavros的接口
roslaunch mavros_connect amp.launch
