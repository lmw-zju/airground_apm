# airground_apm
iris_airground with apm
#无人机仿真用的是PX4-Atupilot的包，魔改加了一个iris带wheels的模型，一个从solidworks导入的空地车模型（添加apm固件链接）
#启动apm固件仿真，有些参数可以自己修改，这里是针对iris型无人机，console允许交互
 sim_vehicle.py -v ArduCopter -f gazebo-iris --console
# 启动gazebo无人机
#启动apm和mavros的接口
roslaunch mavros_connect amp.launch
