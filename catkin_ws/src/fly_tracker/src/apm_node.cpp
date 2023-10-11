#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>


#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
// 一些全局变量
//PID control variables
double Kp,Ki,Ku;
XmlRpc::XmlRpcValue edge_x,edge_y;
XmlRpc::XmlRpcValue coeff;
std::vector<double>edge_getx,edge_gety,coeff_get;
// 悬空高度（追踪小车的高度）
const double h = 4;
// 调整高度的速度（上升或下降）
const double hv = 0.1;

// 控制无人机的速度
geometry_msgs::Twist velocity;

// 无人机当前的高度
double curH;

// 无人机是否已经稳定在空中的标志
bool start = false;



void do_H(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    curH = msg->pose.position.z;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

bool param_Init()
{
    for (size_t i = 0; i < edge_x.size(); ++i) 
{
    XmlRpc::XmlRpcValue tmp_value = edge_x[i];
    if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        edge_getx.push_back(double(tmp_value));
}
    for (size_t i = 0; i < edge_y.size(); ++i) 
{
    XmlRpc::XmlRpcValue tmp_value = edge_y[i];
    if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        edge_gety.push_back(double(tmp_value));
}
    for (size_t i = 0; i < coeff.size(); ++i) 
{
    XmlRpc::XmlRpcValue tmp_value = coeff[i];
    if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        coeff_get.push_back(double(tmp_value));
}
return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apm_node");
    ros::NodeHandle nh,pnh("~");
    setlocale(LC_ALL, "");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vec_pub = nh.advertise<geometry_msgs::Twist>
            ("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
   // ros::Subscriber img_sub = nh.subscribe<sensor_msgs::Image>("/iris/camera/rgb/image_raw", 10, doImg);

    ros::Subscriber height_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, do_H);
    bool is_param_ok=true;
    double target_x,target_y,target_z;
    is_param_ok &= pnh.getParam("Kp",Kp);
    is_param_ok &= pnh.getParam("Ki",Ki);
    is_param_ok &= pnh.getParam("Ku",Ku);
    is_param_ok &= pnh.getParam("edge_x",edge_x);
    is_param_ok &= pnh.getParam("edge_y",edge_y);
    is_param_ok &= pnh.getParam("target_x",target_x);
    is_param_ok &= pnh.getParam("target_y",target_y);
    is_param_ok &= pnh.getParam("target_z",target_z);

    is_param_ok &= pnh.getParam("coeff",coeff);
    if(!is_param_ok)
    {
        ROS_ERROR("param is not ok!!!");
        ros::shutdown();
        return 0;
    }
    if(param_Init()) ROS_INFO("param initialization is ok!!!");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = h;
    pose.pose.position.x = target_x;
    pose.pose.position.y = target_y;
    pose.pose.position.z = target_z;

    velocity.linear.x = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    bool takeoff = false;

    while(ros::ok()){
        if(!takeoff) {
            if( current_state.mode != "GUIDED" &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("GUIDED enabled");
                }
                last_request = ros::Time::now();
            }

            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }

            if( current_state.armed && 
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    takeoff = true;
                    ROS_INFO("Vehicle stabled");
                    // while(abs(curH-0.25)>0.1)
                    // {
                    //    local_pos_pub.publish(pose);
                    //    ros::spinOnce();
                    // }
                    start = true;
                    ROS_INFO("开始追踪...");
                    last_request = ros::Time::now();
                }
            local_pos_pub.publish(pose);
        } else {
            local_pos_pub.publish(pose);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

