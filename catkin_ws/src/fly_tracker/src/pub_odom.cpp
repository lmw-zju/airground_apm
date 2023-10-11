#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud, nav_msgs::Odometry>
      SyncPolicyCloudOdom;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;
#define JudgeValue 0.23

ros::Publisher cloud2_pub;

bool CloudIsQuad(Eigen::Vector3d p)
{
   double sum=pow(p[0],2)+pow(p[1],2)+pow(p[2],2);
   sum=pow(sum,0.5);
   if(sum<=JudgeValue)return true;
   else return false;
}
void CloudOdomCallback(const sensor_msgs::PointCloudConstPtr &pointcloud,
                                const nav_msgs::OdometryConstPtr &odom)
{
  sensor_msgs::PointCloud2 output_cloud2;
  sensor_msgs::PointCloud out_pointcloud=*pointcloud;
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Vector3d p_body2world(odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z);
  Eigen::Matrix3d livox2body;
  livox2body << 1.0,0.0,0.0,
              0.0,1.0,0.0,
              0.0,0.0,1.0;
  for (int i=0; i<out_pointcloud.points.size(); i++) {
		//std::cout << out_pointcloud.points[i].x << ", " << out_pointcloud.points[i].y << ", " << out_pointcloud.points[i].z << std::endl;
    Eigen::Vector3d tmp(out_pointcloud.points[i].x,out_pointcloud.points[i].y,out_pointcloud.points[i].z); 
    if(!CloudIsQuad(tmp)) tmp=body_r_m*livox2body*tmp+p_body2world;
    out_pointcloud.points[i].x=tmp[0];
    out_pointcloud.points[i].y=tmp[1];
    out_pointcloud.points[i].z=tmp[2];
	}
  sensor_msgs::convertPointCloudToPointCloud2(out_pointcloud,output_cloud2);
  cloud2_pub.publish(output_cloud2);
}



nav_msgs::Odometry genOdomFromGZ(const gazebo_msgs::ModelStates::ConstPtr &gz_msg_ptr, 
                                      const std::string &model_name)
{
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  auto it = std::find(gz_msg_ptr->name.begin(), gz_msg_ptr->name.end(), model_name);
  if (it == gz_msg_ptr->name.end()) {
    ROS_ERROR("[generateOdomFromGZ] Cannot find the target: %s", model_name.c_str());
    ros::shutdown();
    exit(1);
  }
  auto index = std::distance(gz_msg_ptr->name.begin(), it);
  odom.pose.pose = gz_msg_ptr->pose[index];
  odom.twist.twist = gz_msg_ptr->twist[index];
  odom.header.frame_id="world";
  return odom;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_odom");
  ros::NodeHandle nh("~");
  SynchronizerCloudOdom sync_cloud_odom;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud>> cloud_sub;
  
  nav_msgs::Odometry QUAD_ODOM;

  cloud2_pub=nh.advertise<sensor_msgs::PointCloud2>("/livox_point2",1);
  
  nav_msgs::Odometry QUAD_TMP_ODOM;
  const std::string GAZEBO_MODELS_TOPIC = "/gazebo/model_states";
  const std::string QUAD_MODEL_NAME="iris_wheel";
  ros::Publisher quad_odom_pub=nh.advertise<nav_msgs::Odometry>("/Odometry_imu",1);
  ros::Subscriber gazebo_model_states_sub = nh.subscribe<gazebo_msgs::ModelStates>(
      GAZEBO_MODELS_TOPIC, 1,
      [&QUAD_TMP_ODOM, QUAD_MODEL_NAME,quad_odom_pub](const gazebo_msgs::ModelStates::ConstPtr &gz_msg_ptr)
      {
          QUAD_TMP_ODOM = genOdomFromGZ(gz_msg_ptr, QUAD_MODEL_NAME);
          quad_odom_pub.publish(QUAD_TMP_ODOM);
      });
  while (!gazebo_model_states_sub.getNumPublishers())
  {
      ros::Rate(1).sleep();
      ROS_WARN("Waiting for connection");
  }

cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud>(nh, "/scan", 50));
odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/Odometry_imu", 100, ros::TransportHints().tcpNoDelay()));
sync_cloud_odom.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
  SyncPolicyCloudOdom(100), *cloud_sub, *odom_sub));
sync_cloud_odom->registerCallback(boost::bind(&CloudOdomCallback, _1, _2));

  ros::Rate rate(120);
  while(ros::ok())
  {
     ros::spinOnce();
     rate.sleep();
  }
}