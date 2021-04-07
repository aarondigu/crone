// Converts the PointCloud2 from the camera frame to the drone frame and publishes it


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h> 
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>


//sensor_msgs::PointCloud point_cloud;


sensor_msgs::PointCloud2 pcl2_camera;

void pcl2_cam_call(const sensor_msgs::PointCloud2ConstPtr& msg){
  pcl2_camera = *msg;
  pcl2_camera.header.stamp = ros::Time::now(); // to correct "future frame" issues
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle node;

  //FOR REALSENSE:
  ros::Subscriber pcl2_cam_sub = node.subscribe("camera/depth/color/points", 1, pcl2_cam_call);
  ros::Publisher pcl2_tf = 
    node.advertise<sensor_msgs::PointCloud2>("transformed_pcl2", 1);

  //ros::Publisher pcl_pub = 
  //  node.advertise<sensor_msgs::PointCloud>("pcl_base", 1);

  ros::Rate rate(20.0);

  tf::TransformListener listener;

  ros::Duration(1.0).sleep();
  ROS_INFO("Ready...");

  //ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("camera_link", "base_link",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    sensor_msgs::PointCloud2 pcl2_base;
    pcl_ros::transformPointCloud("base_link", pcl2_camera, pcl2_base, listener);
    pcl2_tf.publish(pcl2_base);

    ros::spinOnce();
    rate.sleep();
  }
  //ros::spin();
  return 0;
};
