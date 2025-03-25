#include <ros/ros.h>

// ROS Messages
#include <sensor_msgs/PointCloud2.h>

// PCL Includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/common/transforms.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

// tf/tf2 Includes
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// Custom stuff
#include "Point.h"

// Globals
ros::Publisher publisher;
std::unique_ptr<tf::TransformListener> listener;

void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    // Input cloud conversion to PCL type
    pcl::PointCloud<Point> input_cloud;
    pcl::PCLPointCloud2 temp_cloud;

    pcl_conversions::toPCL(*cloud, temp_cloud);
    pcl::fromPCLPointCloud2(temp_cloud, input_cloud);

    // Transformation from the TF-Tree
    tf::StampedTransform temp_tf;
    tf::Transform lidar2cam;

    try {
        listener->lookupTransform(
            "zed2i_left_camera_optical_frame", 
            "velodyne", 
            ros::Time(0), 
            temp_tf
        );
         
        lidar2cam.setBasis(temp_tf.getBasis());
        lidar2cam.setOrigin(temp_tf.getOrigin());
    } catch (tf2::TransformException &e) {
        ROS_ERROR("%s", e.what());
        return;
    }

    pcl::PointCloud<Point> transformed_cloud;

    // PointCloud transformation
    pcl_ros::transformPointCloud(input_cloud, transformed_cloud, lidar2cam);

    // Filtering
    pcl::PointCloud<Point> output_cloud;

    for (auto p : transformed_cloud.points)
    {
        if (p.ring >= 5 && p.ring <= 10)
            output_cloud.push_back(p);
    }

    // Publishing filtered cloud
    pcl::PCLPointCloud2 ros_cloud;
    pcl::toPCLPointCloud2(output_cloud, ros_cloud);
    publisher.publish(ros_cloud);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "cloud_transform_and_filter");
    ros::NodeHandle nh;

    // Creating a tf::TransformListener
    listener = std::make_unique<tf::TransformListener>();

    // Subscribing to the cloud_generator topic
    ros::Subscriber sub = nh.subscribe("cloud_generator", 1, callback);

    // Creating a publisher to publish the filtered PointCloud2
    publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_transform_and_filter", 1);

    ros::spin();

    return 0;
}