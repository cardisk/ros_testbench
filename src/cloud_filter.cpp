#include <ros/ros.h>

// ROS Messages
#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.10/pcl/point_cloud.h>

// Custom stuff
#include "Point.h"

// Globals
ros::Publisher publisher;

void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    // Input cloud conversion to PCL type
    pcl::PointCloud<Point> input_cloud;
    pcl::PCLPointCloud2 temp;

    pcl_conversions::toPCL(*cloud, temp);
    pcl::fromPCLPointCloud2(temp, input_cloud);

    // Filtering
    pcl::PointCloud<Point> output_cloud;

    for (auto p : input_cloud.points)
    {
        if (p.x > 5 && p.x < 10 && p.y > 5 && p.y < 10)
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
    ros::init(argc, argv, "cloud_filter");
    ros::NodeHandle nh;

    // Subscribing to the cloud_generator topic
    ros::Subscriber sub = nh.subscribe("cloud_generator", 1, callback);

    // Creating a publisher to publish the filtered PointCloud2
    publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_filter", 1);

    ros::spin();

    return 0;
}