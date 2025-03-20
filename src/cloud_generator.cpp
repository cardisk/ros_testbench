#include <ros/ros.h>
#include <ros/time.h>

// ROS Messages
#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.10/pcl/point_cloud.h>

// Custom stuff
#include "Point.h"

#define CLOUD_SIZE 50000
#define RANGE 20

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "cloud_generator");
    ros::NodeHandle nh;

    // Create a ROS publisher for the output point cloud
    ros::Publisher publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_generator", 1);

    // To limit the spin frequency
    // ros::Rate loop_rate(1);

    // Custom spinner
    while (ros::ok())
    {
        pcl::PointCloud<Point> cloud;

        for (int i = 0; i < CLOUD_SIZE; i++)
        {
            Point p(
                (1 + i) % RANGE, (2 + i) % RANGE, (3 + i) % RANGE, // x, y, z
                0, 1, static_cast<float>(ros::Time::now().toSec()) // intensity, ring, time
            );

            cloud.push_back(p);
        }
        
        pcl::PCLPointCloud2 ros_cloud;
        pcl::toPCLPointCloud2(cloud, ros_cloud);
        publisher.publish(ros_cloud);

        ros::spinOnce();

        // loop_rate.sleep();
    }

    return 0;
}