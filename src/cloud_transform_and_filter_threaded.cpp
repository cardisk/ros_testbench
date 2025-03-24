#include <vector>
#include <thread>
#include <future>

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

#include <tf_conversions/tf_eigen.h>

// Custom stuff
#include "Point.h"

// Globals
ros::Publisher publisher;
std::unique_ptr<tf::TransformListener> listener;

pcl::PointCloud<Point> filter_task(int start, int end, pcl::PointCloud<Point> &cloud, Eigen::Affine3f lidar2cam)
{
    pcl::PointCloud<Point> output_cloud;

    for (int i = start; i < end; i++)
    {
        Point transformed_point = pcl::transformPoint(cloud[i], lidar2cam);

        if (transformed_point.x > 5 && transformed_point.x < 10 && transformed_point.y > 5 && transformed_point.y < 10)
        {
            output_cloud.push_back(transformed_point);
        }
    }

    return output_cloud;
}

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

    Eigen::Affine3d eigen_double_lidar2cam;
    tf::transformTFToEigen(lidar2cam, eigen_double_lidar2cam);
    Eigen::Affine3f eigen_float_lidar2cam = eigen_double_lidar2cam.cast<float>();

    // Filtering
    pcl::PointCloud<Point> output_cloud;
    std::vector<std::future<pcl::PointCloud<Point>>> threads;

    // Assuming 50_000 points per thread.
    int len = input_cloud.points.size();
    int step_size = 50000;
    int offset = 0;

    // TODO: Test if threads dies when the task is done.

    // IMPORTANT: Pay attention to this answer because it says important things about std::future.
    // https://stackoverflow.com/questions/30810305/confusion-about-threads-launched-by-stdasync-with-stdlaunchasync-parameter
    while (offset < len)
    {
        int end = (offset + step_size > len) ? len : offset + step_size;

        std::future<pcl::PointCloud<Point>> filter_thread = std::async(
            &filter_task, 
            offset, 
            end,
            eigen_float_lidar2cam
        );

        threads.push_back(filter_thread);

        offset += step_size;
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