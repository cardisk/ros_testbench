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

// TODO: Swap this includes to take the transformation 
// as a tf::Transform instead of a tf2::Transform.

// #include <tf/tf.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_listener.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Custom stuff
#include "Point.h"

// Globals
ros::Publisher publisher;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
std::unique_ptr<tf2_ros::TransformListener> tfListener;

void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    // Input cloud conversion to PCL type
    pcl::PointCloud<Point> input_cloud;
    pcl::PCLPointCloud2 temp_cloud;

    pcl_conversions::toPCL(*cloud, temp_cloud);
    pcl::fromPCLPointCloud2(temp_cloud, input_cloud);

    // Transformation from the TF-Tree
    tf2::Stamped<tf2::Transform> temp_tf;
    tf2::Transform lidar2cam;

    try {
        geometry_msgs::TransformStamped tf_msg = tfBuffer->lookupTransform(
            "zed2i_left_camera_optical_frame", 
            "velodyne", 
            ros::Time(0)
        );
        tf2::fromMsg(tf_msg, temp_tf);
        lidar2cam.setBasis(temp_tf.getBasis());
        lidar2cam.setOrigin(temp_tf.getOrigin());
    } catch (tf2::TransformException &e) {
        ROS_ERROR("%s", e.what());
        return;
    }

    // TODO: This code obtains a tf1 transformation from the TF-Tree
    // to make it easy transforming clouds with libPCL.
    // Unfortunately it errors out that the transformation required is
    // ahead of time somehow and this is not possible.

    // static tf::TransformListener listener;
    // tf::StampedTransform temp_tf;
    // tf::Transform lidar2cam;

    // try {
    //     // geometry_msgs::TransformStamped tf_msg = tfBuffer->lookupTransform(
    //     //     "zed2i_left_camera_optical_frame", 
    //     //     "velodyne", 
    //     //     ros::Time(0)
    //     // );
    //     // tf2::fromMsg(tf_msg, temp_tf);
    //     listener.lookupTransform(
    //         "zed2i_left_camera_optical_frame", 
    //         "velodyne", 
    //         ros::Time(0), 
    //         temp_tf
    //     );
         
    //     lidar2cam.setBasis(temp_tf.getBasis());
    //     lidar2cam.setOrigin(temp_tf.getOrigin());
    // } catch (tf2::TransformException &e) {
    //     ROS_ERROR("%s", e.what());
    //     return;
    // }

    pcl::PointCloud<Point> transformed_cloud;

    // TODO: https://github.com/ros-perception/perception_pcl/issues/222
    // pcl_ros::transformPointCloud(input_cloud, transformed_cloud, lidar2cam);

    // Filtering
    pcl::PointCloud<Point> output_cloud;

    for (auto p : transformed_cloud.points)
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
    ros::init(argc, argv, "cloud_transform_and_filter");
    ros::NodeHandle nh;

    tfBuffer = std::make_unique<tf2_ros::Buffer>();
    tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);

    // Subscribing to the cloud_generator topic
    ros::Subscriber sub = nh.subscribe("cloud_generator", 1, callback);

    // Creating a publisher to publish the filtered PointCloud2
    publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_transform_and_filter", 1);

    ros::spin();

    return 0;
}