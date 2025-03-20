#ifndef POINT_H_
#define POINT_H_

// C++ stuff
#include <iostream>

#include <ros/ros.h>

// ROS Messages
#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>

// IMPORTANT: Custom PointT types must contain [x, y, z] vector. PCL_ADD_POINT4D does exactly this
class Point
{
public:
    Point() = default;
    Point(float x, float y, float z, float intensity, std::uint16_t ring, float time): 
            x(x), y(y), z(z), intensity(intensity), ring(ring), time(time) {};

    friend std::ostream& operator <<(std::ostream& os, const Point& p)
    {
        os << "Point { ";
        os << ".x = " << p.x << ", "; 
        os << ".y = " << p.y << ", "; 
        os << ".z = " << p.z << ", "; 
        os << ".intensity = " << p.intensity << ", "; 
        os << ".ring = " << p.ring << ", "; 
        os << ".time = " << p.time; 
        os << " }";

        return os;
    }

    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    float time;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(Point, 
    (float, x, x) 
    (float, y, y) 
    (float, z, z) 
    (float, intensity, intensity) 
    (std::uint16_t, ring, ring)
    (float, time, time) 
);

// PCL Explicit Template Instantiation goes here

#endif // POINT_H_