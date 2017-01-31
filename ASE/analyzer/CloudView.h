#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

struct CloudView
{
    std::string name;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZ>::ConstPtr colorHandler;
    int pointSize = 1;
    float opacity = 1.0;
};
