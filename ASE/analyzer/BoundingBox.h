#pragma once

#include <pcl/point_types.h>

namespace ASE
{
    struct BoundingBox
    {
        double sizeX;
        double sizeY;
        double sizeZ;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    };
}
