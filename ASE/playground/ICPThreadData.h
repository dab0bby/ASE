#pragma once


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class ICPThreadData
{
public:
    ICPThreadData();
    ~ICPThreadData();

    void shutdownThread();
    bool shutdownPending() const;
    bool newDataAvailable() const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr getData();
    void setData(pcl::PointCloud<pcl::PointXYZ>::Ptr& data);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr _data;
    bool _shutdown = false;
    bool _hasData = false;
    bool _isUpdating = false;

};

