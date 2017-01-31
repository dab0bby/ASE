
#include "ICPThreadData.h"


ICPThreadData::ICPThreadData()
{
}


ICPThreadData::~ICPThreadData()
{
}

void ICPThreadData::shutdownThread()
{
    _shutdown = true;
}

bool ICPThreadData::shutdownPending() const
{
    return _shutdown;
}

bool ICPThreadData::newDataAvailable() const
{
    return _hasData;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ICPThreadData::getData()
{
    if (!_hasData)
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(nullptr);

    while (_isUpdating) { }
     
    _hasData = false;
    return _data;
}

void ICPThreadData::setData(pcl::PointCloud<pcl::PointXYZ>::Ptr& data)
{
    _isUpdating = true;
    _data = data;
    _isUpdating = false;
    _hasData = true;
}
