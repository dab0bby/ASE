#pragma once

#ifdef ASE_ANALYZER_EXPORTS
    #define ASE_ANALYZER_API __declspec(dllexport)
#else
    #define ASE_ANALYZER_API __declspec(dllimport)
#endif

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PolygonMesh.h"

namespace ASE
{
    class AnalysisResult
    {
    public:
        AnalysisResult(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& object, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& reference, const std::vector<float>& distances) :
            _objectCloud(object), _referenceCloud(reference), _distances(distances)
        {            
        }

        ASE_ANALYZER_API pcl::PointCloud<pcl::PointXYZ>::ConstPtr getObjectCloud() const { return _objectCloud; }
        ASE_ANALYZER_API pcl::PointCloud<pcl::PointXYZ>::ConstPtr getReferenceCloud() const { return _referenceCloud; }
        ASE_ANALYZER_API std::vector<float> getDistances() const { return _distances; }


    private:
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr _objectCloud;
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr _referenceCloud;
        std::vector<float> _distances;

    };
}
