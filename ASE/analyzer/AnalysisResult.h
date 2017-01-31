#pragma once

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PolygonMesh.h"
#include "Globals.h"


namespace ASE
{
    class AnalysisResult
    {
    public:
        ASE_ANALYZER_API
            AnalysisResult(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& object, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& reference, const std::vector<float>& distances, 
                double scale, double objectArea, double referenceArea, double computationTime);

        ASE_ANALYZER_API pcl::PointCloud<pcl::PointXYZ>::ConstPtr getObjectCloud() const;
        ASE_ANALYZER_API pcl::PointCloud<pcl::PointXYZ>::ConstPtr getReferenceCloud() const;
        ASE_ANALYZER_API std::vector<float> getDistances() const;

        ASE_ANALYZER_API float getQuantilDistance(float q) const;
        ASE_ANALYZER_API float getMedianDistance() const;
        ASE_ANALYZER_API float getAverageDistance() const;
        ASE_ANALYZER_API double getObjectCloudArea() const;
        ASE_ANALYZER_API double getReferenceCloudArea() const;
        ASE_ANALYZER_API double getAreaCoverage() const;
        ASE_ANALYZER_API double getObjectScale() const;
        ASE_ANALYZER_API double getComputationTime() const;


    private:
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr _objectCloud;
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr _referenceCloud;
        std::vector<float> _distances;
        std::vector<float> _sortedDistances;
        double _avgDistance;
        double _objectArea;
        double _referenceArea;
        double _coverage;
        double _scale;
        double _computationTime;
    };
}
