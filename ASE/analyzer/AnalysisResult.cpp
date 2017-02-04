#include "AnalysisResult.h"

using namespace std;

namespace ASE
{
    AnalysisResult::AnalysisResult(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& object, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& reference, const std::vector<float>& distances,
                                   double scale, double objectArea, double referenceArea, double computationTime) :
        _objectCloud(object), _referenceCloud(reference), _distances(distances), _sortedDistances(distances), _objectArea(objectArea), _referenceArea(referenceArea), _scale(scale), _computationTime(computationTime)
    {
        sort(_sortedDistances.begin(), _sortedDistances.end(), less<double>());

        for (auto d : _distances)
            _avgDistance += d;

        _avgDistance /= _distances.size();
        _coverage = _objectArea / _referenceArea;
    }

    double AnalysisResult::getObjectCloudArea() const
    {
        return _objectArea;
    }

    double AnalysisResult::getReferenceCloudArea() const
    {
        return _referenceArea;
    }

    double AnalysisResult::getAreaCoverage() const
    {
        return _coverage;
    }

    double AnalysisResult::getObjectScale() const
    {
        return _scale;
    }

    double AnalysisResult::getComputationTime() const
    {
        return _computationTime;
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr AnalysisResult::getObjectCloud() const
    {
        return _objectCloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr AnalysisResult::getReferenceCloud() const
    {
        return _referenceCloud;
    }

    std::vector<float> AnalysisResult::getDistances() const
    {
        return _distances;
    }

    float AnalysisResult::getQuantilDistance(float q) const
    {
        int idx = static_cast<int>((_sortedDistances.size() - 1) * max(0.0f, min(1.0f, q)));
        return _sortedDistances[idx];
    }

    float AnalysisResult::getMedianDistance() const
    {
        return getQuantilDistance(0.5);
    }

    float AnalysisResult::getAverageDistance() const
    {
        return _avgDistance;
    }
}
