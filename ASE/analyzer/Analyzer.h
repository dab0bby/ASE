// Analyzer.h - Contains declaration of Function class
#pragma once


#define _SCL_SECURE_NO_WARNINGS 1


#include "AnalysisResult.h"
#include "AnalyzerConfig.h"
#include "Viewer.h"
#include "FeedbackController.h"
#include "StopWatch.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/pcl_config.h>

#if PCL_VERSION_COMPARE(<, 1, 8, 0)
    #error Update your PCL
#endif


namespace ASE
{
    class Analyzer
    {
    public:
        ASE_ANALYZER_API explicit Analyzer(const AnalyzerConfig& config = AnalyzerConfig());
        ASE_ANALYZER_API ~Analyzer();

        ASE_ANALYZER_API AnalysisResult run(
            pcl::PointCloud<pcl::PointXYZ>& backgroundScene,
            pcl::PointCloud<pcl::PointXYZ>& objectScene,
            pcl::PolygonMesh& referenceModel);

    private:
        const AnalyzerConfig _config;
        std::shared_ptr<Viewer> _viewer;
        FeedbackController _feedback;
        double _finalScale = 1.0;
        std::shared_ptr<StopWatch> _stopWatch;

        pcl::PointCloud<pcl::PointXYZ>::Ptr _setupModel(pcl::PolygonMesh& mesh) const;
        void _setupViewer();
        void _positionClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, pcl::PointCloud<pcl::PointXYZ>::Ptr& object, pcl::PointCloud<pcl::PointXYZ>::Ptr& reference);
        void _alignScenes(pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, pcl::PointCloud<pcl::PointXYZ>::Ptr& object);
        void _extractObject(pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, pcl::PointCloud<pcl::PointXYZ>::Ptr& object);
        void _alignObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr& object, pcl::PointCloud<pcl::PointXYZ>::Ptr& reference);
        AnalysisResult _computeResult(pcl::PointCloud<pcl::PointXYZ>::Ptr& object, pcl::PointCloud<pcl::PointXYZ>::Ptr& reference, const pcl::PolygonMesh& referenceMesh);
        void _wait(const std::string& msg = "press a key to continue...");
    };
}
