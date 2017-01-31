// Analyzer.h - Contains declaration of Function class
#pragma once

#include "AnalysisResult.h"
#include "AnalyzerConfig.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/pcl_config.h>
#include "Viewer.h"
#include "FeedbackController.h"


#if PCL_VERSION_COMPARE(<, 1, 8, 0)
#error Update your PCL
#endif

#ifdef ASE_ANALYZER_EXPORTS
    #define ASE_ANALYZER_API __declspec(dllexport)
#else
    #define ASE_ANALYZER_API __declspec(dllimport)
#endif

namespace ASE
{
    class Analyzer
    {
    public:
        explicit Analyzer(const AnalyzerConfig& config = AnalyzerConfig());
        ~Analyzer();

        ASE_ANALYZER_API AnalysisResult run(
            pcl::PointCloud<pcl::PointXYZ>& backgroundScene,
            pcl::PointCloud<pcl::PointXYZ>& objectScene,
            pcl::PolygonMesh& referenceModel);
        
    private:
        const AnalyzerConfig _config;
        boost::shared_ptr<Viewer> _viewer;     
        FeedbackController _feedback;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr _setupModel(pcl::PolygonMesh& mesh) const;
        void _setupViewer();
        void _positionClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, pcl::PointCloud<pcl::PointXYZ>::Ptr& object, pcl::PointCloud<pcl::PointXYZ>::Ptr& reference);
        void _alignScenes(pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, pcl::PointCloud<pcl::PointXYZ>::Ptr& object);
        void _extractObject(pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, pcl::PointCloud<pcl::PointXYZ>::Ptr& object);
        void _alignObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr& object, pcl::PointCloud<pcl::PointXYZ>::Ptr& reference);
        AnalysisResult _computeResult(pcl::PointCloud<pcl::PointXYZ>::Ptr& object, pcl::PointCloud<pcl::PointXYZ>::Ptr& reference);

    };
}
