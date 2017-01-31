#include "Analyzer.h"
#include "MeshSampling.h"
#include "Computation.h"
#include <pcl/filters/uniform_sampling.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/gicp.h>
#include <pcl/surface/convex_hull.h>
#include "Locals.h"


namespace ASE
{
    typedef PointCloud<PointXYZ> Cloud;

    Analyzer::Analyzer(const AnalyzerConfig& config) : _config(config), _viewer(new Viewer(config)), _feedback(config.enableViewer && config.forceUserFeedback)
    {             
    }

    Analyzer::~Analyzer()
    {
    }

    AnalysisResult Analyzer::run(pcl::PointCloud<pcl::PointXYZ>& backgroundScene, pcl::PointCloud<pcl::PointXYZ>& objectScene, pcl::PolygonMesh& referenceModel)
    {                   
        auto sceneCloud = backgroundScene.makeShared();
        auto objectCloud = objectScene.makeShared();
        auto referenceCloud = _setupModel(referenceModel);

        _setupViewer();
        _positionClouds(sceneCloud, objectCloud, referenceCloud);
        
        _viewer->addCloud(sceneCloud, "scene", _config.visualizer.pointSize, _config.visualizer.cloudOpacity);
        _viewer->addCloud(objectCloud, "object", _config.visualizer.pointSize, _config.visualizer.cloudOpacity);
        _viewer->addCloud(referenceCloud, "reference", _config.visualizer.pointSize, _config.visualizer.cloudOpacity);
        
        _feedback.wait();
        
        _alignScenes(sceneCloud, objectCloud);

        _feedback.wait();

        _extractObject(sceneCloud, objectCloud);
        
        _feedback.wait();

        _alignObjects(objectCloud, referenceCloud);
        _computeResult(objectCloud, referenceCloud);
        
        _feedback.wait();

        return _computeResult(objectCloud, referenceCloud);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr Analyzer::_setupModel(pcl::PolygonMesh& mesh) const
    {
        MeshSampling sampling;
        return sampling.sample(mesh, _config.preprocess.referenceCloudSamplingSize, _config.preprocess.referenceCloudGridFilterSize);
    }

    void Analyzer::_setupViewer()
    {
        if (_config.enableViewer)
        {
            _viewer->start();
            _vout.setViewer(_viewer);
            _feedback.setViewer(_viewer->getViewer());
        }
    }

    void Analyzer::_positionClouds(Cloud::Ptr& scene, Cloud::Ptr& object, Cloud::Ptr& reference)
    {
        if (!_config.enableViewer)
            return;

        Affine3f m = Affine3f::Identity();
        auto bbScene = computeAABB<PointXYZ>(scene);
        m.translation() << bbScene.sizeX / 2 * 1.2, 0, -bbScene.cloud->at(0).z;
        transformPointCloud(*scene, *scene, m);

        m = Affine3f::Identity();
        auto bbObj = computeAABB<PointXYZ>(object);
        m.translation() << -bbObj.sizeX / 2 * 1.2, 0, -bbObj.cloud->at(0).z;
        transformPointCloud(*object, *object, m);
    }

    void Analyzer::_alignScenes(pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, pcl::PointCloud<pcl::PointXYZ>::Ptr& object)
    {        
        Cloud::Ptr sceneKeys(new Cloud);
        Cloud::Ptr objKeys(new Cloud);
        double ls = _config.preprocess.keypointsLeafSize;

        UniformSampling<PointXYZ> sampling;
        sampling.setInputCloud(scene);
        sampling.setRadiusSearch(ls);
        sampling.filter(*sceneKeys);
        
        sampling.setInputCloud(object);
        sampling.filter(*objKeys);

        _viewer->addCloud(sceneKeys, "scene_keys", _config.visualizer.keyPointSize, _config.visualizer.cloudOpacity);
        _viewer->addCloud(objKeys, "object_keys", _config.visualizer.keyPointSize, _config.visualizer.cloudOpacity);

        _feedback.wait();

        auto keypointsIcpCallback = [this, &sceneKeys, &scene](int n, double fitness, Matrix4f transform)
        {
            transformPointCloud(*scene, *scene, transform);
            _viewer->updateCloud(sceneKeys);
            _viewer->updateCloud(scene);
            return true;
        };

        runIcp(sceneKeys, objKeys, _config.preprocess.icpEpsilon, _config.preprocess.icpMaxIterations, _config.preprocess.icpStep, keypointsIcpCallback);
        
        auto fullCloudIcpCallback = [this, &sceneKeys, &scene](int n, double fitness, Matrix4f transform)
        {
            transformPointCloud(*sceneKeys, *sceneKeys, transform);
            _viewer->updateCloud(sceneKeys);
            _viewer->updateCloud(scene);
            return true;
        };

        runIcp(scene, object, _config.preprocess.icpEpsilon, _config.preprocess.icpMaxIterations, _config.preprocess.icpStep, fullCloudIcpCallback);

        _viewer->removeCloud(sceneKeys);
        _viewer->removeCloud(objKeys);
    }

    void Analyzer::_extractObject(pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, pcl::PointCloud<pcl::PointXYZ>::Ptr& object)
    {
        auto result = subtract(object, scene, _config.objectExtraction.subtractionTolerance);
        _viewer->removeCloud(scene);
        _viewer->removeCloud(object);
        _viewer->addCloud(result, "object_noisy");

        _feedback.wait();

        auto tmp = applyNoiseFilter(result, _config.objectExtraction.noiseFilterK, _config.objectExtraction.noiseStdDevMul);
        tmp = applyClusterFilter(tmp, _config.objectExtraction.clusterTolerance, _config.objectExtraction.clusterMinSize, _config.objectExtraction.clusterMaxSize);
        
        object = tmp;
        _viewer->removeCloud(result);
        _viewer->updateCloud(object);
    }

    void Analyzer::_alignObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr& object, pcl::PointCloud<pcl::PointXYZ>::Ptr& reference)
    {
        auto icpcb = [this, &object](int n, double fitness, Matrix4f transform)
        {            
            _viewer->updateCloud(object);
            return true;
        };

        GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> icpg;

        if (_config.objectAlignment.prealignToConvexHull)
        {
            auto convexHull = boost::make_shared<Cloud>();
            ConvexHull<PointXYZ> hull;
            hull.setInputCloud(reference);
            hull.reconstruct(*convexHull);
            runIcp(icpg, object, convexHull, _config.objectAlignment.icpgEpsilon, _config.objectAlignment.icpMaxIterations, _config.objectAlignment.icpStep, icpcb);
        }

        runIcp(icpg, object, reference, _config.objectAlignment.icpgEpsilon, _config.objectAlignment.icpMaxIterations, _config.objectAlignment.icpStep, icpcb);

        IterativeClosestPoint<PointXYZ, PointXYZ> icp;
        auto te = boost::make_shared<registration::TransformationEstimationSVDScale<PointXYZ, PointXYZ>>();
        icp.setTransformationEstimation(te);
        runIcp(icp, object, reference, _config.objectAlignment.icpEpsilon, _config.objectAlignment.icpMaxIterations, _config.objectAlignment.icpStep, icpcb);
    }

    AnalysisResult Analyzer::_computeResult(pcl::PointCloud<pcl::PointXYZ>::Ptr& object, pcl::PointCloud<pcl::PointXYZ>::Ptr& reference)
    {
        search::KdTree<PointXYZ> kdTree;
        kdTree.setInputCloud(reference);
        vector<float> distances(object->size());

        for (int i = 0; i < object->size(); i++)
        {
            vector<float> dist(1);
            vector<int> idx(1);
            kdTree.nearestKSearch(object->at(i), 1, idx, dist);
            distances[i] = sqrt(dist[0]);
        }

        auto resultCloudWithDistance = boost::make_shared<PointCloud<PointXYZRGB>>();
        double maxDist = 0;

        for (auto d : distances)
            if (d > maxDist)
                maxDist = d;

        for (int i = 0; i < object->size(); i++)
        {
            PointXYZRGB p;
            auto r = object->at(i);
            p.x = r.x;
            p.y = r.y;
            p.z = r.z;
            p.r = min(distances[i] / maxDist * 255, 255.0);
            p.b = 255 - min(distances[i] / maxDist * 255, 255.0);
            resultCloudWithDistance->push_back(p);
        }

        _viewer->removeCloud(object);
        _viewer->lock();
        auto viewer = _viewer->getViewer();
        viewer->addPointCloud(resultCloudWithDistance, "final_result");
        viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "final_result");
        viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.4, "reference");
        _viewer->unlock();

        AnalysisResult result(object, reference, distances);        
        return result;
    }
}
