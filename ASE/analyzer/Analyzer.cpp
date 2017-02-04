#include "Analyzer.h"
#include "MeshSampling.h"
#include "Computation.h"
#include "Globals.h"
#include "SmartStreamBuffer.h"

#include <pcl/filters/uniform_sampling.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/gicp.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/filter.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Dense>
#include <chrono>
#include <ostream>
#include <functional>

namespace ASE
{
    shared_ptr<SmartStreamBuffer> voutBuffer = make_shared<SmartStreamBuffer>(6);
    ostream vout(voutBuffer.get());

    typedef PointCloud<PointXYZ> Cloud;

    Analyzer::Analyzer(const AnalyzerConfig& config) : _config(config), _viewer(new Viewer(config)), _feedback(config.enableViewer && config.forceUserFeedback), _stopWatch(new StopWatch)
    {
    }

    Analyzer::~Analyzer()
    {
    }

    AnalysisResult Analyzer::run(pcl::PointCloud<pcl::PointXYZ>& backgroundScene, pcl::PointCloud<pcl::PointXYZ>& objectScene, pcl::PolygonMesh& referenceModel)
    {
        _setupViewer();

        vout << "preparing clouds..." << endl;

        _stopWatch->start();
        auto sceneCloud = backgroundScene.makeShared();
        auto objectCloud = objectScene.makeShared();
        auto referenceCloud = _setupModel(referenceModel);
        vector<int> dummy;

        vout << "resampling background scene from " << sceneCloud->size() << " to ";
        //resampleCloud<PointXYZ>(sceneCloud);
        removeNaNFromPointCloud(*sceneCloud, *sceneCloud, dummy);
        vout << sceneCloud->size() << " points" << endl;

        vout << "resampling object scene from " << objectCloud->size() << " to ";
        //resampleCloud<PointXYZ>(objectCloud);
        removeNaNFromPointCloud(*objectCloud, *objectCloud, dummy);
        vout << objectCloud->size() << " points" << endl;

        _stopWatch->stop();
        // position clouds next to each other, just for esthetical reasons 
        if (_config.enableViewer)
            _positionClouds(sceneCloud, objectCloud, referenceCloud);
        _stopWatch->start();

        _viewer->addCloud(sceneCloud, "scene", _config.visualizer.pointSize, _config.visualizer.cloudOpacity);
        _viewer->addCloud(objectCloud, "object", _config.visualizer.pointSize, _config.visualizer.cloudOpacity);
        _viewer->addCloud(referenceCloud, "reference", _config.visualizer.pointSize, _config.visualizer.cloudOpacity);

        _wait();

        _alignScenes(sceneCloud, objectCloud);

        _wait();

        _extractObject(sceneCloud, objectCloud);

        _wait();

        _alignObjects(objectCloud, referenceCloud);
        auto result = _computeResult(objectCloud, referenceCloud, referenceModel);

        _wait("press a key to exit...");

        return result;
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
            _feedback.setViewer(_viewer);
        }
    }

    void Analyzer::_positionClouds(Cloud::Ptr& scene, Cloud::Ptr& object, Cloud::Ptr& reference)
    {
        if (!_config.enableViewer)
            return;

        Affine3f m = Affine3f::Identity();
        auto bbScene = computeAABB<PointXYZ>(scene);
        m.translation() << -bbScene.sizeX / 2 * 1.2 , 0 , -bbScene.cloud->at(0).z;
        transformPointCloud(*scene, *scene, m);

        m = Affine3f::Identity();
        auto bbObj = computeAABB<PointXYZ>(object);
        m.translation() << bbObj.sizeX / 2 * 1.2 , 0 , -bbObj.cloud->at(0).z;
        transformPointCloud(*object, *object, m);
    }

    void Analyzer::_alignScenes(pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, pcl::PointCloud<pcl::PointXYZ>::Ptr& object)
    {
        Cloud::Ptr sceneKeys(new Cloud);
        Cloud::Ptr objKeys(new Cloud);
        double ls = _config.preprocess.keypointsLeafSize;

        vout << "computing key points: background... ";

        UniformSampling<PointXYZ> sampling;
        sampling.setInputCloud(scene);
        sampling.setRadiusSearch(ls);
        sampling.filter(*sceneKeys);

        vout << "done (" << sceneKeys->size() << " points), foreground... ";

        sampling.setInputCloud(object);
        sampling.filter(*objKeys);

        vout << "done (" << objKeys->size() << " points)" << endl;

        _viewer->addCloud(sceneKeys, "scene_keys", _config.visualizer.keyPointSize, _config.visualizer.cloudOpacity);
        _viewer->addCloud(objKeys, "object_keys", _config.visualizer.keyPointSize, _config.visualizer.cloudOpacity);

        _wait();

        vout << "aligning key points... ";

        auto keypointsIcpCallback = [this, &sceneKeys, &scene](int n, double fitness, Matrix4f transform)
            {
                transformPointCloud(*scene, *scene, transform);
                _viewer->updateCloud(sceneKeys);
                _viewer->updateCloud(scene);
                return true;
            };

        runIcp(sceneKeys, objKeys, _config.preprocess.icpEpsilon, _config.preprocess.icpMaxIterations, _config.preprocess.icpStep, keypointsIcpCallback);

        vout << "done" << endl << "aligning scenes... ";

        auto fullCloudIcpCallback = [this, &sceneKeys, &scene](int n, double fitness, Matrix4f transform)
            {
                transformPointCloud(*sceneKeys, *sceneKeys, transform);
                _viewer->updateCloud(sceneKeys);
                _viewer->updateCloud(scene);
                return true;
            };

        runIcp(scene, object, _config.preprocess.icpEpsilon, _config.preprocess.icpMaxIterations, _config.preprocess.icpStep, fullCloudIcpCallback);

        vout << "done" << endl;

        _viewer->removeCloud(sceneKeys);
        _viewer->removeCloud(objKeys);
    }

    void Analyzer::_extractObject(pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, pcl::PointCloud<pcl::PointXYZ>::Ptr& object)
    {
        vout << "extracting target object... " << endl << "subtracting (tolerance=" << _config.objectExtraction.subtractionTolerance * 1000 << "mm)... ";
        auto result = subtract(object, scene, _config.objectExtraction.subtractionTolerance);
        vout << "done" << endl;

        _viewer->removeCloud(scene);
        _viewer->removeCloud(object);
        _viewer->addCloud(result, "object_noisy");

        _wait();

        vout << "applying noise filter: K=" << _config.objectExtraction.noiseFilterK << " StdDevMul=" << _config.objectExtraction.noiseStdDevMul << "... ";
        auto tmp = applyNoiseFilter(result, _config.objectExtraction.noiseFilterK, _config.objectExtraction.noiseStdDevMul);
        vout << "done" << endl;
        vout << "finding clusters: Tolerance=" << _config.objectExtraction.clusterTolerance << " MinSize=" << _config.objectExtraction.clusterMinSize
            << " MaxSize=" << _config.objectExtraction.clusterMaxSize << "... ";
        tmp = applyClusterFilter(tmp, _config.objectExtraction.clusterTolerance, _config.objectExtraction.clusterMinSize, _config.objectExtraction.clusterMaxSize);
        vout << "done" << endl;

        object = tmp;
        _viewer->removeCloud(result);
        _viewer->addCloud(object, "object", _config.visualizer.pointSize, _config.visualizer.cloudOpacity);
    }

    void Analyzer::_alignObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr& object, pcl::PointCloud<pcl::PointXYZ>::Ptr& reference)
    {
        vout << "aligning object to reference..." << endl;
        int iterCnt = 0;
        double fitness = 0;

        centerClouds(*object, *reference);

        auto icpcb = [this, &object, &iterCnt, &fitness](int n, double f, Matrix4f transform)
            {
                fitness = f;
                iterCnt = n;
                _viewer->updateCloud(object);
                return true;
            };

        GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> icpg;

        if (_config.objectAlignment.prealignToBoundingBox)
        {
            vout << "performing ICP at bounding boxes of target object and reference model... ";
            auto obbRef = computeOBB<PointXYZ>(reference).cloud;
            auto obbObj = computeOBB<PointXYZ>(object).cloud;

            auto cb = [this, &object, &iterCnt, &fitness](int n, double f, Matrix4f transform)
                {
                    fitness = f;
                    iterCnt = n;
                    transformPointCloud(*object, *object, transform);
                    _viewer->updateCloud(object);
                    return true;
                };

            runIcp(obbObj, obbRef, _config.objectAlignment.icpgEpsilon, _config.objectAlignment.icpMaxIterations, _config.objectAlignment.icpStep, cb);
            vout << "done (iterations=" << iterCnt << " fitness=" << fitness * 1000 << "mm)" << endl;
        }

        if (_config.objectAlignment.prealignToConvexHull)
        {
            vout << "performing generalized ICP at target object and convex hull of reference model... ";
            auto convexHull = boost::make_shared<Cloud>();
            ConvexHull<PointXYZ> hull;
            hull.setInputCloud(reference);
            hull.reconstruct(*convexHull);
            runIcp(icpg, object, convexHull, _config.objectAlignment.icpgEpsilon, _config.objectAlignment.icpMaxIterations, _config.objectAlignment.icpStep, icpcb);
            vout << "done (iterations=" << iterCnt << " fitness=" << fitness * 1000 << "mm)" << endl;
        }

        vout << "performing generalized ICP at target object and reference model... ";
        runIcp(icpg, object, reference, _config.objectAlignment.icpgEpsilon, _config.objectAlignment.icpMaxIterations, _config.objectAlignment.icpStep, icpcb);
        vout << "done (iterations=" << iterCnt << " fitness=" << fitness * 1000 << "mm)" << endl;

        if (_config.objectAlignment.postalignByScale)
        {
            auto initialBB = computeOBB<PointXYZ>(object);

            IterativeClosestPoint<PointXYZ, PointXYZ> icp;
            auto te = boost::make_shared<registration::TransformationEstimationSVDScale<PointXYZ, PointXYZ>>();
            icp.setTransformationEstimation(te);

            vout << "performing scale invariant ICP at target object and reference model... ";
            runIcp(icp, object, reference, _config.objectAlignment.icpEpsilon, _config.objectAlignment.icpMaxIterations, _config.objectAlignment.icpStep, icpcb);
            auto bb = computeOBB<PointXYZ>(object);
            _finalScale = (bb.sizeX / initialBB.sizeX + bb.sizeY / initialBB.sizeY + bb.sizeZ / initialBB.sizeZ) / 3;
            vout << "done (iterations=" << iterCnt << " fitness=" << fitness * 1000 << "mm, scale=" << _finalScale << ")" << endl;
        }
    }

    AnalysisResult Analyzer::_computeResult(pcl::PointCloud<pcl::PointXYZ>::Ptr& object, pcl::PointCloud<pcl::PointXYZ>::Ptr& reference, const PolygonMesh& referenceMesh)
    {
        vout << "computing distances... ";
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
        //_viewer->lock();
        //auto viewer = _viewer->getViewer();
        //viewer->addPointCloud(resultCloudWithDistance, "final_result");
        //viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "final_result");
        //viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.4, "reference");
        //_viewer->unlock();

        vout << "done" << endl;

        // compute area
        boost::shared_ptr<Cloud> meshCloud(new Cloud);
        fromPCLPointCloud2(referenceMesh.cloud, *meshCloud);
        double refArea = 0;

        vout << "computing area of reference model...";

        for (auto triangle : referenceMesh.polygons)
        {
            auto a = meshCloud->at(triangle.vertices[0]),
                b = meshCloud->at(triangle.vertices[1]),
                c = meshCloud->at(triangle.vertices[2]);

            Vector3f v1(a.x, a.y, a.z),
                v2(b.x, b.y, b.z),
                v3(c.x, c.y, c.z);

            // compute size of triangle http://mathworld.wolfram.com/TriangleArea.html (division by 2 follows at the end)
            refArea += (v2 - v1).cross(v1 - v3).norm();
        }
        refArea /= 2;

        vout << "done (area=" << refArea * 1e6 << "sqmm)" << endl;
        vout << "triangulating object cloud...";

        PointCloud<PointNormal>::Ptr normalCloud(new PointCloud<PointNormal>);
        NormalEstimation<PointXYZ, PointNormal> normals;
        normals.setInputCloud(object);
        //normals.setKSearch(_config.analysis.objectNormalEstimationK);
        normals.setRadiusSearch(_config.analysis.objectNormalEstimationRadius);
        normals.compute(*normalCloud);

        pcl::concatenateFields(*object, *normalCloud, *normalCloud);

        vector<Vertices> polygon;
        GreedyProjectionTriangulation<PointNormal> triangulation;
        triangulation.setInputCloud(normalCloud);
        triangulation.setMu(_config.analysis.objectTriangulationKMultiplier);
        triangulation.setNormalConsistency(false);
        triangulation.setSearchRadius(_config.analysis.objectTriangulationSearchRadius);
        triangulation.reconstruct(polygon);

        vout << "done" << endl;

        if (_config.enableViewer)
        {
            _viewer->lock();
            auto viewer = _viewer->getViewer();
            //viewer->addPointCloudNormals<PointNormal>(normalCloud, 30, 0.03, "result_normals");
            viewer->addPolygonMesh<PointXYZRGB>(resultCloudWithDistance, polygon, "meshed_result");
            _viewer->unlock();
        }

        vout << "computing area of object's triangulation... ";

        double objArea = 0;

        for (auto triangle : polygon)
        {
            auto a = object->at(triangle.vertices[0]),
                b = object->at(triangle.vertices[1]),
                c = object->at(triangle.vertices[2]);

            Vector3f v1(a.x, a.y, a.z),
                v2(b.x, b.y, b.z),
                v3(c.x, c.y, c.z);

            // compute size of triangle http://mathworld.wolfram.com/TriangleArea.html (division by 2 follows at the end)
            objArea += (v2 - v1).cross(v1 - v3).norm();
        }
        objArea /= 2;

        vout << "done (area=" << objArea * 1e6 << "sqmm)" << endl;
        _stopWatch->stop();

        double time = _stopWatch->getTime().count() * 1e-9;

        AnalysisResult result(object, reference, distances, _finalScale, objArea, refArea, time);
        return result;
    }

    void Analyzer::_wait(const std::string& msg)
    {
        _stopWatch->stop();
        _feedback.wait(msg);
        _stopWatch->start();
    }
}
