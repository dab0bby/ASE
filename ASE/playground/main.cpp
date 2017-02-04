#define _SCL_SECURE_NO_WARNINGS 1

#include "MeshSampling.h"
#include "ICPThreadData.h"
#include "out.h"

#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/narf.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/from_meshes.h>
#include <pcl/pcl_config.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/filters/voxel_grid.h>

#include <math.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <future>

#if PCL_VERSION_COMPARE(<, 1, 8, 0)
    #error Update your PCL
#endif

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace visualization;

typedef PointCloud<PointXYZ> Cloud;
typedef PointCloud<PointNormal> NormalCloud;

#define endl (std::endl<char, char_traits<char>>)

enum ProgramState
{
    _BEGIN,
    LoadingClouds,
    CloudsLoaded,
    ICPCalib,
    FilteringNoise,
    ICPRef,
    _END,
    Idle,
    Continue,
};

template<typename PointT>
struct CloudView
{
    string name;
    typename PointCloud<PointT>::Ptr cloud;
    typename PointCloudColorHandler<PointT>::Ptr colorHandler;
    int pointSize;
};

template<>
struct CloudView<PointNormal>
{
    string name;
    PointCloud<PointNormal>::Ptr cloud;
    PointCloudColorHandler<PointNormal>::Ptr colorHandler;
    vector<Vertices> vertices;
    int pointSize;
};

const string dataRoot = "../sample data/";
const string outputRoot = "../output/";

const string backgroundPath = dataRoot + "scene.pcd";
const string scenePath = dataRoot + "object.pcd";
const string referencePath = dataRoot + "reference.stl";

const map<string, int> CLOUD_SIZE =
{
    {"background", 50000},
    {"scene", 55000},
    {"reference", 5000},
};

map<string, CloudView<PointXYZ>> cloudViews;
//map<string, CloudView<PointNormal>> normalViews;

ProgramState _state;

const int ICP_MAX_ITER = 500;
const int ICP_ITER_STEP = 1;
const double ICP_EPSILON_BACKGROUND = 4e-7;
const double ICP_EPSILON_OBJECT = 1e-12;
const int PC_RES = 70000;
const double BLUR_RAD = 0.05;
const int NOISE_MIN_HOTSPOT = 10;
const int NOISE_MAX_HOTSPOT = 30;
const int NOISE_MIN_DENSITY = 2;
const int NOISE_MAX_DENSITY = 20;
const double NOISE_DISTRIBUTION = 0.7;
const double KEYPOINTS_SAMPLE_SIZE = 0.1;

string currentIcpCloudName;
mutex viewerMutex;

const int COLOR_PRESETS[][3] =
{
    { 197,0,0 }, { 57,187,0 }, { 186,187,0 }, { 255,0,0 }, { 135,134,255 }, { 208,0,192 }, { 65,197,198 },
    { 94,0,0 }, { 22,87,0 },{ 83,83,0 }, { 29,15,255 }, { 124,0,114 }, { 34,114,114 }
};

random_device _rd;
mt19937 _mt;

NormalCloud::Ptr _referenceCloudWithNormals;
Cloud::Ptr _backgroundCloud, _sceneCloud, _referenceCloud, _referenceConvexCloud;
vector<Cloud::Ptr> _clouds { _backgroundCloud, _sceneCloud, _referenceCloud };

PCLVisualizer::Ptr viewer(new PCLVisualizer("3D Viewer"));
out vout(viewer, 6);

#define vout tmpout<out>(vout)

//void makeF32(cv::Mat& mat)
//{
//    cv::Mat tmp(mat.rows, mat.cols, CV_8UC1);
//    cv::transform(mat, tmp, cv::Matx13f(1 / 3.0, 1 / 3.0, 1 / 3.0));
//    mat.create(tmp.rows, tmp.cols, CV_32FC1);
//    tmp.convertTo(mat, CV_32F, 1.0 / 20);
//}

boost::shared_ptr<PointCloud<PointXYZ>> raycast(boost::shared_ptr<PointCloud<PointXYZ>>& cloud, Vector3f camera, Vector3f cameraDirection, Vector3f up = Vector3f(0, 0, 1),
    double fov = 70, double aspectratio = 16.0 / 9.0, int resX = 1920 / 2, int resY = 1080 / 2)
{
    boost::shared_ptr<PointCloud<PointXYZ>> result(new PointCloud<PointXYZ>);

    cameraDirection.normalize();
    up.normalize();

    double fovh = fov  * M_PI / 180.0;
    double fovv = 2 * atan(tan(fovh / 2) / aspectratio);
    double dh = fovh / resX;
    double dv = fovv / resY;

    octree::OctreePointCloudSearch<PointXYZ> octree(0.1);
    octree.setInputCloud(cloud);
    octree.defineBoundingBox();
    octree.addPointsFromInputCloud();

    Vector3f right = cameraDirection.cross(up).normalized();

    float total = resY;
    int progress = 0;
    int lastOutput = 0;

    for (int y = -resY / 2; y < resY / 2; y++)
    {
        for (int x = -resX / 2; x < resX / 2; x++)
        {
            double h = dh * x;
            double v = dv * y;
            Matrix3f rot;
            rot = AngleAxisf(v, right) * AngleAxisf(h, up);
            Vector3f direction = rot * cameraDirection;

            vector<int> indices(1);
            octree.getIntersectedVoxelIndices(camera, direction, indices, 1);

            if (indices.size() > 0)
                result->push_back(cloud->at(indices[0]));
        }

        progress++;
        auto p = static_cast<int>(progress / total * 100);
        if (p > lastOutput)
        {
            lastOutput = p;
            cout << "\rraycast: " << p << "%";
        }
    }

    return result;
}

double distance(PointXYZ a, Vector3f b)
{
    return abs((Vector3f(a.x, a.y, a.z) - b).norm());
}

void blur(PointCloud<PointXYZ>& cloud, double radius)
{
    uniform_real_distribution<> dist(-radius, radius);

    for (int i = 0; i < cloud.size(); i++)
    {
        double x = dist(_mt), y = dist(_mt), z = dist(_mt);
        cloud[i].x += x;
        cloud[i].y += y;
        cloud[i].z += z;
    }
}

// Adds noise to a cloud. The noise will create a random number of hotspots with a random number of points in each hotspot. Each hotspot has a gaussian point distribution.
void noise(PointCloud<PointXYZ>& cloud, int minHotspots, int maxHostspots, int minDensity, int maxDensity, double distribution)
{
    PointXYZ min, max;
    getMinMax3D(cloud, min, max);
    int hotspotCount = uniform_int_distribution<>(minHotspots, maxHostspots)(_mt);
    uniform_int_distribution<> denseDist(minDensity, maxDensity);
    uniform_real_distribution<> hotXDist(min.x, max.x);
    uniform_real_distribution<> hotYDist(min.y, max.y);
    uniform_real_distribution<> hotZDist(min.z, max.z);
    normal_distribution<> dist(0, distribution);

    for (; hotspotCount > 0; hotspotCount--)
    {
        PointXYZ hotspot(hotXDist(_mt), hotYDist(_mt), hotZDist(_mt));
        int count = denseDist(_mt);

        for (; count > 0; count--)
        {
            PointXYZ p(hotspot.x + dist(_mt), hotspot.y + dist(_mt), hotspot.z + dist(_mt));
            cloud.push_back(p);
        }
    }
}

template<typename PointT>
void randomTransform(PointCloud<PointT>& cloud, double radius)
{
    uniform_real_distribution<double> transdist(-radius, radius);
    uniform_real_distribution<double> rotdist(-M_PI / 8, M_PI / 8);
    Affine3f m = Affine3f::Identity();
    m.rotate(AngleAxisf(rotdist(_mt), Vector3f::UnitX()) * AngleAxisf(rotdist(_mt), Vector3f::UnitY()) * AngleAxisf(rotdist(_mt), Vector3f::UnitZ()));
    m.translation() << transdist(_mt), transdist(_mt), transdist(_mt);
    transformPointCloud(cloud, cloud, m);
}

void computeICP(PointCloud<PointXYZ>::Ptr& source, PointCloud<PointXYZ>::Ptr& target, shared_ptr<ICPThreadData> data)
{
    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(ICP_MAX_ITER);

    do
    {
        icp.align(*source);
        PointCloud<PointXYZ>::Ptr result(new PointCloud<PointXYZ>(*source));
        data->setData(result);
    } while (icp.hasConverged() && !data->shutdownPending());
}

Cloud::Ptr subtract(Cloud::Ptr& left, Cloud::Ptr& right, float precision = 0.1)
{
    Cloud::Ptr result(new Cloud);
    KdTreeFLANN<PointXYZ> rightTree;
    rightTree.setInputCloud(right);

    for(auto p : *left)
    {
        vector<int> index(1);
        vector<float> sqrDistance(1);

        // if no point of 'right' is near enough 'p' -> add it to the result
        if (rightTree.radiusSearch(p, precision, index, sqrDistance, 1) == 0)
            result->push_back(p);
    }

    return result;
}

void loadClouds()
{
    MeshSampling sampling;

    for (int i = 0; i < 3; i++)
    {
        string file = vector<string>{ backgroundPath, scenePath, referencePath }[i];

        vout << "loading \"" << file << "\" ... ";
        string name = boost::filesystem::path(file).stem().string();
        string extension = boost::filesystem::path(file).extension().string();
        int points = CLOUD_SIZE.count(name) ? CLOUD_SIZE.at(name) : PC_RES;

        if (extension == ".stl")
        {
            PolygonMesh mesh;
            io::loadPolygonFileSTL(file, mesh);

            if (i < 2)
            {
                // sample mesh to point cloud
                auto cloud = sampling.sample(mesh, points);
                vout << "sampled " << cloud->size() << " points ... ";

                // blur the point cloud
                blur(*cloud, 0.05);
                vout << "blured with " << BLUR_RAD << " radius ... ";

                // add some noise
                noise(*cloud, NOISE_MIN_HOTSPOT, NOISE_MAX_HOTSPOT, NOISE_MIN_DENSITY, NOISE_MAX_DENSITY, NOISE_DISTRIBUTION);
                vout << "randomly noised with [" << NOISE_MIN_HOTSPOT << ", " << NOISE_MAX_HOTSPOT << "[ hotspots, [" << NOISE_MIN_DENSITY << ", " << NOISE_MAX_DENSITY
                    << "[ density and " << NOISE_DISTRIBUTION << " distribution ... ";

                (i == 0 ? _backgroundCloud : _sceneCloud) = cloud;
            }
            else
            {
                _referenceConvexCloud = boost::make_shared<Cloud>();

                auto cloud = sampling.sample(mesh, 200000, 0.0025);
                ConvexHull<PointXYZ> hull;
                hull.setInputCloud(cloud);
                hull.reconstruct(*_referenceConvexCloud);

                vout << "sampled " << cloud->size() << " points ... ";

                _referenceCloud = cloud;

                NormalCloud::Ptr cwn(new NormalCloud);
                fromPCLPointCloud2(mesh.cloud, *cwn);
                features::computeApproximateNormals(*cwn, mesh.polygons, *cwn);
                _referenceCloudWithNormals = cwn;
            }
        }
        else if (extension == ".pcd")
        {
            Cloud::Ptr cloud(new Cloud);
            io::loadPCDFile(file, *cloud);
            vout << "point cloud has " << cloud->size() << " points ... ";

            double ls = 3;
            VoxelGrid<PointXYZ> grid;
            grid.setLeafSize(ls, ls, ls);
            grid.setInputCloud(cloud);
            grid.filter(*cloud);
            vout << "voxel grid filter reduces to " << cloud->size() << " points ... ";

            (i == 0 ? _backgroundCloud : _sceneCloud) = cloud;
        }
        else
        {
            vout << "error: unsupported file format";
        }

        vout << endl;
    }

    vout << endl;
}

PointCloud<PointXYZRGB>::Ptr convertCloud(Cloud& cloud)
{
    auto result = boost::make_shared<PointCloud<PointXYZRGB>>();
    result->reserve(cloud.size());

    for (auto p : cloud)
    {
        PointXYZRGB r;
        r.x = p.x;
        r.y = p.y;
        r.z = p.z;
        result->push_back(r);
    }

    return result;
}

void addCloudView(PointCloud<PointXYZ>::Ptr cloud, string name, int r, int g, int b, int pointSize = 1)
{
    if (cloudViews.find(name) != cloudViews.end())
        return;

    PointCloudColorHandlerCustom<PointXYZ>::Ptr colorHandler(new PointCloudColorHandlerCustom<PointXYZ>(cloud, r, g, b));

    viewerMutex.lock();
    viewer->addPointCloud(cloud, *colorHandler, name);
    viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, pointSize, name);
    viewerMutex.unlock();

    CloudView<PointXYZ> cv{ name, cloud, colorHandler, pointSize };
    cloudViews[cv.name] = cv;
}


void updateCloudView(string name)
{
    auto cv = cloudViews.at(name);
    viewerMutex.lock();
    viewer->updatePointCloud(cv.cloud, *cv.colorHandler, cv.name);
    viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, cv.pointSize, name);
    viewerMutex.unlock();
}

void updateCloudView(Cloud::ConstPtr cloud)
{
    for (auto cv : cloudViews)
    {
        if (cv.second.cloud == cloud)
        {
            updateCloudView(cv.second.name);
            return;
        }
    }
}

void removeCloudView(string name)
{
    viewerMutex.lock();
    viewer->removePointCloud(name);
    viewerMutex.unlock();
}

void removeCloudView(Cloud::ConstPtr cloud)
{
    for (auto cv : cloudViews)
    {
        if (cv.second.cloud == cloud)
        {
            removeCloudView(cv.second.name);
            return;
        }
    }
}


void keyboardCallback(const KeyboardEvent& e)
{
    if (e.keyUp() || e.isAltPressed() || e.isShiftPressed() || e.isCtrlPressed())
        return;

    if (_state == Idle)
        _state = Continue;
}

void wait()
{
    _state = Idle;
    while (_state != Continue)
    {
        if (!viewer->wasStopped())
            viewer->spinOnce(100);
        this_thread::sleep_for(20ms);
    }
}

template<typename... T>
void wait(future<T>&... futures)
{
    bool rdy = false;
    while (!rdy)
    {
        rdy = true;

        // rolls out to: rdy = rdy && f1.wait_for(20ms), rdy = rdy && f2.wait_for(20ms), rdy = rdy && f3.wait_for(20ms), ...
        initializer_list<int>{ (rdy = rdy && (std::forward<future<T>>(futures).wait_for(20ms) == future_status::ready), 0)... };

        if (!viewer->wasStopped())
        {
            viewerMutex.lock();
            viewer->spinOnce(100);
            viewerMutex.unlock();
        }
    }
}

template<typename ...T>
void wait(future<T>&&... f)
{
    wait(f);
}

template<typename SourceT, typename TargetT>
Affine3f centerClouds(PointCloud<SourceT>& source, PointCloud<TargetT>& target)
{
    Vector4f massSource, massTarget;
    compute3DCentroid(source, massSource);
    compute3DCentroid(target, massTarget);
    Vector4f delta = massTarget - massSource;

    Affine3f m = Affine3f::Identity();
    m.translation() << delta.x(), delta.y(), delta.z();
    transformPointCloud(source, source, m);
    return m;
}

PointXYZ size(PointXYZ a, PointXYZ b)
{
    return{ abs(a.x - b.x), abs(a.y - b.y), abs(a.z - b.z) };
}

void main()
{
    // init basic stuff
    _mt = mt19937(_rd());

    // init viewer
    viewer->registerKeyboardCallback(keyboardCallback);
    viewer->setShowFPS(false);
    //viewer->addCoordinateSystem();
    viewer->initCameraParameters();
    viewer->setBackgroundColor(0.25, 0.1, 0.3);


    // loading clouds async
    _state = LoadingClouds;
    {
        auto future = async(launch::async, loadClouds);
        wait(future);
    }

    _state = CloudsLoaded;

    // translate clouds that they can be shown side by side
    {
        // background
        Affine3f m = Affine3f::Identity();
        m.scale(0.001);
        m.translation() << -1.5, 0, -1;
        transformPointCloud(*_backgroundCloud, *_backgroundCloud, m);
        //randomTransform(*_backgroundCloud, 0.5);
        addCloudView(_backgroundCloud, "background_cloud", COLOR_PRESETS[0][0], COLOR_PRESETS[0][1], COLOR_PRESETS[0][2], 1);

        // scene
        m = Affine3f::Identity();
        m.scale(0.001);
        m.translation() << 1.5, 0, -1;
        transformPointCloud(*_sceneCloud, *_sceneCloud, m);
        //randomTransform(*_sceneCloud, 0.5);
        addCloudView(_sceneCloud, "scene_cloud", COLOR_PRESETS[1][0], COLOR_PRESETS[1][1], COLOR_PRESETS[1][2], 1);

        // ref model
        addCloudView(_referenceCloud, "reference_cloud", COLOR_PRESETS[2][0], COLOR_PRESETS[2][1], COLOR_PRESETS[2][2], 2);
    }

    vout << "press any key to start background filtering" << endl;
    wait();

    _state = ICPCalib;
    vout << "finding key points ... ";
    Cloud::Ptr calibKeys(new Cloud);
    Cloud::Ptr sceneKeys(new Cloud);
    //Cloud::Ptr refKeys(new Cloud);

    {
        auto future1 =  async(launch::async,
        [&calibKeys]()
        {
            UniformSampling<PointXYZ> sampling;
            sampling.setInputCloud(_backgroundCloud);
            sampling.setRadiusSearch(KEYPOINTS_SAMPLE_SIZE);
            sampling.filter(*calibKeys);
        });

        auto future2 = async(launch::async,
        [&sceneKeys]()
        {
            UniformSampling<PointXYZ> sampling;
            sampling.setInputCloud(_sceneCloud);
            sampling.setRadiusSearch(KEYPOINTS_SAMPLE_SIZE);
            sampling.filter(*sceneKeys);
        });

        wait(future1, future2);

        vout << " found " << calibKeys->size() << " on background ... ";
        vout << " found " << sceneKeys->size() << " on scene" << endl;

        addCloudView(calibKeys, "background_keys", COLOR_PRESETS[3][0], COLOR_PRESETS[3][1], COLOR_PRESETS[3][2], 5);
        addCloudView(sceneKeys, "scene_keys", COLOR_PRESETS[4][0], COLOR_PRESETS[4][1], COLOR_PRESETS[4][2], 5);
    }

    wait();

    vout << "aligning key points ... ";

    {
        IterativeClosestPoint<PointXYZ, PointXYZ> icp;
        icp.setInputSource(calibKeys);
        icp.setInputTarget(sceneKeys);
        icp.setMaximumIterations(ICP_ITER_STEP);

        int iterCnt = 0;
        double fitness = numeric_limits<double>::max();
        auto start = chrono::high_resolution_clock::now();
        auto future = async(launch::async, [&icp, &iterCnt, &calibKeys, &sceneKeys, &fitness]()
        {
            auto m = centerClouds(*calibKeys, *sceneKeys);
            transformPointCloud(*_backgroundCloud, *_backgroundCloud, m);

            for (iterCnt = 0; iterCnt < ICP_MAX_ITER; iterCnt += ICP_ITER_STEP)
            {
                icp.align(*calibKeys);
                auto f = icp.getFitnessScore();

                if (fitness - f < ICP_EPSILON_BACKGROUND)
                    break;

                auto transform = icp.getFinalTransformation();
                transformPointCloud(*_backgroundCloud, *_backgroundCloud, transform);
                updateCloudView(calibKeys);
                updateCloudView(_backgroundCloud);
                fitness = f;
            }
        });

        wait(future);
        auto stop = chrono::high_resolution_clock::now();
        vout << chrono::duration_cast<chrono::milliseconds>(stop - start).count() / 1000.0f << "s, " << iterCnt << " iterations, fitness: " << fitness << endl;

        vout << "aligning clouds ... ";

        fitness = numeric_limits<double>::max();
        icp.setInputSource(_backgroundCloud);
        icp.setInputTarget(_sceneCloud);

        start = chrono::high_resolution_clock::now();
        future = async(launch::async, [&icp, &iterCnt, &calibKeys, &fitness]()
        {
            for (iterCnt = 0; iterCnt < ICP_MAX_ITER; iterCnt += ICP_ITER_STEP)
            {
                icp.align(*_backgroundCloud);
                auto f = icp.getFitnessScore();

                if (fitness - f < ICP_EPSILON_BACKGROUND)
                    break;

                auto transform = icp.getFinalTransformation();
                transformPointCloud(*calibKeys, *calibKeys, transform);
                updateCloudView(calibKeys);
                updateCloudView(_backgroundCloud);
                fitness = f;
            }
        });

        wait(future);
        stop = chrono::high_resolution_clock::now();
        vout << chrono::duration_cast<chrono::milliseconds>(stop - start).count() / 1000.0f << "s, " << iterCnt << " iterations, fitness: " << fitness << endl;
    }

    wait();
    Cloud::Ptr resultCloud;

    vout << "removing background ... ";
    {
        auto future = async(launch::async, subtract, _sceneCloud, _backgroundCloud, 0.02);
        wait(future);

        resultCloud = future.get();
        removeCloudView(calibKeys);
        removeCloudView(_backgroundCloud);
        removeCloudView(sceneKeys);
        removeCloudView(_sceneCloud);
        addCloudView(resultCloud, "result", COLOR_PRESETS[7][0], COLOR_PRESETS[7][1], COLOR_PRESETS[7][2], 2);
    }

    wait();

    vout << "removing noise ... ";
    {
        Cloud::Ptr tmpCloud(new Cloud);

        StatisticalOutlierRemoval<PointXYZ> noiseFilter;
        noiseFilter.setInputCloud(resultCloud);
        noiseFilter.setMeanK(100);
        noiseFilter.setStddevMulThresh(0.5);
        noiseFilter.filter(*tmpCloud);

        vout << "finding largest cluster ... " << endl;

        vector<PointIndices> clusterPoints;
        EuclideanClusterExtraction<PointXYZ> ec;
        ec.setClusterTolerance(0.02); // 2cm
        ec.setMinClusterSize(1);
        ec.setMaxClusterSize(100000);
        ec.setInputCloud(tmpCloud);
        ec.extract(clusterPoints);

        PointIndices pts;

        for (auto c : clusterPoints)
        {
            if (pts.indices.size() < c.indices.size())
                pts = c;
        }

        resultCloud->clear();
        resultCloud->reserve(pts.indices.size());

        for (auto i : pts.indices)
            resultCloud->push_back(tmpCloud->at(i));

        updateCloudView(resultCloud);
    }

    wait();

    {
        vout << "aligning object to reference model ... ";

        //{
        //    PointXYZ a, b, c, d, e, f, g, h, pos, s;
        //    Matrix3f rot;
        //    MomentOfInertiaEstimation<PointXYZ> moments;
        //    moments.setInputCloud(resultCloud);
        //    moments.compute();
        //    moments.getOBB(a, g, pos, rot);
        //    s = size(g, a);
        //    b = { a.x + s.x, a.y, a.z };
        //    c = { a.x + s.x, a.y, a.z + s.z };
        //    d = { a.x, a.y, a.z + s.z };
        //    e = { g.x - s.x, g.y, g.z - s.z };
        //    f = { g.x, g.y, g.z - s.z };
        //    h = { g.x, g.y, g.z };

        //    Cloud::Ptr tmp1(new Cloud);
        //    tmp1->reserve(8);
        //    tmp1->push_back(a);
        //    tmp1->push_back(b);
        //    tmp1->push_back(c);
        //    tmp1->push_back(d);
        //    tmp1->push_back(e);
        //    tmp1->push_back(f);
        //    tmp1->push_back(g);
        //    tmp1->push_back(h);

        //    Affine3f mat = Affine3f::Identity();
        //    mat.rotate(rot);
        //    mat.translation() << pos.x, pos.y, pos.z;
        //    transformPointCloud(*tmp1, *tmp1, mat);

        //    moments.setInputCloud(_referenceCloud);
        //    moments.compute();
        //    moments.getOBB(a, g, pos, rot);
        //    s = size(g, a);
        //    b = { a.x + s.x, a.y, a.z };
        //    c = { a.x + s.x, a.y, a.z + s.z };
        //    d = { a.x, a.y, a.z + s.z };
        //    e = { g.x - s.x, g.y, g.z - s.z };
        //    f = { g.x, g.y, g.z - s.z };
        //    h = { g.x, g.y, g.z };

        //    Cloud::Ptr tmp2(new Cloud);
        //    tmp2->reserve(8);
        //    tmp2->push_back(a);
        //    tmp2->push_back(b);
        //    tmp2->push_back(c);
        //    tmp2->push_back(d);
        //    tmp2->push_back(e);
        //    tmp2->push_back(f);
        //    tmp2->push_back(g);
        //    tmp2->push_back(h);

        //    mat = Affine3f::Identity();
        //    mat.rotate(rot);
        //    mat.translation() << pos.x, pos.y, pos.z;
        //    transformPointCloud(*tmp2, *tmp2, mat);

        //    addCloudView(tmp1, "tmp1", 0, 255, 0, 5);
        //    addCloudView(tmp2, "tmp2", 0, 0, 255, 5);
        //    wait();

        //    centerClouds(*tmp1, *tmp2);

        //    auto tmpest = boost::make_shared<registration::TransformationEstimationSVDScale<PointXYZ, PointXYZ>>();
        //    IterativeClosestPoint<PointXYZ, PointXYZ> tmpicp;
        //    tmpicp.setTransformationEstimation(tmpest);
        //    tmpicp.setInputSource(tmp1);
        //    tmpicp.setInputTarget(tmp2);
        //    tmpicp.setMaximumIterations(30);
        //    tmpicp.align(*tmp1);
        //    auto m = tmpicp.getFinalTransformation();

        //    transformPointCloud(*resultCloud, *resultCloud, m);
        //    updateCloudView(resultCloud);
        //    wait();
        //}


        double fitness = numeric_limits<double>::max();
        auto te =  boost::make_shared<registration::TransformationEstimationSVDScale<PointXYZ, PointXYZ>>();

        IterativeClosestPoint<PointXYZ, PointXYZ> icp;
        icp.setTransformationEstimation(te);
        icp.setInputSource(resultCloud);
        icp.setInputTarget(_referenceCloud);
        icp.setMaximumIterations(ICP_ITER_STEP);

        GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> icpg;
        icpg.setInputSource(resultCloud);
        icpg.setInputTarget(_referenceConvexCloud);
        icpg.setMaximumIterations(ICP_ITER_STEP);

        int iterCnt = 0;
        auto start = chrono::high_resolution_clock::now();
        auto future = async(launch::async, [&icp, &icpg, &iterCnt, &resultCloud, &fitness]()
        {
            centerClouds(*resultCloud, *_referenceCloud);

            for (iterCnt = 0; iterCnt < 200; iterCnt += ICP_ITER_STEP)
            {
                icpg.align(*resultCloud);
                auto f = icpg.getFitnessScore();

                //if (fitness - f < ICP_EPSILON_OBJECT)
                //    break;

                updateCloudView(resultCloud);
                fitness = f;
            }

            icpg.setInputTarget(_referenceCloud);

            for (iterCnt = 0; iterCnt < ICP_MAX_ITER; iterCnt += ICP_ITER_STEP)
            {
                icpg.align(*resultCloud);
                auto f = icpg.getFitnessScore();

                //if (fitness - f < ICP_EPSILON_OBJECT)
                //    break;

                updateCloudView(resultCloud);
                fitness = f;
            }

            for (iterCnt = 0; iterCnt < ICP_MAX_ITER; iterCnt += ICP_ITER_STEP)
            {
                icp.align(*resultCloud);
                auto f = icp.getFitnessScore();

                if (fitness - f < ICP_EPSILON_OBJECT)
                    break;

                updateCloudView(resultCloud);
                fitness = f;
            }
        });

        wait(future);
        auto stop = chrono::high_resolution_clock::now();
        vout << chrono::duration_cast<chrono::milliseconds>(stop - start).count() / 1000.0f << "s, " << iterCnt << " iterations, fitness: " << fitness << endl;
    }

    {
        search::KdTree<PointXYZ> kdTree;
        kdTree.setInputCloud(_referenceCloud);
        vector<float> distances(resultCloud->size());

        for (int i = 0; i < resultCloud->size(); i++)
        {
            vector<float> dist(1);
            vector<int> idx(1);
            kdTree.nearestKSearch(resultCloud->at(i), 1, idx, dist);
            distances[i] = sqrt(dist[0]);
        }

        auto resultCloudWithDistance = boost::make_shared<PointCloud<PointXYZRGB>>();
        double maxDist = 0;

        for (auto d : distances)
            if (d > maxDist)
                maxDist = d;

        for (int i = 0; i < resultCloud->size(); i++)
        {
            PointXYZRGB p;
            auto r = resultCloud->at(i);
            p.x = r.x;
            p.y = r.y;
            p.z = r.z;
            p.r = min(distances[i] / maxDist * 255, 255.0);
            p.b = 255 - min(distances[i] / maxDist * 255, 255.0);
            resultCloudWithDistance->push_back(p);
        }

        removeCloudView(resultCloud);
        viewer->addPointCloud(resultCloudWithDistance, "result_distance");
        viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "result_distance");
        viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_OPACITY, 0.2, "reference_cloud");

        float  min = 0, max = 0, median = 0, avg = 0;
        for (auto d : distances)
        {
            min = std::min(min, d);
            max = std::max(max, d);
            avg += d;
        }
        avg /= distances.size();
        sort(distances.begin(), distances.end());
        median = distances[distances.size() / 2];
        vout << "Result: " << endl;
        vout << "min: " << min * 1000 << "mm" << endl;
        vout << "max: " << max * 1000 << "mm" << endl;
        vout << "avg: " << avg * 1000 << "mm" << endl;
        vout << "median: " << median * 1000 << "mm" << endl;
    }

    if (!viewer->wasStopped())
        viewer->spin();
}
