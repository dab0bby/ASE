#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include "BoundingBox.h"
#include <pcl/common/transforms.h>
#include <thread>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "utils.h"


namespace ASE
{
    using namespace std;
    using namespace pcl;
    using namespace Eigen;

    ////////////////////////////////
    //   forward declarations
    ////////////////////////////////

    PointCloud<PointXYZ>::Ptr applyNoiseFilter(PointCloud<PointXYZ>::Ptr& cloud, int kNearestNeighbor = 100, double stdDevMul = 0.5);

    PointCloud<PointXYZ>::Ptr applyClusterFilter(const PointCloud<PointXYZ>::ConstPtr& cloud, double clusterTolerance = 0.02, int minClusterSize = 1, int maxClusterSize = 100000);

    PointCloud<PointXYZ>::Ptr subtract(PointCloud<PointXYZ>::Ptr& left, PointCloud<PointXYZ>::Ptr& right, float precision = 0.1);

    template<typename IcpType>
    void runIcp(IcpType icp, PointCloud<PointXYZ>::Ptr source, PointCloud<PointXYZ>::Ptr target, double fitnessEpsilon = 1e-9, int maxIterations = 200,
        int iterationsPerUpdate = 1, function<bool(int, double, Matrix4f&)> updateCallback = [](int, double, Matrix4f&) {return true; });

    void runIcp(PointCloud<PointXYZ>::Ptr source, PointCloud<PointXYZ>::Ptr target, double fitnessEpsilon = 1e-9, int maxIterations = 200,
        int iterationsPerUpdate = 1, function<bool(int, double, Matrix4f&)> updateCallback = [](int, double, Matrix4f&) {return true; });

    template<typename SourceT, typename TargetT>
    Affine3f centerClouds(PointCloud<SourceT>& source, PointCloud<TargetT>& target);

    template<typename ...FuncTypes>
    void runParallel(FuncTypes... funcs);

    template<typename PointIn>
    BoundingBox computeOBB(typename PointCloud<PointIn>::Ptr& cloud);
    
    template<typename PointIn>
    BoundingBox computeAABB(typename PointCloud<PointIn>::Ptr& cloud);

    template<typename PointA, typename PointB>
    PointXYZ size(PointA a, PointB b);

    ////////////////////////////////
    // end forward declarations
    ////////////////////////////////

    PointCloud<PointXYZ>::Ptr applyNoiseFilter(const PointCloud<PointXYZ>::ConstPtr& cloud, int kNearestNeighbor, double stdDevMul)
    {
        PointCloud<PointXYZ>::Ptr result(new PointCloud<PointXYZ>);

        StatisticalOutlierRemoval<PointXYZ> noiseFilter;
        noiseFilter.setInputCloud(cloud);
        noiseFilter.setMeanK(kNearestNeighbor);
        noiseFilter.setStddevMulThresh(stdDevMul);
        noiseFilter.filter(*result);

        return result;
    }

    PointCloud<PointXYZ>::Ptr applyClusterFilter(const PointCloud<PointXYZ>::ConstPtr& cloud, double clusterTolerance, int minClusterSize, int maxClusterSize)
    {
        PointCloud<PointXYZ>::Ptr result(new PointCloud<PointXYZ>);

        vector<PointIndices> clusterPoints;
        EuclideanClusterExtraction<PointXYZ> ec;
        ec.setClusterTolerance(clusterTolerance);
        ec.setMinClusterSize(minClusterSize);
        ec.setMaxClusterSize(maxClusterSize);
        ec.setInputCloud(cloud);
        ec.extract(clusterPoints);

        PointIndices pts;

        for (auto c : clusterPoints)
        {
            if (pts.indices.size() < c.indices.size())
                pts = c;
        }

        result->reserve(pts.indices.size());

        for (auto i : pts.indices)
            result->push_back(cloud->at(i));

        return result;
    }

    PointCloud<PointXYZ>::Ptr subtract(PointCloud<PointXYZ>::Ptr& left, PointCloud<PointXYZ>::Ptr& right, float precision)
    {
        PointCloud<PointXYZ>::Ptr result(new PointCloud<PointXYZ>);
        KdTreeFLANN<PointXYZ> rightTree;
        rightTree.setInputCloud(right);

        for (auto p : *left)
        {
            vector<int> index(1);
            vector<float> sqrDistance(1);

            // if no point of 'right' is near enough 'p' -> add it to the result
            if (rightTree.radiusSearch(p, precision, index, sqrDistance, 1) == 0)
                result->push_back(p);
        }

        return result;
    }

    template<typename IcpType>
    void runIcp(IcpType icp, PointCloud<PointXYZ>::Ptr source, PointCloud<PointXYZ>::Ptr target, double fitnessEpsilon, int maxIterations,
        int iterationsPerUpdate, function<bool(int, double, Matrix4f&)> updateCallback)
    {
        icp.setInputSource(source);
        icp.setInputTarget(target);
        icp.setMaximumIterations(iterationsPerUpdate);

        double fitness = numeric_limits<double>::max();

        for (int iterCnt = 0; iterCnt < maxIterations; iterCnt += iterationsPerUpdate)
        {
            icp.align(*source);
            auto f = icp.getFitnessScore();

            if (fitness - f < fitnessEpsilon)
                break;

            auto transform = icp.getFinalTransformation();
            if (!updateCallback(iterCnt, f, transform))
                break;
            fitness = f;
        }
    }

    void runIcp(PointCloud<PointXYZ>::Ptr source, PointCloud<PointXYZ>::Ptr target, double fitnessEpsilon, int maxIterations,
        int iterationsPerUpdate, function<bool(int, double, Matrix4f&)> updateCallback) 
    {
        IterativeClosestPoint<PointXYZ, PointXYZ> icp;
        runIcp(icp, source, target, fitnessEpsilon, maxIterations, iterationsPerUpdate, updateCallback);
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

    template<typename ...FuncTypes>
    void runParallel(FuncTypes... funcs)
    {
        vector<thread> threads;
        threads.reserve(sizeof...(FuncTypes));        
        for_each_argument([&threads](thread& t) {threads.push_back(t); }, thread(funcs)...);

        for (auto t : threads)
            t.join();
    }

    template<typename PointIn>
    BoundingBox computeOBB(typename PointCloud<PointIn>::Ptr& cloud)
    {
        PointXYZ a, b, c, d, e, f, g, h, pos, s;
        Matrix3f rot;
        MomentOfInertiaEstimation<PointIn> moments;
        moments.setInputCloud(cloud);
        moments.compute();
        moments.getOBB(a, g, pos, rot);
        s = size(g, a);
        b = { a.x + s.x, a.y, a.z };
        c = { a.x + s.x, a.y, a.z + s.z };
        d = { a.x, a.y, a.z + s.z };
        e = { g.x - s.x, g.y, g.z - s.z };
        f = { g.x, g.y, g.z - s.z };
        h = { g.x, g.y, g.z };

        PointCloud<PointXYZ>::Ptr tmp(new PointCloud<PointXYZ>);
        tmp->push_back(a);
        tmp->push_back(b);
        tmp->push_back(c);
        tmp->push_back(d);
        tmp->push_back(e);
        tmp->push_back(f);
        tmp->push_back(g);
        tmp->push_back(h);
        
        Affine3f mat = Affine3f::Identity();
        mat.rotate(rot);
        mat.translation() << pos.x, pos.y, pos.z;
        
        transformPointCloud(*tmp, *tmp, mat);

        return BoundingBox{ s.x, s.y, s.z, tmp };
    }

    template<typename PointIn>
    BoundingBox computeAABB(typename PointCloud<PointIn>::Ptr& cloud)
    {
        PointXYZ a, b, c, d, e, f, g, h, s;

        MomentOfInertiaEstimation<PointIn> moments;
        moments.setInputCloud(cloud);
        moments.compute();
        moments.getAABB(a, g);       
        
        s = size(g, a);
        b = { a.x + s.x, a.y, a.z };
        c = { a.x + s.x, a.y, a.z + s.z };
        d = { a.x, a.y, a.z + s.z };
        e = { g.x - s.x, g.y, g.z - s.z };
        f = { g.x, g.y, g.z - s.z };
        h = { g.x, g.y, g.z };

        PointCloud<PointXYZ>::Ptr tmp(new PointCloud<PointXYZ>);
        tmp->push_back(a);
        tmp->push_back(b);
        tmp->push_back(c);
        tmp->push_back(d);
        tmp->push_back(e);
        tmp->push_back(f);
        tmp->push_back(g);
        tmp->push_back(h);

        return BoundingBox{ s.x, s.y, s.z, tmp };
    }
    
    template<typename PointA, typename PointB>
    PointXYZ size(PointA a, PointB b)
    {
        return{ abs(a.x - b.x), abs(a.y - b.y), abs(a.z - b.z) };
    }
}
