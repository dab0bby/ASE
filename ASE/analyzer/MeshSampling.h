#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <random>

namespace ASE 
{
    class MeshSampling
    {
    public:
        MeshSampling();
        pcl::PointCloud<pcl::PointXYZ>::Ptr sample(pcl::PolygonMesh& mesh, int pointCount = 100000, float leafSize = 0.1);

    private:
        std::random_device _rd;
        std::mt19937 _mt;
        std::uniform_real_distribution<double> _dist;

        // static double _uniformDeviate(int seed);
        void _randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector4f& p);
        void _randPSurface(vtkPolyData* polydata, std::vector<double>* cumulativeAreas, double totalArea, Eigen::Vector4f& p);
        void _uniformSampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, pcl::PointCloud<pcl::PointXYZ>& cloud_out);
    };
}