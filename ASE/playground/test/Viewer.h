#pragma once

#include <mutex>
#include <future>
#include <map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "CloudView.h"
#include "AnalyzerConfig.h"

namespace ASE
{
    class Viewer
    {
    public:
        explicit Viewer(const AnalyzerConfig& config, const std::string name = "3D Viewer");
        ~Viewer();

        bool addCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const std::string& name, int pointSize = 1, float opacity = 1);
        void updateCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
        void updateCloud(std::string& name);
        void removeCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
        void removeCloud(std::string& name);
        void start();
        void stop() const;
        bool isOpen() const;
        void lock();
        void updateText(const std::string& text, float r, float g, float b, float fontSize = 12);
        void unlock();
        pcl::visualization::PCLVisualizer::Ptr getViewer() const;

    private:
        std::mutex _mutex, _viewerMutex;
        std::map<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, std::string> _cloudMap;
        std::map<std::string, CloudView> _viewMap;
        std::set<std::string> _pendingUpdates;
        std::set<std::string> _pendingErases;
        pcl::visualization::PCLVisualizer::Ptr _viewer;
        std::future<void> _future;
        int _colorIndex = 0;
        AnalyzerConfig _config;
        std::string _name;
        std::string _currentText;
        std::vector<float> _textColor = { 1.0, 1.0, 1.0 };
        float _textSize = 12;

        void _spin();
    };

}