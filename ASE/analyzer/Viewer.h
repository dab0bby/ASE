#pragma once

#include <mutex>
#include <future>
#include <map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "CloudView.h"
#include "AnalyzerConfig.h"
#include <unordered_set>

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
        void registerKeyboardCallback(boost::function<void(const pcl::visualization::KeyboardEvent&)> callback) const;
        void start();
        void stop();
        bool isOpen() const;
        void lock();
        void updateText(const std::string& text, float r, float g, float b, float fontSize = 12);
        void unlock();
        pcl::visualization::PCLVisualizer::Ptr getViewer() const;


    private:
        enum UpdateType
        {
            AddOrUpdate,
            Remove,
        };

        std::mutex _mutex, _viewerMutex;
        std::map<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, std::string> _cloudMap;
        std::map<std::string, CloudView> _viewMap;
        std::map<std::string, UpdateType> _pendingUpdates;        
        pcl::visualization::PCLVisualizer::Ptr _viewer;
        std::future<void> _spinThread;
        int _colorIndex = 0;
        AnalyzerConfig _config;
        std::string _name;
        std::string _currentText;
        std::array<float, 3> _textColor = { 1.0, 1.0, 1.0 };
        float _textSize = 12;
        std::function<void(const std::string&)> _updateLambda;
        bool _shutdown = false;

        void _spin();
        void _voutUpdate(const std::string& text);
    };

}
