#include "Viewer.h"

using namespace std;
using namespace pcl;

namespace ASE 
{
    Viewer::Viewer(const AnalyzerConfig& config, const std::string name) : _config(config), _name(name)
    {
    }


    Viewer::~Viewer()
    {
        stop();
    }

    bool Viewer::addCloud(const PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const std::string& name, int pointSize, float opacity)
    {
        if (!isOpen())
            return false;

        _mutex.lock();
        if (_viewMap.find(name) == _viewMap.end())
        {
            _mutex.unlock();
            return false;
        }

        auto c = _config.visualizer.colors[_colorIndex];
        CloudView view;
        view.name = name;
        view.cloud = cloud;
        view.colorHandler = boost::make_shared<visualization::PointCloudColorHandlerCustom<PointXYZ>>(c[0], c[1], c[2]);
        view.pointSize = pointSize;
        view.opacity = opacity;

        _viewMap[name] = view;
        _cloudMap[cloud] = name;
        _pendingUpdates.insert(name);
        _mutex.unlock();

        _colorIndex = (_colorIndex + 1) % _config.visualizer.colors.size();
        return true;
    }

    void Viewer::updateCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
    {
        if (!isOpen() || _cloudMap.find(cloud) == _cloudMap.end())
            return;

        updateCloud(_cloudMap[cloud]);
    }

    void Viewer::updateCloud(std::string& name)
    {
        if (!isOpen())
            return;

        _mutex.lock();
        if (_viewMap.find(name) == _viewMap.end())
        {
            _mutex.unlock();
            return;
        }

        _pendingUpdates.insert(name);
        _mutex.unlock();
    }

    void Viewer::removeCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
    {
        if (!isOpen())
            return;

        if (_cloudMap.find(cloud) == _cloudMap.end())
            return;

        removeCloud(_cloudMap[cloud]);

    }

    void Viewer::removeCloud(std::string& name)
    {
        if (!isOpen())
            return;

        _mutex.lock();
        if (_viewMap.find(name) == _viewMap.end())
        {
            _mutex.unlock();
            return;
        }

        _pendingUpdates.erase(name);
        auto cv = _viewMap[name];
        _cloudMap.erase(cv.cloud);
        _viewMap.erase(name);
        _pendingErases.insert(name);

        _mutex.unlock();
    }


    void Viewer::start()
    {
        if (_viewer && !_viewer->wasStopped())
            stop();

        _viewer = boost::make_shared<visualization::PCLVisualizer>(_name);
        _viewer->setShowFPS(false);
        _viewer->initCameraParameters();
        _viewer->setBackgroundColor(1.0 / _config.visualizer.backgroundColor[0], 1.0 / _config.visualizer.backgroundColor[1], 1.0 / _config.visualizer.backgroundColor[2]);

        _future = async(launch::async, [this] {this->_spin(); });
    }

    void Viewer::stop() const
    {
        _viewer->close();
        _future.wait();
    }

    bool Viewer::isOpen() const
    {
        return _viewer && !_viewer->wasStopped();
    }

    void Viewer::lock()
    {
        _viewerMutex.lock();
    }

    void Viewer::updateText(const std::string& text, float r, float g, float b, float fontSize)
    {
        _mutex.lock();
        _currentText = text;
        _textColor = { r, g, b };
        _textSize = fontSize;
        _mutex.unlock();
    }

    void Viewer::unlock()
    {
        _viewerMutex.unlock();
    }

    pcl::visualization::PCLVisualizer::Ptr Viewer::getViewer() const
    {
        return _viewer;
    }


    void Viewer::_spin()
    {
        if (!_viewer->wasStopped())
        {
            if (_mutex.try_lock())
            {
                if (!_pendingUpdates.empty())
                {
                    for (auto c : _pendingUpdates)
                    {
                        auto v = _viewMap[c];

                        if (_viewer->contains(v.name))
                            _viewer->updatePointCloud(v.cloud, *v.colorHandler, v.name);
                        else
                            _viewer->addPointCloud(v.cloud, *v.colorHandler, v.name);

                        _viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, v.pointSize);
                        _viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, v.opacity);
                    }

                    _pendingUpdates.clear();
                }

                if (!_pendingErases.empty())
                {
                    for (auto c : _pendingErases)
                        _viewer->removePointCloud(c);

                    _pendingErases.clear();
                }

                if (_viewer->updateText(_currentText, 8, 8, _textColor[0], _textColor[1], _textColor[2], "description"))
                    _mutex.unlock();
            }

            _viewerMutex.lock();
            _viewer->spinOnce(20);
            _viewerMutex.unlock();

            this_thread::sleep_for(20ms);
        }
    }
}