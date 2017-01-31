#include "Viewer.h"
#include "Globals.h"
#include "vtkRenderWindow.h"
#include "vtkSmartPointer.h"
#include "vtkRenderWindowInteractor.h"

using namespace std;
using namespace pcl;

namespace ASE 
{
    Viewer::Viewer(const AnalyzerConfig& config, const std::string name) :
        _config(config), _name(name), _updateLambda([=](const std::string& s) { _voutUpdate(s); }),
        _textColor{ config.visualizer.textColor[0], config.visualizer.textColor[1], config.visualizer.textColor[2] }
    {
        voutBuffer->subscribe(_updateLambda);
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
        if (_viewMap.find(name) != _viewMap.end())
        {
            _mutex.unlock();
            return false;
        }

        auto c = _config.visualizer.colors[_colorIndex];
        CloudView view;
        view.name = name;
        view.cloud = cloud;
        view.colorHandler = boost::make_shared<visualization::PointCloudColorHandlerCustom<PointXYZ>>(cloud, c[0], c[1], c[2]);
        view.pointSize = pointSize;
        view.opacity = opacity;

        _viewMap[name] = view;
        _cloudMap[cloud] = name;
        _pendingUpdates[name] = AddOrUpdate;
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

        _pendingUpdates[name] = AddOrUpdate;
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

        auto cv = _viewMap[name];
        _cloudMap.erase(cv.cloud);
        _viewMap.erase(cv.name);
        _pendingUpdates[cv.name] = Remove;

        _mutex.unlock();
    }

    void Viewer::registerKeyboardCallback(boost::function<void(const pcl::visualization::KeyboardEvent&)> callback) const
    {
        _viewer->registerKeyboardCallback(callback);
    }
    
    void Viewer::start()
    {
        if (_viewer && !_viewer->wasStopped())
            stop();

        //voutBuffer->subscribe(_updateLambda);

        _spinThread = async(launch::async, [=]
        {
            _viewerMutex.lock();
            _viewer = boost::make_shared<visualization::PCLVisualizer>(_name);
            _viewer->setShowFPS(false);
            _viewer->setBackgroundColor(_config.visualizer.backgroundColor[0], _config.visualizer.backgroundColor[1], _config.visualizer.backgroundColor[2]);
            _viewer->initCameraParameters();
            _viewerMutex.unlock();

            _spin();

            _shutdown = false;
            if (!_viewer->wasStopped())
                _viewer->close();
        });

        // let viewer some time to init
        this_thread::sleep_for(10ms);
        _viewerMutex.lock();
        _viewerMutex.unlock();
    }

    void Viewer::stop()
    {   
        _viewerMutex.lock();
        _shutdown = true;
        _viewerMutex.unlock();

        if (_spinThread.valid())
            _spinThread.wait();
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
        while (!_viewer->wasStopped())
        {
            if (_mutex.try_lock())
            {
                if (!_pendingUpdates.empty())
                {
                    for (auto c : _pendingUpdates)
                    {
                        switch (c.second)
                        {
                        case AddOrUpdate:
                        {
                            auto v = _viewMap[c.first];
                            if (_viewer->contains(v.name))
                                _viewer->updatePointCloud(v.cloud, *v.colorHandler, v.name);
                            else
                                _viewer->addPointCloud(v.cloud, *v.colorHandler, v.name);

                            _viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, v.pointSize, v.name);
                            _viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, v.opacity, v.name);
                            break;
                        }

                        case Remove:
                            _viewer->removePointCloud(c.first);
                            break;
                        }
                    }

                    _pendingUpdates.clear();
                }

                if (!_viewer->updateText(_currentText, 8, 8, _config.visualizer.fontSize, _textColor[0], _textColor[1], _textColor[2], "description"))
                    _viewer->addText(_currentText, 8, 8, _config.visualizer.fontSize, _textColor[0], _textColor[1], _textColor[2], "description");
                
                _mutex.unlock();
            }

            _viewerMutex.lock();
            if (_shutdown)  // needs the lock
                break;
            _viewer->spinOnce(20);
            _viewerMutex.unlock();

            this_thread::sleep_for(20ms);
        }
    }

    void Viewer::_voutUpdate(const std::string& text)
    {
        if (!isOpen())
            return;

        _mutex.lock();
        _currentText = text;
        _mutex.unlock();
    }
}
