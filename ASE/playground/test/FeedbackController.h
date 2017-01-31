#pragma once
#include "AnalyzerConfig.h"
#include <pcl/visualization/pcl_visualizer.h>

namespace ASE
{
    class FeedbackController
    {
    public:
        FeedbackController(bool hasToWait);

        void wait();
        void setViewer(pcl::visualization::PCLVisualizer::Ptr viewer);
        void removeViewer();

    private:
        bool _hasToWait = false;
        bool _waiting = false;
        pcl::visualization::PCLVisualizer::Ptr _viewer;

        void _keyboardInput(const pcl::visualization::KeyboardEvent& e);
    };

}