#pragma once

#include "Viewer.h"

#include <pcl/visualization/pcl_visualizer.h>

namespace ASE
{
    class FeedbackController
    {
    public:
        FeedbackController(bool hasToWait);

        void wait(std::string msg = "press a key to continue...");
        void setViewer(std::shared_ptr<Viewer> viewer);
        void removeViewer();
        void _keyboardCallback(const pcl::visualization::KeyboardEvent& ke);

    private:
        bool _hasToWait = false;
        bool _waiting = false;
        std::shared_ptr<Viewer> _viewer;
    };
}
