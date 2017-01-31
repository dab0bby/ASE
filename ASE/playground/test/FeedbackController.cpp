
#include "FeedbackController.h"

#include <pcl/visualization/keyboard_event.h>
#include <thread>
#include <chrono>
#include "Locals.h"
#include <boost/signals2.hpp>
#include <boost/function.hpp>

using namespace std;

namespace ASE 
{
    FeedbackController::FeedbackController(bool hasToWait) : _hasToWait(hasToWait)
    {
    }

    void FeedbackController::wait()
    {
        if (!_hasToWait)
            return;
        
        vout << "press a key to continue...";
        _waiting = true;
        
        while (_waiting)        
            this_thread::sleep_for(20ms);        
    }

    void FeedbackController::removeViewer()
    {
        _hasToWait = false;
    }

    void FeedbackController::_keyboardInput(const pcl::visualization::KeyboardEvent& ke)
    {
        if (ke.isAltPressed() || ke.isCtrlPressed() || ke.isShiftPressed())
            return;

        _waiting = false;
    }

    void FeedbackController::setViewer(pcl::visualization::PCLVisualizer::Ptr viewer )
    {
        if (viewer)
        {
            _viewer = viewer;
            _viewer->registerKeyboardCallback([this](const pcl::visualization::KeyboardEvent& e) {_keyboardInput(e); });
            _hasToWait = true;
        }
    }
}