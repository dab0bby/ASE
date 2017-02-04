#include "FeedbackController.h"

#include "Globals.h"

#include <pcl/visualization/keyboard_event.h>
#include <thread>
#include <chrono>

using namespace std;

namespace ASE
{
    FeedbackController::FeedbackController(bool hasToWait) : _hasToWait(hasToWait)
    {
    }

    void FeedbackController::wait(string msg)
    {
        if (!_hasToWait || !_viewer->isOpen())
            return;

        vout << msg << endl;
        _waiting = true;

        while (_waiting && _viewer->isOpen())
            this_thread::sleep_for(20ms);

        _hasToWait &= _viewer->isOpen();
    }

    void FeedbackController::setViewer(std::shared_ptr<Viewer> viewer)
    {
        if (viewer)
        {
            _viewer = viewer;
            _viewer->registerKeyboardCallback(boost::bind(&FeedbackController::_keyboardCallback, this, _1));
        }
    }

    void FeedbackController::removeViewer()
    {
        _waiting = false;
    }

    void FeedbackController::_keyboardCallback(const pcl::visualization::KeyboardEvent& ke)
    {
        if (ke.keyUp() || ke.isAltPressed() || ke.isCtrlPressed() || ke.isShiftPressed())
            return;

        _waiting = false;
    }
}
