#include "StopWatch.h"

using namespace std;
using namespace chrono;

namespace ASE
{
    StopWatch::StopWatch() : _start(), _lap(0s)
    {
    }

    void StopWatch::start()
    {
        if (_isRunning)
            return;

        _start = high_resolution_clock::now();
        _isRunning = true;
    }

    void StopWatch::stop()
    {
        if (!_isRunning)
            return;

        _lap += high_resolution_clock::now() - _start;
        _isRunning = false;
    }

    nanoseconds StopWatch::getTime() const
    {
        return _isRunning
                   ? _lap + (high_resolution_clock::now() - _start)
                   : _lap;
    }

    void StopWatch::reset()
    {
        stop();
        _lap = 0s;
    }

    bool StopWatch::isRunning() const
    {
        return _isRunning;
    }
}
