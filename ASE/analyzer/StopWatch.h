#pragma once

#include <chrono>
#include "Globals.h"

namespace ASE 
{
    class StopWatch
    {
    public:
        ASE_ANALYZER_API StopWatch();

        ASE_ANALYZER_API void start();
        ASE_ANALYZER_API  void stop(); 
        ASE_ANALYZER_API std::chrono::nanoseconds getTime() const;
        ASE_ANALYZER_API void reset();
        ASE_ANALYZER_API bool isRunning() const;

    private:
        std::chrono::high_resolution_clock::time_point _start;
        std::chrono::high_resolution_clock::duration _lap;
        bool _isRunning = false;

    };
}