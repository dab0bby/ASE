#pragma once

#include "SmartStreamBuffer.h"
#include <memory>

#ifdef ASE_ANALYZER_EXPORTS
    #define ASE_ANALYZER_API __declspec(dllexport)
#else
    #define ASE_ANALYZER_API __declspec(dllimport)
#endif

namespace ASE 
{
    extern std::ostream vout;
    extern std::shared_ptr<SmartStreamBuffer> voutBuffer;
}
