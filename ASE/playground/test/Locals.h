#pragma once


#include "OStreamCombiner.h"
#include "ViewerOutputRedirector.h"

namespace ASE 
{
    ViewerOutputRedirector _vout(nullptr, 6);

    #define endl (std::endl<char, char_traits<char>>)
    #define vout ASE::OStreamCombiner<ASE::ViewerOutputRedirector>(_vout)
}