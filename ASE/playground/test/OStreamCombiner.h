#pragma once

#include <sstream>

namespace ASE
{
    template<typename TStream>
    class OStreamCombiner
    {
    public:
        OStreamCombiner(TStream& out) : _out(out), _ss()
        {
        }

        template <class T>
        OStreamCombiner& operator<<(T&& v)
        {
            _ss << v;
            return *this;
        }

        ~OStreamCombiner()
        {
            _out << _ss.str();
        }

    private:
        TStream& _out;
        std::ostringstream _ss;
    };
}