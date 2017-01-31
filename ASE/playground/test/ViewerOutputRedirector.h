#pragma once

#include <sstream>
#include "utils.h"
#include "Viewer.h"

namespace ASE
{
    class ViewerOutputRedirector
    {

    public:
        ViewerOutputRedirector(boost::shared_ptr<Viewer> viewer = nullptr, int lineCount = 1) :
            _ss(), _output(), _lineCount(lineCount), _viewer(viewer)
        {
        }

        template <typename T>
        ViewerOutputRedirector& operator<<(T&& obj)
        {
            std::cout << obj;

            _ss.str("");
            _ss.clear();

            _ss << obj;

            auto tmp = removeRepetitions(_ss.str(), '\n');

            if (tmp.empty())
                return *this;

            if (_endsWithLF && tmp.size() != 1 && tmp[0] != '\n')
                tmp = '\n' + trimStart(tmp, '\n');

            _endsWithLF = tmp[tmp.size() - 1] == '\n';
            _output += trimEnd(tmp, '\n');

            int cnt = 0;
            for (int i = _output.size() - 1; i >= 0; i--)
            {
                if (_output[i] != '\n')
                    continue;

                cnt++;

                if (cnt == _lineCount)
                {
                    _output = _output.substr(i + 1);
                    break;
                }
            }

            _lastSize = _output.size();
            print();
            return *this;
        }

        void setViewer(boost::shared_ptr<Viewer> viewer)
        {
            _viewer = viewer;
        }

        boost::shared_ptr<Viewer> getViewer() const
        {
            return _viewer;
        }


    private:
        const std::string _id = "description";
        std::ostringstream _ss;
        std::string _output;
        int index = 0;
        bool _endsWithLF = false;
        int _lineCount;
        int _lastSize = 0;
        boost::shared_ptr<Viewer> _viewer;

        void print() const
        {
            if (_viewer && _viewer->isOpen())            
                _viewer->updateText(_output, 0, 0.5, 0.7, 12);            
        }
    };

}
