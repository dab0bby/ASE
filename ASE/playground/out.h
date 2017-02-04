#pragma once

#include "utils.h"

#include <sstream>
#include <ostream>
#include <pcl/visualization/pcl_visualizer.h>


class out
{

public:
    out(pcl::visualization::PCLVisualizer::Ptr viewer, int lineCount = 1, bool updateViewer = false) :
        _ss(), _output(), _lineCount(lineCount), _updateViewer(updateViewer), _viewer(viewer)
    {
    }

    template <typename T>
    out& operator<<(T&& obj)
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

private:
    const std::string _id = "description";
    std::ostringstream _ss;
    std::string _output;
    int index = 0;
    bool _endsWithLF = false;
    int _lineCount;
    int _lastSize = 0;
    bool _updateViewer;
    pcl::visualization::PCLVisualizer::Ptr _viewer;

    void print() const
    {
        if (!_viewer->wasStopped())
        {
            if (!_viewer->updateText(_output, 8, 5, 14,  0, 0.5, 0.7, _id))
                _viewer->addText(_output, 8, 5, 14, 0, 0.5, 0.7, _id);

            if (_updateViewer)
                _viewer->spinOnce();
        }
    }
};


template<typename TStream>
class tmpout
{
public:
    tmpout(TStream& out) : _out(out), _ss()
    {
    }

    template <class T>
    tmpout& operator<<(T&& v)
    {
        _ss << v;
        return *this;
    }

    ~tmpout()
    {
        _out << _ss.str();
    }

private:
    TStream& _out;
    std::ostringstream _ss;
};
