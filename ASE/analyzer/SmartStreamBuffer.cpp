#include "SmartStreamBuffer.h"
#include "utils.h"

#include <iostream>

using namespace std;

namespace ASE
{
    void SmartStreamBuffer::subscribe(Subscriber sub)
    {
        _subscriber.push_back(sub);
    }

    //void SmartStreamBuffer::unsubscribe(SubscriberPtr sub)
    //{
    //    auto i = find(_subscriber.begin(), _subscriber.end(), sub);
    //    if (i != _subscriber.end())
    //        _subscriber.erase(i);
    //}

    std::basic_stringbuf<char>::int_type SmartStreamBuffer::overflow(int_type c)
    {
        cout.put(c);
        string s;
        s += static_cast<char>(c);
        _update(s);

        return c;
    }

    std::streamsize SmartStreamBuffer::xsputn(const char* s, std::streamsize n)
    {
        cout << s;
        _update(s);

        return n;
    }

    void SmartStreamBuffer::_update(const string& s)
    {
        auto tmp = removeRepetitions(s, '\n');

        if (tmp.empty())
            return;

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

        _notify();
    }

    void SmartStreamBuffer::_notify()
    {
        for (auto sub : _subscriber)
            sub(_output);
    }
}
