#pragma once

#include <sstream>
#include <functional>
#include <vector>

namespace ASE
{
    class SmartStreamBuffer : public std::stringbuf
    {
    public:
        typedef void (SubscriberPtr)(const std::string&);
        typedef std::function<void(const std::string&)> Subscriber;

        explicit SmartStreamBuffer(int lineCount = 1) : _lineCount(lineCount)
        {
        };

        void subscribe(Subscriber sub);
        //void unsubscribe(SubscriberPtr sub);

        int_type overflow(int_type c) override;
        std::streamsize xsputn(const char* s, std::streamsize n) override;

    private:
        std::vector<Subscriber> _subscriber;
        std::string _output;
        bool _endsWithLF = false;
        int _lineCount;

        void _update(const std::string& s);
        void _notify();
    };
}
