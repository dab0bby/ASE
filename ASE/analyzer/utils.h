#pragma once

#include <string>
#include <sstream>
#include <vector>
#include "Globals.h"

namespace ASE
{
    ASE_ANALYZER_API
    inline void split(const std::string& s, char delim, std::vector<std::string>& elems)
    {
        std::stringstream ss;
        ss.str(s);
        std::string item;
        while (std::getline(ss, item, delim))
            elems.push_back(item);
    }
     
    ASE_ANALYZER_API
    inline std::vector<std::string> split(const std::string& s, char delim)
    {
        std::vector<std::string> elems;
        split(s, delim, elems);
        return elems;
    }

    ASE_ANALYZER_API
    inline std::string trimStart(const std::string& s, char delim)
    {
        if (s.empty())
            return s;

        int cnt = 0;
        while (cnt < s.size() && s[cnt] == delim)
            cnt++;

        return cnt == 0 ? s : cnt > s.size() ? "" : s.substr(cnt);
    }

    ASE_ANALYZER_API
    inline std::string trimEnd(const std::string& s, char delim)
    {
        if (s.empty())
            return s;

        int cnt = 1;
        while (cnt <= s.size() && s[s.size() - cnt] == delim)
            cnt++;

        cnt--;
        return cnt == 0 ? s : cnt > s.size() ? "" : s.substr(0, s.size() - cnt);
    }

    ASE_ANALYZER_API
    inline std::string trim(const std::string& s, char delim)
    {
        return trimEnd(trimStart(s, delim), delim);
    }

    ASE_ANALYZER_API
    inline std::string removeRepetitions(const std::string s, char delim)
    {
        if (s == "")
            return s;

        std::string res;
        res.reserve(s.size() + 1);
        int cnt = 0;

        for (char c : s)
        {
            cnt = c == delim ? cnt + 1 : 0;

            if (cnt > 1)
                continue;

            res += c;
        }

        return res;
    }

    template <typename TF, typename... Ts>
    ASE_ANALYZER_API
    void for_each_argument(TF&& f, Ts&&... xs)
    {
        std::initializer_list<int>{ (f(std::forward<Ts>(xs)) , 0)... };
    }

}
