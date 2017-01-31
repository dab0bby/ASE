#pragma once

template <typename T>
class ReleaseDeleter
{
public:
    ReleaseDeleter() : _released(false)
    {        
    }

    void release()
    {
        _released = true;
    }

    void operator()(T* ptr)
    { 
        if (!_released)
            delete ptr; 
    }

private:
    bool _released;    
};