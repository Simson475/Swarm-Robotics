#include <exception>
#include <iostream>
#include <string>

class Exception : public std::exception
{
    std::string _msg;
public:
    Exception(const std::string& msg) : _msg(msg){}

    virtual const char* what() const noexcept override
    {
        return _msg.c_str();
    }
}; 