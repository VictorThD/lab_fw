#pragma once

#include "templates.h"
#include "types.h"
#include "cstdint"

struct ICmdProcessor {    
    virtual bool onCommand(const char *command, size_t length) = 0; 
};

template <typename T> void fetch_param(const char *pstr, size_t len, T &param)
{
    assert(0, "Unknown type");        
}    

template <> void fetch_param(const char *pstr, size_t len, FWT::str &str);
template <> void fetch_param(const char *pstr, size_t len, int &num);
template <> void fetch_param(const char *pstr, size_t len, uint16_t &num);
template <> void fetch_param(const char *pstr, size_t len, size_t &num);

template <typename Tail>
    void FillParamTuple(const char *params, size_t length, FWT::tuple<Tail> &tup)
{
    Tail val;
    fetch_param(params, length, val);
    tup.set(val);
}

template <typename Head, typename ...Tail>
    void FillParamTuple(const char *params, size_t length, FWT::tuple<Head, Tail...> &tup)
{
    for (const char *param_start = params, *ptr = params; (ptr - params) < length; ++ptr)
    {
        switch (*ptr)
        {
        case ' ':
        case '\t':
            Head val;
            fetch_param(param_start, ptr - param_start, val);
            tup.set(val);
            ptr++;
            FillParamTuple<Tail...>(ptr, length - (ptr - params), tup.get_rest());
            return;
        }
    }
}

template <>
    void FillParamTuple(const char *params, size_t length, FWT::tuple<void> &tup);

template <typename Handler_Fn_T /* void (const FWT::tuple<params...> &params)*/, typename ...Args> 
class CommandHandler : public ICmdProcessor {
public:    
    
    virtual bool onCommand(const char *command, size_t length) override 
    {
        size_t cmd_len = FWT::min(strlen(_command_name), length);

        // Command parsing should be replaced with FSM
        if (strncmp(command, _command_name, cmd_len) != 0 ||
            (command[cmd_len] != ' ' && command[cmd_len] != '\r' && command[cmd_len] != '\n' && command[cmd_len] != '\n' && command[cmd_len] != 0))
            return false;

        const char *param_start = command + strlen(_command_name);
        while (*param_start == ' ')
            ++param_start;

        FWT::tuple<Args...> params; 
        FillParamTuple<Args...>(param_start, length - (param_start - command), params);

        _handler(params);

        return true; 
    }

    CommandHandler(const char *command_name, const Handler_Fn_T &handler) : 
        _command_name(command_name),
        _handler(handler)
    {

    }
private:
    FWT::str _command_name;
    FWT::function<Handler_Fn_T> _handler;
};

template <typename Fn_T, typename ...Args> auto MakeCommandHandler(const char *command_name, const Fn_T&fn)
{
    return CommandHandler<Fn_T, Args...>(command_name, fn);
}

