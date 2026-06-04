#pragma once //include only once
#include <cstdint>

namespace tbot3_nav_monitor
{

enum class Nav2State : uint8_t
{
    UNKNOWN  = 0,
    SUCCEEDED = 1,
    ABORTED   = 2,
    CANCELED  = 3
};

}