#pragma once

#include <cmath>

namespace lib15442c {
    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }
}