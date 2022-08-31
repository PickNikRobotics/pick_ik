#include <gd_ik/math.hpp>

namespace gd_ik {

auto clamp2(double v, double lo, double hi) -> double {
    if (v <= lo) v = lo;
    if (v >= hi) v = hi;
    return v;
}

}  // namespace gd_ik
