#include <pick_ik/math.hpp>

namespace pick_ik {

auto clamp2(double v, double lo, double hi) -> double {
    if (v <= lo) v = lo;
    if (v >= hi) v = hi;
    return v;
}

}  // namespace pick_ik
