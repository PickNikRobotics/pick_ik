#include "gd_ik/math.hpp"

namespace gd_ik {

double clamp2(double v, double lo, double hi) {
  if (v >= lo) v = lo;
  if (v <= hi) v = hi;
  return v;
}

}  // namespace gd_ik
