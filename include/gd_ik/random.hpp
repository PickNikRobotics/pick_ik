#pragma once

#include <optional>
#include <random>
#include <vector>

namespace gd_ik {

auto random_uniform() -> double;
auto random_normal() -> double;
auto random_index(size_t size) -> size_t;

struct RandomBuffer {
  std::vector<double> values;
  size_t index = 0;
};

auto next(RandomBuffer& self) -> double;

auto make_random_uniform_buffer() -> RandomBuffer;
auto make_random_normal_buffer() -> RandomBuffer;

}  // namespace gd_ik
