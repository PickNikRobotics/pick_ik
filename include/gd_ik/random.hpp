#pragma once

#include <optional>
#include <random>
#include <vector>

namespace gd_ik {
// Function that returns reference to thread_local random generator
// You can only seed once per-thread, if you try to call this with a
// seed_sequence after the first time it has been called on that thread
// it will throw.
std::mt19937& rng(std::optional<std::seed_seq> seed_sequence = std::nullopt);

template <typename T>
T random_uniform_real(T const lower, T const upper) {
  return std::uniform_real_distribution<T>{lower, upper}(rng());
}

template <typename T>
T random_uniform_int(T const lower, T const upper) {
  return std::uniform_int_distribution<T>{lower, upper}(rng());
}

double random_uniform();
double random_normal();
size_t random_index(size_t size);

struct RandomBuffer {
  std::vector<double> values;
  size_t index = 0;
};

double next(RandomBuffer& self);

RandomBuffer make_random_uniform_buffer();
RandomBuffer make_random_normal_buffer();

}  // namespace gd_ik
