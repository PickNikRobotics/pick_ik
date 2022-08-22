#pragma once

#include <optional>
#include <random>
#include <vector>

namespace gd_ik {
// Function that returns reference to thread_local random generator
// You can only seed once per-thread, if you try to call this with a
// seed_sequence after the first time it has been called on that thread
// it will throw.
auto rng(std::optional<std::seed_seq> seed_sequence = std::nullopt)
    -> std::mt19937&;

template <typename T>
auto random_uniform_real(T const lower, T const upper) -> T {
  return std::uniform_real_distribution<T>{lower, upper}(rng());
}

template <typename T>
auto random_uniform_int(T const lower, T const upper) -> T {
  return std::uniform_int_distribution<T>{lower, upper}(rng());
}

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
