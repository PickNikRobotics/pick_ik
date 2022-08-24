#include "gd_ik/random.hpp"

#include <array>
#include <functional>
#include <optional>
#include <random>
#include <rsl/rand.hpp>

namespace gd_ik {
namespace {

constexpr auto kRandomBufferSize = size_t(1024 * 1024 * 8);

}

auto random_uniform() -> double { return rsl::uniform_real<double>(0.0, 1.0); }

auto random_normal() -> double {
  return std::normal_distribution<double>{}(rsl::rng());
}

auto random_index(size_t size) -> size_t {
  return rsl::uniform_int<size_t>(0, size - 1);
}

auto next(RandomBuffer& self) -> double {
  return self.values[self.index++ & (kRandomBufferSize - 1)];
}

auto make_random_uniform_buffer() -> RandomBuffer {
  RandomBuffer buffer;
  buffer.index = 0;
  buffer.values.resize(kRandomBufferSize);
  for (auto& val : buffer.values) {
    val = random_uniform();
  }

  return buffer;
}

auto make_random_normal_buffer() -> RandomBuffer {
  RandomBuffer buffer;
  buffer.index = 0;
  buffer.values.resize(kRandomBufferSize);
  for (auto& val : buffer.values) {
    val = random_normal();
  }

  return buffer;
}

}  // namespace gd_ik
