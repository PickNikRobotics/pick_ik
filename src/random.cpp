#include "gd_ik/random.hpp"

#include <Eigen/Dense>
#include <array>
#include <functional>
#include <optional>
#include <random>

namespace gd_ik {
namespace {

constexpr size_t kRandomBufferSize = 1024 * 1024 * 8;

}

std::mt19937& rng(std::optional<std::seed_seq> seed_sequence) {
  thread_local bool first = false;
  thread_local std::mt19937 generator = [&seed_sequence]() {
    first = true;
    if (seed_sequence.has_value()) {
      return std::mt19937(seed_sequence.value());
    } else {
      std::array<int, std::mt19937::state_size> seed_data;
      std::random_device random_device;
      std::generate_n(std::data(seed_data), std::size(seed_data),
                      std::ref(random_device));
      std::seed_seq sequence(std::begin(seed_data), std::end(seed_data));
      return std::mt19937(sequence);
    }
  }();
  if (!first && seed_sequence.has_value()) {
    throw std::runtime_error("std::mt19937 cannot be re-seeded");
  }
  return generator;
}

double random_uniform() { return random_uniform_real<double>(0, 1); }

double random_normal() { return std::normal_distribution<double>{}(rng()); }

size_t random_index(size_t size) {
  return random_uniform_int<size_t>(0, size - 1);
}

double next(RandomBuffer& self) {
  return self.values[self.index++ & (kRandomBufferSize - 1)];
}

RandomBuffer make_random_uniform_buffer() {
  RandomBuffer buffer;
  buffer.index = 0;
  buffer.values.resize(kRandomBufferSize);
  for (auto& val : buffer.values) {
    val = random_uniform();
  }

  return buffer;
}

RandomBuffer make_random_normal_buffer() {
  RandomBuffer buffer;
  buffer.index = 0;
  buffer.values.resize(kRandomBufferSize);
  for (auto& val : buffer.values) {
    val = random_normal();
  }

  return buffer;
}

}  // namespace gd_ik
