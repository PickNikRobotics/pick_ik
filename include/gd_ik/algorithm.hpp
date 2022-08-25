#pragma once

#include <algorithm>
#include <vector>

namespace gd_ik {

template <typename T>
auto select(std::vector<T> const& values, std::vector<size_t> const& indexes)
    -> std::vector<T> {
  std::vector<T> select;
  std::transform(indexes.cbegin(), indexes.cend(), select.begin(),
                 [&values](auto index) { return values.at(index); });
  return select;
}

}  // namespace gd_ik
