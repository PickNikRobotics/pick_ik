#pragma once

#include <algorithm>
#include <vector>

namespace gd_ik {

template <typename T>
auto select_indexes(std::vector<T> const& values,
                    std::vector<size_t> const& indexes) {
  std::vector<T> res;
  std::transform(indexes.cbegin(), indexes.cend(), res.begin(),
                 [&values](auto index) { return values.at(index); });
  return res;
}

}  // namespace gd_ik
