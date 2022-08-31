#pragma once

#include <algorithm>
#include <cassert>
#include <vector>

namespace gd_ik {

template <typename T>
auto select_indexes(std::vector<T> const& values, std::vector<size_t> const& indexes) {
    std::vector<T> res;
    std::transform(indexes.cbegin(), indexes.cend(), std::back_inserter(res),
                   [&values](auto index) { return values.at(index); });
    return res;
}

template <typename T>
auto set_indexes(std::vector<T> const& initial_state, std::vector<T> const& values,
                 std::vector<size_t> const& indexes) {
    auto ret = initial_state;
    for (size_t i = 0; i < values.size(); ++i) {
        ret.at(indexes.at(i)) = values.at(i);
    }
    return ret;
}

}  // namespace gd_ik
