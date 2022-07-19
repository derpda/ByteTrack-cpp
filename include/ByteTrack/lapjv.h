#pragma once

#include <cstddef>
#include <vector>

namespace byte_track {
int lapjv_internal(const size_t n, const std::vector<float>& cost,
                   std::vector<int>& x, std::vector<int>& y);
}
