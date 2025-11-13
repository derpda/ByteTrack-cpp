#pragma once

#include <cstddef>
#include <vector>

namespace byte_track {

std::tuple<std::vector<int>, std::vector<int>, double> exec_lapjv(
    std::vector<float> &&cost, size_t n_rows, size_t n_cols, bool extend_cost,
    float cost_limit, bool return_cost);

}
