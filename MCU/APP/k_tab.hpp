#pragma once

#include <cstddef>

inline constexpr std::size_t kGainTableLengthCount = 17U;
inline constexpr std::size_t kGainTableValuesPerLength = 12U;
inline constexpr std::size_t kGainTableValueCount =
    kGainTableLengthCount * kGainTableValuesPerLength;

extern float k_tab[kGainTableValueCount];
extern float k_length[kGainTableLengthCount];
