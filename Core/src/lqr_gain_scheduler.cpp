#include "lqr_gain_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace {

constexpr size_t kLqrStateSize = 6U;
constexpr size_t kLqrInputSize = 2U;
constexpr size_t kKValuesPerEntry = kLqrStateSize * kLqrInputSize;

}  // namespace

LqrGainScheduler::LqrGainScheduler()
{
    ground_gain_.setZero();
    air_gain_.setZero();
}

bool LqrGainScheduler::load_table(const std::vector<double>& lengths,
                                  const std::vector<double>& values,
                                  std::string& error)
{
    if (lengths.empty()) {
        error = "K.lengths must contain at least one leg length";
        return false;
    }
    if (values.size() != lengths.size() * kKValuesPerEntry) {
        error = "K.values must contain exactly 12 values for each K.lengths entry";
        return false;
    }

    table_.clear();
    table_.reserve(lengths.size());
    for (size_t entry_index = 0; entry_index < lengths.size(); ++entry_index) {
        const double length = lengths[entry_index];
        if (!std::isfinite(length)) {
            error = "K.lengths contains a non-finite value";
            table_.clear();
            return false;
        }

        LqrGainMatrix gain;
        for (size_t row = 0; row < kLqrInputSize; ++row) {
            for (size_t column = 0; column < kLqrStateSize; ++column) {
                const size_t value_index =
                    entry_index * kKValuesPerEntry + row * kLqrStateSize + column;
                const double value = values[value_index];
                if (!std::isfinite(value)) {
                    error = "K.values contains a non-finite value";
                    table_.clear();
                    return false;
                }
                gain(row, column) = value;
            }
        }
        table_.emplace_back(length, gain);
    }

    std::sort(table_.begin(),
              table_.end(),
              [](const TableEntry& lhs, const TableEntry& rhs) {
                  return lhs.first < rhs.first;
              });

    for (size_t index = 1; index < table_.size(); ++index) {
        if (table_[index].first <= table_[index - 1].first) {
            error = "K.lengths entries must be unique";
            table_.clear();
            return false;
        }
    }

    return update(table_.front().first);
}

bool LqrGainScheduler::update(double leg_length)
{
    if (table_.empty() || !std::isfinite(leg_length)) {
        return false;
    }

    if (leg_length <= table_.front().first || table_.size() == 1U) {
        ground_gain_ = table_.front().second;
    } else if (leg_length >= table_.back().first) {
        ground_gain_ = table_.back().second;
    } else {
        const auto upper = std::lower_bound(
            table_.begin(),
            table_.end(),
            leg_length,
            [](const TableEntry& entry, double length) {
                return entry.first < length;
            });
        const auto lower = std::prev(upper);
        const double ratio =
            (leg_length - lower->first) / (upper->first - lower->first);
        ground_gain_ = (1.0 - ratio) * lower->second + ratio * upper->second;
    }

    air_gain_.setZero();
    air_gain_(1, 2) = ground_gain_(1, 2);
    air_gain_(1, 3) = ground_gain_(1, 3);
    return true;
}

bool LqrGainScheduler::empty() const
{
    return table_.empty();
}

const LqrGainMatrix& LqrGainScheduler::ground_gain() const
{
    return ground_gain_;
}

const LqrGainMatrix& LqrGainScheduler::air_gain() const
{
    return air_gain_;
}
