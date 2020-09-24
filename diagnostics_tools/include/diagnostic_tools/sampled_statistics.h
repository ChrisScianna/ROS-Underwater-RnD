/*
 * diagnostic_tools/sampled_statistics.h
 */

#ifndef _DIAGNOSTIC_TOOLS__SAMPLED_STATISTICS_H_
#define _DIAGNOSTIC_TOOLS__SAMPLED_STATISTICS_H_

namespace qna {
namespace diagnostic_tools {

template <typename T>
class SampledStatistics {
 public:
  void reset() {
    average_ = T{};
    minimum_ = T{};
    maximum_ = T{};
    sample_count_ = 0;
  }

  void update(const T& sample) {
    average_ = (sample + average_ * sample_count_) / (sample_count_ + 1);
    if (sample_count_ > 0) {
      if (maximum_ < sample) {
        maximum_ = sample;
      } else if (minimum_ > sample) {
        minimum_ = sample;
      }
    } else {
      minimum_ = maximum_ = sample;
    }
    sample_count_++;
  }

  size_t sample_count() const { return sample_count_; }

  T average() const { return average_; }

  T maximum() const { return maximum_; }

  T minimum() const { return minimum_; }

 private:
  T average_{};
  T minimum_{};
  T maximum_{};
  size_t sample_count_{0};
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // _DIAGNOSTIC_TOOLS__SAMPLED_STATISTICS_H_
