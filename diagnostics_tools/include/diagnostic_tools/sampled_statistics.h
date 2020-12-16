/*
 * diagnostic_tools/sampled_statistics.h
 */

#ifndef _DIAGNOSTIC_TOOLS__SAMPLED_STATISTICS_H_
#define _DIAGNOSTIC_TOOLS__SAMPLED_STATISTICS_H_

#include <queue>

namespace qna
{
namespace diagnostic_tools
{
template <typename T>
class SampledStatistics
{
public:
  explicit SampledStatistics(T windows_size) { windows_size_ = windows_size; }

  void reset()
  {
    average_ = T{};
    minimum_ = T{};
    maximum_ = T{};

    std::queue<T> empty;
    std::swap(buffer_, empty);
  }

  void update(const T &sample)
  {
    if (buffer_.size() > 0)
    {
      if (maximum_ < sample)
      {
        maximum_ = sample;
      }
      else if (minimum_ > sample)
      {
        minimum_ = sample;
      }
    }
    else
    {
      minimum_ = maximum_ = sample;
    }

    if (buffer_.size() == windows_size_)
    {
      accumulate_ -= buffer_.front();
      buffer_.pop();
    }
    buffer_.push(sample);
    accumulate_ += sample;
    average_ = (accumulate_ / buffer_.size());
  }

  size_t sample_count() const { return buffer_.size(); }

  T average() const { return average_; }

  T maximum() const { return maximum_; }

  T minimum() const { return minimum_; }

private:
  T average_{};
  T minimum_{};
  T maximum_{};
  T accumulate_{0};
  std::queue<T> buffer_;
  T windows_size_{};
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // _DIAGNOSTIC_TOOLS__SAMPLED_STATISTICS_H_
