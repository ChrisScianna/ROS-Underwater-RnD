/*
 * diagnostic_tools/periodic_event_status.h
 */

#ifndef _DIAGNOSTIC_TOOLS__PERIODIC_EVENT_STATUS_H_
#define _DIAGNOSTIC_TOOLS__PERIODIC_EVENT_STATUS_H_

#include <mutex>

#include <diagnostic_updater/diagnostic_updater.h>

#include <ros/ros.h>

#include "diagnostic_tools/diagnostic.h"
#include "diagnostic_tools/periodic_diagnostic_task.h"
#include "diagnostic_tools/sampled_statistics.h"

namespace qna {
namespace diagnostic_tools {

class PeriodicEventStatusParams final {
 public:
  PeriodicEventStatusParams() = default;

  PeriodicEventStatusParams &min_acceptable_period(double min_acceptable_period) {
    min_acceptable_period_ = min_acceptable_period;
    return *this;
  }

  double min_acceptable_period() const { return min_acceptable_period_; }

  PeriodicEventStatusParams &max_acceptable_period(double max_acceptable_period) {
    max_acceptable_period_ = max_acceptable_period;
    return *this;
  }

  double max_acceptable_period() const { return max_acceptable_period_; }

  PeriodicEventStatusParams &normal_diagnostic(Diagnostic diagnostic) {
    if (diagnostic.description().empty()) {
      diagnostic.description("Rate within tolerance");
    }
    normal_diagnostic_ = std::move(diagnostic);
    return *this;
  }

  const Diagnostic &normal_diagnostic() const { return normal_diagnostic_; }

  PeriodicEventStatusParams &abnormal_diagnostic(Diagnostic diagnostic) {
    if (diagnostic.description().empty()) {
      diagnostic.description("Rate too high or too low");
    }
    abnormal_diagnostic_ = std::move(diagnostic);
    return *this;
  }

  const Diagnostic &abnormal_diagnostic() const { return abnormal_diagnostic_; }

  PeriodicEventStatusParams &stale_diagnostic(Diagnostic diagnostic) {
    if (diagnostic.description().empty()) {
      diagnostic.description("Not enough data since last update");
    }
    stale_diagnostic_ = std::move(diagnostic);
    return *this;
  }

  const Diagnostic &stale_diagnostic() const { return stale_diagnostic_; }

 private:
  double min_acceptable_period_{-1};
  double max_acceptable_period_{5};
  Diagnostic normal_diagnostic_{Diagnostic::OK, "Rate within tolerance"};
  Diagnostic abnormal_diagnostic_{Diagnostic::WARN, "Rate too high or too low"};
  Diagnostic stale_diagnostic_{Diagnostic::STALE, "Not enough data since last update"};
};

class PeriodicEventStatus : public PeriodicDiagnosticTask {
 public:
  PeriodicEventStatus(const std::string &name);

  PeriodicEventStatus(const std::string &name, PeriodicEventStatusParams params);

  void tick(const ros::Time &stamp) override;

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

 private:
  PeriodicEventStatusParams params_;
  SampledStatistics<double> last_cycle_period_;
  SampledStatistics<double> historic_period_;
  ros::Time last_stamp_;
  std::mutex mutex_;
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // _DIAGNOSTIC_TOOLS__PERIODIC_EVENT_STATUS_H_
