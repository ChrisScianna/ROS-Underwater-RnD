#include "diagnostic_tools/periodic_event_status.h"

#include <diagnostic_msgs/DiagnosticStatus.h>

namespace qna {
namespace diagnostic_tools {

PeriodicEventStatus::PeriodicEventStatus(const std::string& name)
    : PeriodicEventStatus(name, PeriodicEventStatusParams{}) {}

PeriodicEventStatus::PeriodicEventStatus(const std::string& name, PeriodicEventStatusParams params)
    : PeriodicDiagnosticTask(name), params_(std::move(params)) {}

void PeriodicEventStatus::tick(const ros::Time& stamp) {
  std::lock_guard<std::mutex> guard(mutex_);
  if (!last_stamp_.isZero()) {
    if (stamp < last_stamp_) {
      ROS_WARN_NAMED("diagnostics_tools", "Time went backwards from %f to %f, resetting.",
                     last_stamp_.toSec(), stamp.toSec());
      last_cycle_period_.reset();
    } else {
      const double delta_in_seconds = (stamp - last_stamp_).toSec();
      last_cycle_period_.update(delta_in_seconds);
      historic_period_.update(delta_in_seconds);
    }
  }
  last_stamp_ = stamp;
}

void PeriodicEventStatus::run(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  std::lock_guard<std::mutex> guard(mutex_);
  if (last_cycle_period_.sample_count() > 0) {
    if (last_cycle_period_.average() < params_.min_acceptable_period() ||
        last_cycle_period_.average() > params_.max_acceptable_period()) {
      stat.summary(params_.abnormal_diagnostic().status(),
                   params_.abnormal_diagnostic().description());
      if (params_.abnormal_diagnostic().has_code()) {
        stat.addf("Code", "%u", params_.abnormal_diagnostic().code());
      }
    } else {
      stat.summary(params_.normal_diagnostic().status(), params_.normal_diagnostic().description());
      if (params_.normal_diagnostic().has_code()) {
        stat.addf("Code", "%u", params_.normal_diagnostic().code());
      }
    }
    stat.addf("Average period (last cycle)", "%f", last_cycle_period_.average());
    stat.addf("Minimum period (last cycle)", "%f", last_cycle_period_.minimum());
    stat.addf("Maximum period (last cycle)", "%f", last_cycle_period_.maximum());
  } else {
    stat.summary(params_.stale_diagnostic().status(), params_.stale_diagnostic().description());
    if (params_.stale_diagnostic().has_code()) {
      stat.addf("Code", "%f", params_.stale_diagnostic().code());
    }
  }
  if (historic_period_.sample_count() > 0) {
    stat.addf("Average period (historic)", "%f", historic_period_.average());
    stat.addf("Minimum period (historic)", "%f", historic_period_.minimum());
    stat.addf("Maximum period (historic)", "%f", historic_period_.maximum());
  }
  stat.addf("Minimum acceptable period", "%f", params_.min_acceptable_period());
  stat.addf("Maximum acceptable period", "%f", params_.max_acceptable_period());

  last_cycle_period_.reset();
}

}  // namespace diagnostic_tools
}  // namespace qna
