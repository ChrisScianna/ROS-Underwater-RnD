#include "diagnostic_tools/periodic_event_status.h"

#include <diagnostic_msgs/DiagnosticStatus.h>

namespace qna {
namespace diagnostic_tools {
PeriodicEventStatus::PeriodicEventStatus(const std::string& name)
    : PeriodicEventStatus(name, PeriodicEventStatusParams{}){}

PeriodicEventStatus::PeriodicEventStatus(const std::string& name, PeriodicEventStatusParams params)
    : PeriodicDiagnosticTask(name),
      params_(std::move(params)),
      short_term_period_(params.short_term_avg_window()),
      long_term_period_(params.long_term_avg_window()){}

void PeriodicEventStatus::tick(const ros::Time& stamp) {
  std::lock_guard<std::mutex> guard(mutex_);
  if (!last_stamp_.isZero()) {
    if (last_stamp_ <= stamp) {
      const double delta_in_seconds = (stamp - last_stamp_).toSec();
      if (delta_in_seconds <= params_.max_reasonable_period()) {
        short_term_period_.update(delta_in_seconds);
        long_term_period_.update(delta_in_seconds);
      } else {
        ROS_DEBUG_NAMED(
            "diagnostics_tools",
            "Time delta %f too long, ignoring",
            delta_in_seconds);
      }
    } else {
      ROS_DEBUG_NAMED(
          "diagnostics_tools",
          "Time went backwards from %f to %f, ignoring",
          last_stamp_.toSec(), stamp.toSec());
    }
  }
  last_stamp_ = stamp;
}

void PeriodicEventStatus::run(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  std::lock_guard<std::mutex> guard(mutex_);
  const double delta_in_seconds = (ros::Time::now() - last_stamp_).toSec();
  if (delta_in_seconds > params_.max_reasonable_period()) {
    // Unreasonable time delta, event is likely stale.
    short_term_period_.reset();
  }
  if (short_term_period_.sample_count() > 0) {
    if (short_term_period_.average() < params_.min_acceptable_period() ||
        short_term_period_.average() > params_.max_acceptable_period()) {
      stat.summary(params_.abnormal_diagnostic().status(),
                   params_.abnormal_diagnostic().description());
      if (params_.abnormal_diagnostic().has_code()) {
        stat.addf("Code", "%u", params_.abnormal_diagnostic().code());
      }
    } else {
      stat.summary(params_.normal_diagnostic().status(),
                   params_.normal_diagnostic().description());
      if (params_.normal_diagnostic().has_code()) {
        stat.addf("Code", "%u", params_.normal_diagnostic().code());
      }
    }
    stat.addf("Average period (short term)", "%f", short_term_period_.average());
    stat.addf("Minimum period (short term)", "%f", short_term_period_.minimum());
    stat.addf("Maximum period (short term)", "%f", short_term_period_.maximum());
  } else {
    stat.summary(params_.stale_diagnostic().status(),
                 params_.stale_diagnostic().description());
    if (params_.stale_diagnostic().has_code()) {
      stat.addf("Code", "%f", params_.stale_diagnostic().code());
    }
  }
  if (long_term_period_.sample_count() > 0) {
    stat.addf("Average period (long term)", "%f", long_term_period_.average());
    stat.addf("Minimum period (long term)", "%f", long_term_period_.minimum());
    stat.addf("Maximum period (long term)", "%f", long_term_period_.maximum());
  }
  stat.addf("Minimum acceptable period", "%f", params_.min_acceptable_period());
  stat.addf("Maximum acceptable period", "%f", params_.max_acceptable_period());
}

}  // namespace diagnostic_tools
}  // namespace qna
