/*
 * diagnostic_tools/health_check.h
 */

#ifndef _DIAGNOSTIC_TOOLS__HEALTH_CHECK_H_
#define _DIAGNOSTIC_TOOLS__HEALTH_CHECK_H_

#include <map>
#include <mutex>
#include <string>

#include <boost/make_unique.hpp>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "diagnostic_tools/diagnostic.h"

namespace qna {
namespace diagnostic_tools {

template <typename... Ts>
class HealthCheck {
 public:
  using TestFunctionType = boost::function<Diagnostic(Ts...)>;

  HealthCheck() : impl_(boost::make_unique<Implementation>("Some health check")) {}

  HealthCheck(const std::string &name, TestFunctionType test_function)
      : impl_(boost::make_unique<Implementation>(name, test_function)) {}

  bool test(const Ts &... args) { return impl_->test(args...); }

  explicit operator bool() const { return impl_->has_test(); }

  operator diagnostic_updater::DiagnosticTask &() { return *impl_; }

 private:
  class Implementation : public diagnostic_updater::DiagnosticTask {
   public:
    Implementation(const std::string &name) : diagnostic_updater::DiagnosticTask(name) {}

    Implementation(const std::string &name, TestFunctionType test_function)
        : diagnostic_updater::DiagnosticTask(name), test_function_(test_function) {}

    bool test(const Ts &... args) {
      if (test_function_) {
        std::lock_guard<std::mutex> guard(mutex_);
        diagnostic_ = test_function_(args...);
        if (diagnostic_.description().empty()) {
          diagnostic_.description(description_for(diagnostic_.status()));
        }
      }
      return diagnostic_.status() == Diagnostic::OK;
    }

    bool has_test() const { return test_function_; }

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override {
      std::lock_guard<std::mutex> guard(mutex_);
      stat.summary(diagnostic_.status(), diagnostic_.description());
      if (diagnostic_.has_code()) {
        stat.add("Code", diagnostic_.code());
      }
      for (const auto &kv : diagnostic_.data()) {
        stat.add(kv.first, kv.second);
      }
    }

   private:
    static const char *description_for(Diagnostic::status_type status) {
      switch (status) {
        case Diagnostic::OK:
          return "Everything OK";
        case Diagnostic::WARN:
          return "Something doesn't seem right";
        case Diagnostic::ERROR:
          return "Something's wrong";
        case Diagnostic::STALE:
          return "Not enough tests conducted";
        default:
          throw std::logic_error("Invalid diagnostic status");
      }
    }

    Diagnostic diagnostic_;
    TestFunctionType test_function_;
    std::mutex mutex_;
  };

  std::unique_ptr<Implementation> impl_;
};

template <typename... Ts>
HealthCheck<Ts...> create_health_check(
    const std::string &name, typename HealthCheck<Ts...>::TestFunctionType test_function) {
  return HealthCheck<Ts...>(name, test_function);
}

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // _DIAGNOSTIC_TOOLS__INTERNAL_DIAGNOSTIC_TASK_H_
