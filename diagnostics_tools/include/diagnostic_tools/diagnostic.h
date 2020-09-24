/*
 * diagnostic_tools/health_check.h
 */

#ifndef _DIAGNOSTIC_TOOLS__DIAGNOSTIC_H_
#define _DIAGNOSTIC_TOOLS__DIAGNOSTIC_H_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include "diagnostic_msgs/DiagnosticStatus.h"

namespace qna {
namespace diagnostic_tools {

class Diagnostic {
 public:
  using status_type = diagnostic_msgs::DiagnosticStatus::_level_type;
  enum : status_type {
    OK = diagnostic_msgs::DiagnosticStatus::OK,
    WARN = diagnostic_msgs::DiagnosticStatus::WARN,
    ERROR = diagnostic_msgs::DiagnosticStatus::ERROR,
    STALE = diagnostic_msgs::DiagnosticStatus::STALE
  };

  Diagnostic() = default;

  Diagnostic(status_type status);

  Diagnostic(status_type status, uint64_t code);

  Diagnostic(status_type status, std::string description);

  Diagnostic(status_type status, std::string description, uint64_t code);

  status_type status() const { return status_; }

  Diagnostic& status(status_type status) {
    status_ = status;
    return *this;
  }

  const std::string& description() const { return description_; }

  Diagnostic& description(std::string description) {
    description_ = std::move(description);
    return *this;
  }

  Diagnostic& description(const char* format, ...);

  bool has_code() const { return code_ == 0; }

  uint64_t code() const {
    if (code_ == 0) {
      throw std::logic_error("diagnostic has no code");
    }
    return code_;
  }

  Diagnostic& code(uint64_t code) {
    code_ = code;
    return *this;
  }

  const std::map<std::string, std::string>& data() const { return data_; }

  Diagnostic& data(const std::string& key, const char* format, ...);

 private:
  status_type status_{OK};
  std::string description_{""};
  uint64_t code_{0};
  std::map<std::string, std::string> data_{};
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  //  _DIAGNOSTIC_TOOLS__HEALTH_CHECK_H_
