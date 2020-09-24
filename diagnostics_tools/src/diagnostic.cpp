#include "diagnostic_tools/diagnostic.h"

#include <cstdarg>
#include <cstring>
#include <memory>

namespace qna {
namespace diagnostic_tools {

Diagnostic::Diagnostic(Diagnostic::status_type status)
    : status_(status), description_(""), code_(0) {}

Diagnostic::Diagnostic(Diagnostic::status_type status, std::string description)
    : status_(status), description_(std::move(description)), code_(0) {}

Diagnostic::Diagnostic(Diagnostic::status_type status, uint64_t code)
    : status_(status), description_(""), code_(code) {}

Diagnostic::Diagnostic(Diagnostic::status_type status, std::string description, uint64_t code)
    : status_(status), description_(std::move(description)), code_(code) {
  if (description.empty()) {
    throw std::logic_error("Diagnostic description is empty");
  }
}

namespace {

std::string vaformat(const char* format, va_list va) {
  va_list other_va;
  va_copy(other_va, va);
  int size = vsnprintf(NULL, 0, format, other_va);
  va_end(other_va);
  if (size < 0) {
    return "Failed to format";
  }
  std::unique_ptr<char> buffer(new (std::nothrow) char[size + 1]);
  if (!buffer) {
    return "Failed to format";
  }
  if (vsnprintf(buffer.get(), size + 1, format, va) < 0) {
    return "Failed to format";
  }
  return buffer.get();
}

}  // namespace

Diagnostic& Diagnostic::description(const char* format, ...) {
  va_list va;
  va_start(va, format);
  description_ = vaformat(format, va);
  va_end(va);
  return *this;
}

Diagnostic& Diagnostic::data(const std::string& key, const char* format, ...) {
  va_list va;
  va_start(va, format);
  data_[key] = vaformat(format, va);
  va_end(va);
  return *this;
}

}  // namespace diagnostic_tools
}  // namespace qna
