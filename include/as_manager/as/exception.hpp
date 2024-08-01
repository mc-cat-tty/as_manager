#pragma once

#include <exception>

namespace as {
  class EmergencyException : std::exception {
    public:
    virtual const char* what() const noexcept {
      return "Emergency Exception thrown";
    }
  };
};