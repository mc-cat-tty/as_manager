#include <exception>

namespace as {
  class EmergencyException : std::exception {
    virtual const char* what() const noexcept {
      return "Emergency Exception thrown";
    }
  };
};