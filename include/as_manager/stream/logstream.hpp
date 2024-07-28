#include <iostream>
#include <sstream>


namespace as::logstream {
  enum class OutputLog {
    STANDARD = 0,
    ROS
  };

template <OutputLog typeOfOutput = OutputLog::STANDARD>
class Logstream : public std::ostream {
public:
    Logstream(std::string_view level) : std::ostream(&buf), level(level) {}

    template <typename T>
    Logstream& operator<<(const T& data) {
        std::ostringstream oss;
        oss << "[" << level << "] " << data;
        buf.sputn(oss.str().c_str(), oss.str().size());
        return *this;
    }

    Logstream& operator<<(std::ostream& (*manip)(std::ostream&)) {
        if (manip == static_cast<std::ostream& (*)(std::ostream&)>(std::endl)) {
            if constexpr (typeOfOutput == OutputLog::STANDARD) {
                std::cout << std::endl;
            } else {
                std::cerr << std::endl;
            }
        }
        return *this;
    }

private:
    class Buffer : public std::streambuf {
    public:
        int overflow(int c) override {
            if (c != EOF) {
                if constexpr (typeOfOutput == OutputLog::STANDARD) {
                    std::cout.put(c);
                } else {
                    std::cerr.put(c);
                }
            }
            return c;
        }
    };

    Buffer buf;
    std::string_view level;
};

  Logstream log=Logstream("LOG");
  Logstream error=Logstream<OutputLog::ROS>("ERRORE");
}
}