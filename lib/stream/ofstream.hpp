#include <iostream>
#include <fstream>
#include <memory>
#include <mutex>


class OfStream {
public:
    static OfStream& getInstance() {
        static OfStream instance;
        return instance;
    }

    template <typename T>
    OfStream& operator<<(const T& data) {
        outputStream << data;
        return *this;
    }

    void setOutputStream(std::ostream& stream) {
        outputStream.rdbuf(stream.rdbuf());
    }

private:
    OfStream() : outputStream(std::cout.rdbuf()) {}

    OfStream(const OfStream&) = delete;
    OfStream& operator=(const OfStream&) = delete;

    std::ostream outputStream;
};