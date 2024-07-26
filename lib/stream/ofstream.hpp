#include <iostream>
#include <fstream>
#include <memory>

namespace as {
  class OfStream {
    public:
    // Metodo statico per ottenere l'istanza del singleton
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
    // Costruttore privato per prevenire la creazione di istanze multiple
    OfStream() : outputStream(std::cout.rdbuf()) {}

    OfStream(const OfStream&) = delete;
    OfStream& operator=(const OfStream&) = delete;

    // Stream di output
    std::ostream outputStream;
  };
}