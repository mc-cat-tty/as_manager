#include <iostream>
#include <fstream>

namespace as {
  class Logstream {
    public:
    // Metodo statico per ottenere l'istanza del singleton
    static Logstream& getInstance() {
        static Logstream instance;
        return instance;
    }

    template <typename T>
    Logstream& operator<<(const T& data) {
        outputStream << data;
        return *this;
    }

    void setOutputStream(std::ostream& stream) {
        outputStream.rdbuf(stream.rdbuf());
    }

    private:
    // Costruttore privato per prevenire la creazione di istanze multiple
    Logstream() : outputStream(std::cout.rdbuf()) {}

    Logstream(const Logstream&) = delete;
    Logstream& operator=(const Logstream&) = delete;

    // Stream di output
    std::ostream outputStream;
  };
}