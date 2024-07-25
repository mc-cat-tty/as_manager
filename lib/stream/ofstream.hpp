#include <iostream>
#include <fstream>
#include <memory>
#include <mutex>


class OfStream {
public:
    // Metodo statico per ottenere l'istanza del singleton
    static OfStream& getInstance() {
        static OfStream instance;
        return instance;
    }

    // Metodo per wrappare lo stream di output
    template <typename T>
    OfStream& operator<<(const T& data) {
        outputStream << data;
        return *this;
    }

    // Metodo per impostare lo stream di output
    void setOutputStream(std::ostream& stream) {
        outputStream.rdbuf(stream.rdbuf());
    }

private:
    // Costruttore privato per prevenire la creazione di istanze multiple
    OfStream() : outputStream(std::cout.rdbuf()) {}

    // Rimuovere copy constructor e assignment operator
    OfStream(const OfStream&) = delete;
    OfStream& operator=(const OfStream&) = delete;

    // Stream di output
    std::ostream outputStream;
};