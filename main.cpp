#include <actions/actions.hpp>
#include <fsm_manager/fsm_manager.hpp>
#include <signals/low_pass_filter.hpp>
#include <hal/hal.hpp>
#include <signals/signals.hpp>
#include <signals/updater.hpp>
#include <stream/logstream.hpp>

int main() {
    auto& ofs = as::Logstream::getInstance();
    ofs.setOutputStream(std::cout);
    ofs << "Hello world!";
    return 0;
}