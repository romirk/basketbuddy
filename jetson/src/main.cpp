#include <rover.hpp>
#include <sync.hpp>
#include <config.hpp>

int main() {
    setupSignalHandler();
    auto options = Config::load("config/default.ini");

    Rover rover(options);
    return 0;
}
