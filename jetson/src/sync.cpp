#include <csignal>
#include <mutex>
#include "sync.hpp"

std::atomic_bool running;

// mutex for udp socket
std::mutex udp_mutex;

void signal_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) running.store(false);
}

void setupSignalHandler() {
    struct sigaction sigIntHandler {};
    sigIntHandler.sa_handler = signal_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, nullptr);
}