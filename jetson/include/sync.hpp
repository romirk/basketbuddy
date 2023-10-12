#pragma once

#include <atomic>
#include <mutex>

extern std::atomic_bool running;
extern std::mutex udp_mutex;

void signal_handler(int sig);

void setupSignalHandler();