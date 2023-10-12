#pragma once

#include <atomic>
#include <boost/program_options/variables_map.hpp>
#include <string>
#include <thread>


class Rover {
public:
    const std::string name;

    explicit Rover(boost::program_options::variables_map *);

    static void shutdown(int sig);

private:
    int serial_port;
    int udp_sock;
    std::string udp_host;
    int udp_port;
    std::string server_host;
    int server_port;

    std::thread control_thread;
    std::thread img_thread;

    void controlLoop() const;

    bool registerServer() const;
};