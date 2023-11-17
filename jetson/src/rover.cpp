#include "rover.hpp"

#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>

#include "ports.hpp"
#include "sync.hpp"

// TODO print error messages

Rover::Rover(boost::program_options::variables_map *options)
    : name(options->at("Options.name").as<std::string>()),
      udp_host(options->at("Options.udp_host").as<std::string>()),
      udp_port(stoi(options->at("Options.udp_port").as<std::string>())),
      server_host(options->at("Options.server_host").as<std::string>()),
      server_port(stoi(options->at("Options.server_port").as<std::string>())) {
    // create serial port
    this->serial_port = createSerialPort(
        options->at("Options.serial_addr").as<std::string>().c_str());

    if (this->serial_port < 0) {
        printf("\033[1;31mfailed to create serial port; exiting\033[0m\n");
        exit(1);
    }

    printf("created serial port: %d\n", this->serial_port);

    this->udp_sock = createUdpSocket(udp_host, udp_port);

    if (this->udp_sock < 0) {
        printf("\033[1;31mfailed to create udp socket; exiting\033[0m\n");
        exit(1);
    }

    printf("created udp socket: %d\n", this->udp_sock);

    printf("Startup Complete\n");
    printf("--------------------\n");

    running.store(true);

    this->control_thread = std::thread(&Rover::controlLoop, this);
    this->control_thread.join();
}

void Rover::shutdown(int sig) { running.store(false); }

void Rover::controlLoop() const {
    printf("control loop started\n");
    while (running.load()) {
        uint8_t message[3] = {0};

        if (recv(udp_sock, message, 3, 0) < 0) continue;

        switch (message[0]) {
            case 'E':
                if (write(serial_port, message, 1) < 0)
                    ;  // print error message
                printf("estop\n");
                running.store(false);
                break;
            case 'V':
                printf("velocity: %u %u\n", message[1], message[2]);
                if (write(serial_port, message, 3) < 0)
                    ;  // print error message
                break;
            case 'S':
                printf("stopping\n");
                if (write(serial_port, "Vdd", 3) < 0)
                    ;

                running.store(false);
                break;
            case 'B':
                printf("braking\n");
                if (write(serial_port, message, 1) < 0)
                    ;  // print error message
                break;
            case 'L':
                printf("Lift\n");
                if (write(serial_port, message, 2) < 0)
                    ;  // print error message
                break;
            default:
                break;
        }

        if (!running.load()) break;
        // read from serial
        uint8_t serial_message[6] = {0};
        if (read(serial_port, serial_message, 6) < 0)
            ;  // print error message

        int16_t cur = (int16_t)serial_message[1] << 8 | serial_message[2];
        int16_t vol = (int16_t)serial_message[3] << 8 | serial_message[4];
        printf("current: %d, voltage: %d\n", cur, vol);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // close serial port
    close(serial_port);

    // close udp socket
    close(udp_sock);
    printf("control loop ended\n");
}
