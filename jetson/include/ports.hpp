#pragma once

#include <string>

#define SERIAL_ADDR "/dev/ttyACM0"
#define UDP_PORT 5000
#define UDP_HOST "0.0.0.0"

int createSerialPort(const char *serial_addr);

int createUdpSocket(const std::string &server, int port);

ssize_t sendUdp(int sock, const char *hostname, int port, const char *data,
                ssize_t size);