#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>

#include <cstdio>
#include <cstring>
#include <ports.hpp>
#include <string>

int createUdpSocket(const std::string &server, int port) {
    int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) {
        printf("\033[1;31mfailed to create udp socket\033[0m\n");
        return -1;
    }

    struct sockaddr_in udp_addr {};
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_port = htons(port);
    udp_addr.sin_addr.s_addr = inet_addr(server.c_str());
    memset(udp_addr.sin_zero, '\0', sizeof(udp_addr.sin_zero));

    // set socket timeout
    struct timeval tv {};
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    if (setsockopt(udp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        printf("\033[1;31mfailed to set socket timeout\033[0m\n");
        return -1;
    }

    // bind socket
    if (bind(udp_sock, (struct sockaddr *)&udp_addr, sizeof(udp_addr)) < 0) {
        printf("\033[1;31mfailed to bind udp socket\033[0m\n");
        return -1;
    }
    return udp_sock;
}

int createSerialPort(const char *serial_addr) {
    int serial_port = open(serial_addr, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port < 0) {
        printf("\033[1;31mfailed to open serial port %s\033[0m\n", serial_addr);
        return -1;
    }

    termios tty{};
    if (tcgetattr(serial_port, &tty) != 0) {
        printf("\033[1;31mfailed to get serial port attributes\033[0m\n");
        return -1;
    }

    cfsetspeed(&tty, B115200);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("\033[1;31mfailed to set serial port attributes\033[0m\n");
        return -1;
    }

    auto val = fcntl(serial_port, F_GETFL, 0);

    return serial_port;
}

int resolveHelper(const char *hostname, int family, const char *service,
                  sockaddr_storage *pAddr) {
    int result;
    addrinfo *result_list = NULL;
    addrinfo hints = {};
    hints.ai_family = family;
    hints.ai_socktype =
        SOCK_DGRAM;  // without this flag, getaddrinfo will return 3x the number
                     // of addresses (one for each socket type).
    result = getaddrinfo(hostname, service, &hints, &result_list);
    if (result == 0) {
        // ASSERT(result_list->ai_addrlen <= sizeof(sockaddr_in));
        memcpy(pAddr, result_list->ai_addr, result_list->ai_addrlen);
        freeaddrinfo(result_list);
    }

    return result;
}

ssize_t sendUdp(int sock, const char *hostname, int service, const char *data,
                ssize_t size) {
    ssize_t result = 0;

    char szIP[100];

    sockaddr_in addrListen =
        {};  // zero-int, sin_port is 0, which picks a random port for bind.
    addrListen.sin_family = AF_INET;

    sockaddr_storage addrDest = {};
    result = resolveHelper(hostname, AF_INET, std::to_string(service).c_str(),
                           &addrDest);
    if (result != 0) {
        printf("\033[1;31mfailed to resolve hostname\033[0m\n");
        return -1;
    }

    result =
        sendto(sock, data, size, 0, (sockaddr *)&addrDest, sizeof(addrDest));

    return result;
}
