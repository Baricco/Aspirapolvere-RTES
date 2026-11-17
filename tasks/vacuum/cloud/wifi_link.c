#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "config_check.h"
#include "wifi_link.h"

#define WIFI_RESPONSE_MAX   8192    // risposta più lunga (la lista delle reti)

// funzione helper che ritorna i millisecondi dall'avvio (orologio monotono)
static int64_t monotonic_ms(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

char *wifi_request(const char *request, int timeout_ms) {

    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return NULL;

    struct sockaddr_un address = { .sun_family = AF_UNIX };
    const char *path = getenv(WIFI_SOCKET_ENV) ? getenv(WIFI_SOCKET_ENV) : WIFI_SOCKET_PATH;
    snprintf(address.sun_path, sizeof address.sun_path, "%s", path);

    char *response = NULL;
    size_t length = 0;
    int64_t end = monotonic_ms() + timeout_ms;

    if (connect(fd, (struct sockaddr *)&address, sizeof address) != 0 ||
        send(fd, request, strlen(request), MSG_NOSIGNAL) < 0 || send(fd, "\n", 1, MSG_NOSIGNAL) < 0) {
        close(fd);
        return NULL;
    }

    response = malloc(WIFI_RESPONSE_MAX);
    if (response == NULL) {
        close(fd);
        return NULL;
    }

    // la risposta finisce con un a capo
    while (length + 1 < WIFI_RESPONSE_MAX) {
        int64_t left = end - monotonic_ms();
        struct pollfd p = { .fd = fd, .events = POLLIN };
        if (left <= 0 || poll(&p, 1, (int)left) <= 0) break;
        ssize_t n = recv(fd, response + length, WIFI_RESPONSE_MAX - 1 - length, 0);
        if (n <= 0) break;
        length += (size_t)n;
        if (response[length - 1] == '\n') break;
    }
    close(fd);

    if (length == 0 || response[length - 1] != '\n') {
        free(response);
        return NULL;
    }
    response[length - 1] = '\0';
    return response;
}
