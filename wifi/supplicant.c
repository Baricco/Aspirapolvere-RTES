#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "supplicant.h"
#include "wifi.h"

#define COMMAND_TIMEOUT_MS  5000    // attesa massima della risposta a un comando (SAVE_CONFIG scrive su flash)

bool supplicant_open(supplicant_link_t *link, const char *ctrl_path, bool is_events) {

    static unsigned counter;

    link->fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (link->fd < 0) return false;

    // il nostro capo del socket: il programma risponde a questo indirizzo
    struct sockaddr_un local = { .sun_family = AF_UNIX };
    snprintf(link->local_path, sizeof link->local_path, "%s/ctrl-%d-%u", WIFI_RUN_DIR, (int)getpid(), counter++);
    snprintf(local.sun_path, sizeof local.sun_path, "%s", link->local_path);
    unlink(local.sun_path);

    struct sockaddr_un remote = { .sun_family = AF_UNIX };
    snprintf(remote.sun_path, sizeof remote.sun_path, "%s", ctrl_path);

    if (bind(link->fd, (struct sockaddr *)&local, sizeof local) != 0 ||
        connect(link->fd, (struct sockaddr *)&remote, sizeof remote) != 0) {
        supplicant_close(link);
        return false;
    }

    if (is_events && !supplicant_ok(link, "ATTACH")) {
        supplicant_close(link);
        return false;
    }
    return true;
}

void supplicant_close(supplicant_link_t *link) {
    if (link->fd >= 0) {
        close(link->fd);
        unlink(link->local_path);
    }
    link->fd = -1;
}

bool supplicant_command(supplicant_link_t *link, const char *command, char *reply, size_t size) {

    if (link->fd < 0) return false;

    // risposte vecchie arrivate dopo un'attesa scaduta: si buttano
    char old[64];
    while (recv(link->fd, old, sizeof old, MSG_DONTWAIT) > 0) {}

    if (send(link->fd, command, strlen(command), 0) < 0) return false;

    int64_t end = now_ms() + COMMAND_TIMEOUT_MS;

    while (now_ms() < end) {
        struct pollfd p = { .fd = link->fd, .events = POLLIN };
        if (poll(&p, 1, (int)(end - now_ms())) <= 0) return false;

        ssize_t length = recv(link->fd, reply, size - 1, 0);
        if (length < 0) return false;
        reply[length] = '\0';

        // sul collegamento degli eventi possono arrivare eventi in mezzo: iniziano con "<"
        if (reply[0] != '<') return true;
    }
    return false;
}

bool supplicant_ok(supplicant_link_t *link, const char *command) {
    char reply[64];
    return supplicant_command(link, command, reply, sizeof reply) && strncmp(reply, "OK", 2) == 0;
}

bool supplicant_event(supplicant_link_t *link, char *event, size_t size, int64_t timeout_ms) {

    if (link->fd < 0) return false;

    struct pollfd p = { .fd = link->fd, .events = POLLIN };
    if (poll(&p, 1, (int)timeout_ms) <= 0) return false;

    ssize_t length = recv(link->fd, event, size - 1, MSG_DONTWAIT);
    if (length <= 0) return false;
    event[length] = '\0';
    return true;
}

bool supplicant_value(const char *reply, const char *key, char *value, size_t size) {

    size_t key_length = strlen(key);
    const char *line = reply;

    while (line != NULL && *line) {
        if (strncmp(line, key, key_length) == 0 && line[key_length] == '=') {
            const char *start = line + key_length + 1;
            size_t length = strcspn(start, "\n");
            if (length >= size) length = size - 1;
            memcpy(value, start, length);
            value[length] = '\0';
            return true;
        }
        line = strchr(line, '\n');
        if (line != NULL) line++;
    }
    return false;
}
