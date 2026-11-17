#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip_icmp.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <curl/curl.h>

#include "check.h"
#include "wifi.h"
#include "tasks/vacuum/cloud/server_roots.h"

#define URL_MAX             512
#define AUTH_HEADER_MAX     (TOKEN_MAX_LENGTH + 32)
#define PING_PAYLOAD        32      // byte di dati del ping
#define PING_REPLY_MAX      1024

static char server_url[URL_MAX];
static char auth_header[AUTH_HEADER_MAX];
static const char *server_ca_file;
static bool has_token;

// funzione helper che legge il token dal suo file, senza spazi e a capo (come tasks/vacuum/cloud/network.c)
static bool read_token(char *token, size_t size) {

    const char *path = getenv(TOKEN_FILE_ENV) ? getenv(TOKEN_FILE_ENV) : TOKEN_FILE;

    FILE *f = fopen(path, "r");
    if (f == NULL) return false;

    size_t length = fread(token, 1, size - 1, f);
    fclose(f);

    while (length > 0 && isspace((unsigned char)token[length - 1])) length--;
    token[length] = '\0';
    return length > 0;
}

bool init_backend_check(void) {

    const char *url = getenv(SERVER_URL_ENV) ? getenv(SERVER_URL_ENV) : SERVER_URL;
    snprintf(server_url, sizeof server_url, "%s", url);
    server_ca_file = getenv(SERVER_CA_ENV);

    char token[TOKEN_MAX_LENGTH + 1];
    has_token = read_token(token, sizeof token);
    if (has_token) snprintf(auth_header, sizeof auth_header, "Authorization: Bearer %s", token);
    memset(token, 0, sizeof token);

    if (curl_global_init(CURL_GLOBAL_DEFAULT) != 0) return false;
    return has_token;
}

// funzione che libcurl chiama con il corpo della risposta: si butta
static size_t discard_response(char *data, size_t size, size_t count, void *context) {
    (void)data;
    (void)context;
    return size * count;
}

bool check_backend(void) {

    if (!has_token) return false;

    char url[URL_MAX];
    if (snprintf(url, sizeof url, "%s%s", server_url, PATH_DEVICE_PING) >= (int)sizeof url) return false;

    CURL *curl = curl_easy_init();
    if (curl == NULL) return false;

    struct curl_slist *headers = curl_slist_append(NULL, auth_header);

    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    // solo HTTPS e certificato del server sempre verificato, come il programma dei task
    curl_easy_setopt(curl, CURLOPT_PROTOCOLS_STR, "https");
    curl_easy_setopt(curl, CURLOPT_REDIR_PROTOCOLS_STR, "https");
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);
    if (server_ca_file != NULL) {
        curl_easy_setopt(curl, CURLOPT_CAINFO, server_ca_file);
    } else {
        struct curl_blob roots = { .data = (void *)SERVER_ROOTS_PEM, .len = sizeof SERVER_ROOTS_PEM - 1, .flags = CURL_BLOB_NOCOPY };
        curl_easy_setopt(curl, CURLOPT_CAINFO_BLOB, &roots);
    }

    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, (long)WIFI_BACKEND_TIMEOUT_S);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, (long)WIFI_BACKEND_TIMEOUT_S);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, discard_response);

    CURLcode result = curl_easy_perform(curl);
    long code = 0;
    if (result == CURLE_OK) curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    if (result != CURLE_OK) {
        fprintf(stderr, "rtes-wifi: backend: %s\n", curl_easy_strerror(result));
    } else if (code != 200) {
        fprintf(stderr, "rtes-wifi: backend: risposta %ld\n", code);
    }
    return result == CURLE_OK && code == 200;
}

// funzione helper che calcola il checksum di un pacchetto ICMP
static uint16_t icmp_checksum(const void *data, size_t size) {

    const uint8_t *bytes = data;
    uint32_t sum = 0;

    for (size_t i = 0; i + 1 < size; i += 2) sum += (uint32_t)(bytes[i] << 8 | bytes[i + 1]);
    if (size % 2) sum += (uint32_t)(bytes[size - 1] << 8);
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);

    return htons((uint16_t)~sum);
}

bool ping_internet(void) {

    int fd = socket(AF_INET, SOCK_RAW | SOCK_CLOEXEC, IPPROTO_ICMP);
    if (fd < 0) {
        fprintf(stderr, "rtes-wifi: ping: socket ICMP non disponibile (%s)\n", strerror(errno));
        return false;
    }

    struct sockaddr_in to = { .sin_family = AF_INET };
    inet_pton(AF_INET, WIFI_PING_ADDRESS, &to.sin_addr);

    static uint16_t sequence;
    uint16_t id = (uint16_t)getpid();
    sequence++;

    uint8_t packet[sizeof(struct icmphdr) + PING_PAYLOAD];
    memset(packet, 0, sizeof packet);
    struct icmphdr *header = (struct icmphdr *)packet;
    header->type = ICMP_ECHO;
    header->un.echo.id = htons(id);
    header->un.echo.sequence = htons(sequence);
    header->checksum = icmp_checksum(packet, sizeof packet);

    bool ok = false;

    if (sendto(fd, packet, sizeof packet, 0, (struct sockaddr *)&to, sizeof to) == (ssize_t)sizeof packet) {

        int64_t end = now_ms() + WIFI_PING_TIMEOUT_S * 1000;

        while (!ok && now_ms() < end) {
            struct pollfd p = { .fd = fd, .events = POLLIN };
            if (poll(&p, 1, (int)(end - now_ms())) <= 0) break;

            // il socket raw riceve tutti i pacchetti ICMP: si cerca la risposta a questo ping
            uint8_t reply[PING_REPLY_MAX];
            ssize_t size = recv(fd, reply, sizeof reply, 0);
            if (size < (ssize_t)sizeof(struct iphdr)) continue;
            size_t ip_length = (size_t)(((const struct iphdr *)reply)->ihl * 4);
            if ((size_t)size < ip_length + sizeof(struct icmphdr)) continue;

            const struct icmphdr *answer = (const struct icmphdr *)(reply + ip_length);
            ok = answer->type == ICMP_ECHOREPLY && ntohs(answer->un.echo.id) == id &&
                 ntohs(answer->un.echo.sequence) == sequence;
        }
    }

    close(fd);
    return ok;
}
