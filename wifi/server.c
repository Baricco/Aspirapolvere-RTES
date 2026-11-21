#define _GNU_SOURCE   // per accept4 e strcasestr
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "third_party/cjson/cJSON.h"

#include "manager.h"
#include "server.h"

#define LISTEN_POLL_MS      500     // il thread che ascolta controlla ogni tanto se aprire o chiudere la porta HTTP
#define READ_TIMEOUT_MS     5000    // attesa massima della richiesta dopo la connessione
#define REQUEST_MAX         4096    // richiesta più lunga (intestazioni e corpo)
#define MAX_CONNECTIONS     8       // connessioni servite insieme; le altre si chiudono subito
#define HANDLER_STACK       (128 * 1024)

#define LOG(...) do { fprintf(stderr, "rtes-wifi: " __VA_ARGS__); fputc('\n', stderr); } while (0)

static char socket_path[108];
static char http_address[64];
static int http_port;
static atomic_int connections;

typedef struct {
    int fd;
    bool is_http;
} connection_t;

// --- JSON ---

// stato pubblicato (docs/wifi.md, sezione 7.4)
static cJSON *status_json(void) {

    wifi_status_t status;
    manager_get_status(&status);

    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "mode", wifi_mode_name(status.mode));
    if (status.link.ssid[0]) cJSON_AddStringToObject(json, "ssid", status.link.ssid);
    else cJSON_AddNullToObject(json, "ssid");
    if (status.link.signal_dbm != 0) cJSON_AddNumberToObject(json, "signal_dbm", status.link.signal_dbm);
    else cJSON_AddNullToObject(json, "signal_dbm");
    if (status.link.ip[0]) cJSON_AddStringToObject(json, "ip", status.link.ip);
    else cJSON_AddNullToObject(json, "ip");
    if (status.saved_ssid[0]) cJSON_AddStringToObject(json, "saved_ssid", status.saved_ssid);
    else cJSON_AddNullToObject(json, "saved_ssid");

    cJSON *ap = cJSON_AddObjectToObject(json, "ap");
    cJSON_AddBoolToObject(ap, "on", status.is_ap_on);
    cJSON_AddNumberToObject(ap, "clients", status.ap_clients);

    cJSON *change = cJSON_AddObjectToObject(json, "change");
    cJSON_AddNumberToObject(change, "number", status.change_number);
    if (status.change_number > 0) cJSON_AddStringToObject(change, "ssid", status.change_ssid);
    cJSON_AddStringToObject(change, "state", wifi_change_state_name(status.change_state));
    cJSON_AddStringToObject(change, "result", wifi_result_name(status.change_result));
    if (status.change_time > 0) cJSON_AddNumberToObject(change, "time", (double)status.change_time);

    return json;
}

// reti vicine
static cJSON *scan_json(const wifi_network_t *networks, const wifi_reply_t *reply) {

    cJSON *json = cJSON_CreateObject();
    if (reply->scanned_at > 0) cJSON_AddNumberToObject(json, "scanned_at", (double)reply->scanned_at);
    else cJSON_AddNullToObject(json, "scanned_at");

    cJSON *list = cJSON_AddArrayToObject(json, "networks");
    for (int i = 0; i < reply->network_count; i++) {
        cJSON *network = cJSON_CreateObject();
        cJSON_AddStringToObject(network, "ssid", networks[i].ssid);
        cJSON_AddNumberToObject(network, "signal_dbm", networks[i].signal_dbm);
        cJSON_AddStringToObject(network, "security", wifi_security_name(networks[i].security));
        cJSON_AddBoolToObject(network, "in_use", networks[i].is_in_use);
        cJSON_AddItemToArray(list, network);
    }
    return json;
}

// legge la richiesta di cambio dal JSON. Ritorna false (con la risposta già scritta) se è fatta male
static bool parse_connect(const cJSON *body, wifi_connect_request_t *request, wifi_reply_t *reply) {

    memset(request, 0, sizeof *request);
    const cJSON *ssid = cJSON_GetObjectItemCaseSensitive(body, "ssid");
    const cJSON *security = cJSON_GetObjectItemCaseSensitive(body, "security");
    const cJSON *password = cJSON_GetObjectItemCaseSensitive(body, "password");

    memset(reply, 0, sizeof *reply);
    reply->http_code = 400;

    if (!cJSON_IsString(ssid) || strlen(ssid->valuestring) > WIFI_SSID_MAX) {
        reply->reason = "invalid_ssid";
        snprintf(reply->message, sizeof reply->message, "Il nome della rete non è valido");
        return false;
    }
    if (!cJSON_IsString(security) ||
        (strcmp(security->valuestring, "open") != 0 && strcmp(security->valuestring, "wpa") != 0)) {
        reply->reason = "invalid_request";
        snprintf(reply->message, sizeof reply->message, "Tipo di rete sconosciuto");
        return false;
    }
    if (password != NULL && (!cJSON_IsString(password) || strlen(password->valuestring) > WIFI_PASSWORD_MAX)) {
        reply->reason = "invalid_password";
        snprintf(reply->message, sizeof reply->message, "La password deve avere da 8 a 63 caratteri");
        return false;
    }

    snprintf(request->ssid, sizeof request->ssid, "%s", ssid->valuestring);
    request->security = strcmp(security->valuestring, "wpa") == 0 ? WIFI_SECURITY_WPA : WIFI_SECURITY_OPEN;
    if (password != NULL) snprintf(request->password, sizeof request->password, "%s", password->valuestring);
    return true;
}

// esegue una richiesta (type: "status", "scan", "connect", "resting") e ritorna i dati della risposta, oppure NULL
// con reply che dice perché
static cJSON *run_request(const char *type, const cJSON *body, wifi_reply_t *reply) {

    memset(reply, 0, sizeof *reply);
    reply->ok = true;
    reply->http_code = 200;

    if (strcmp(type, "status") == 0) return status_json();

    if (strcmp(type, "scan") == 0) {
        static __thread wifi_network_t networks[WIFI_SCAN_MAX_NETWORKS];
        manager_scan(networks, reply);
        return reply->ok ? scan_json(networks, reply) : NULL;
    }

    if (strcmp(type, "connect") == 0) {
        wifi_connect_request_t request;
        if (!parse_connect(body, &request, reply)) return NULL;
        manager_connect(&request, reply);
        memset(&request, 0, sizeof request);
        if (!reply->ok) return NULL;
        cJSON *json = cJSON_CreateObject();
        cJSON_AddNumberToObject(json, "change_number", reply->change_number);
        return json;
    }

    if (strcmp(type, "resting") == 0) {
        const cJSON *value = cJSON_GetObjectItemCaseSensitive(body, "value");
        if (cJSON_IsBool(value)) {
            manager_set_resting(cJSON_IsTrue(value));
            return cJSON_CreateObject();
        }
    }

    reply->ok = false;
    reply->http_code = 400;
    reply->reason = "invalid_request";
    snprintf(reply->message, sizeof reply->message, "Richiesta sconosciuta");
    return NULL;
}

// --- Lettura e scrittura ---

// legge dal socket finché in buffer non c'è end (o finché il buffer è pieno). Ritorna i byte letti, -1 se scade
static int read_until(int fd, char *buffer, size_t size, const char *end) {

    size_t length = 0;
    int64_t deadline = now_ms() + READ_TIMEOUT_MS;

    while (length + 1 < size) {
        struct pollfd p = { .fd = fd, .events = POLLIN };
        int64_t left = deadline - now_ms();
        if (left <= 0 || poll(&p, 1, (int)left) <= 0) return -1;
        ssize_t n = recv(fd, buffer + length, size - 1 - length, 0);
        if (n <= 0) break;
        length += (size_t)n;
        buffer[length] = '\0';
        if (strstr(buffer, end) != NULL) break;
    }
    buffer[length] = '\0';
    return (int)length;
}

static void send_all(int fd, const char *data, size_t size) {
    while (size > 0) {
        ssize_t n = send(fd, data, size, MSG_NOSIGNAL);
        if (n <= 0) return;
        data += n;
        size -= (size_t)n;
    }
}

// --- Socket per il programma dei task ---

// una riga JSON: {"type": ..., parametri}. Risposta: {"ok": true, "data": {...}} oppure {"ok": false, "reason", "message"}
static void serve_local(int fd) {

    char request[REQUEST_MAX];
    if (read_until(fd, request, sizeof request, "\n") <= 0) return;

    cJSON *body = cJSON_Parse(request);
    const cJSON *type = cJSON_GetObjectItemCaseSensitive(body, "type");

    wifi_reply_t reply;
    cJSON *data = cJSON_IsString(type) ? run_request(type->valuestring, body, &reply) : NULL;
    if (!cJSON_IsString(type)) {
        memset(&reply, 0, sizeof reply);
        reply.reason = "invalid_request";
        snprintf(reply.message, sizeof reply.message, "Richiesta sconosciuta");
    }

    // la password non resta nella memoria del processo
    cJSON *password = cJSON_GetObjectItemCaseSensitive(body, "password");
    if (cJSON_IsString(password)) memset(password->valuestring, 0, strlen(password->valuestring));
    memset(request, 0, sizeof request);
    cJSON_Delete(body);

    cJSON *answer = cJSON_CreateObject();
    cJSON_AddBoolToObject(answer, "ok", data != NULL);
    if (data != NULL) {
        cJSON_AddItemToObject(answer, "data", data);
    } else {
        cJSON_AddStringToObject(answer, "reason", reply.reason ? reply.reason : "failed");
        cJSON_AddStringToObject(answer, "message", reply.message);
    }

    char *text = cJSON_PrintUnformatted(answer);
    cJSON_Delete(answer);
    if (text != NULL) {
        send_all(fd, text, strlen(text));
        send_all(fd, "\n", 1);
        free(text);
    }
}

// --- HTTP sulla rete del robot ---

static void send_http(int fd, int code, cJSON *json) {

    const char *status = code == 200 ? "OK" : code == 400 ? "Bad Request" : code == 404 ? "Not Found" :
                         code == 409 ? "Conflict" : "Service Unavailable";
    char *body = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    if (body == NULL) return;

    char header[256];
    int length = snprintf(header, sizeof header,
                          "HTTP/1.1 %d %s\r\nContent-Type: application/json\r\nContent-Length: %zu\r\n"
                          "Cache-Control: no-store\r\nConnection: close\r\n\r\n", code, status, strlen(body));
    send_all(fd, header, (size_t)length);
    send_all(fd, body, strlen(body));
    free(body);
}

// errore nella forma del backend: {"reason", "message"}
static cJSON *error_json(const char *reason, const char *message) {
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "reason", reason);
    cJSON_AddStringToObject(json, "message", message);
    return json;
}

static void serve_http(int fd) {

    char request[REQUEST_MAX];
    int length = read_until(fd, request, sizeof request, "\r\n\r\n");
    if (length <= 0) return;

    char method[8] = "", path[128] = "";
    if (sscanf(request, "%7s %127s", method, path) != 2) {
        send_http(fd, 400, error_json("invalid_request", "Richiesta non valida"));
        return;
    }

    // corpo: quello che manca rispetto a Content-Length
    char *headers_end = strstr(request, "\r\n\r\n");
    char *body_text = headers_end != NULL ? headers_end + 4 : request + length;
    size_t body_length = (size_t)(request + length - body_text);
    const char *content_length = strcasestr(request, "\r\nContent-Length:");
    size_t expected = content_length != NULL ? (size_t)strtoul(content_length + 17, NULL, 10) : 0;
    if (body_text + expected >= request + sizeof request) {
        send_http(fd, 400, error_json("invalid_request", "Richiesta troppo lunga"));
        return;
    }
    while (body_length < expected) {
        int n = read_until(fd, body_text + body_length, (size_t)(request + sizeof request - body_text) - body_length, "\0");
        if (n <= 0) break;
        body_length += (size_t)n;
    }
    body_text[body_length] = '\0';

    const char *type = NULL;
    if (strcmp(method, "GET") == 0 && strcmp(path, "/api/wifi/status") == 0) {
        type = "status";
    } else if (strcmp(method, "GET") == 0 && strncmp(path, "/api/wifi/networks", 18) == 0 &&
               (path[18] == '\0' || path[18] == '?')) {
        type = "scan";
    } else if (strcmp(method, "POST") == 0 && strcmp(path, "/api/wifi/connect") == 0) {
        type = "connect";
    }

    if (type == NULL) {
        send_http(fd, 404, error_json("not_found", "Richiesta sconosciuta"));
        return;
    }

    cJSON *body = cJSON_Parse(body_text);
    wifi_reply_t reply;
    cJSON *data = run_request(type, body, &reply);

    cJSON *password = cJSON_GetObjectItemCaseSensitive(body, "password");
    if (cJSON_IsString(password)) memset(password->valuestring, 0, strlen(password->valuestring));
    memset(request, 0, sizeof request);
    cJSON_Delete(body);

    if (data != NULL) send_http(fd, 200, data);
    else send_http(fd, reply.http_code, error_json(reply.reason ? reply.reason : "failed", reply.message));
}

// --- Thread ---

static void *serve_connection(void *arg) {

    connection_t connection = *(connection_t *)arg;
    free(arg);

    if (connection.is_http) serve_http(connection.fd);
    else serve_local(connection.fd);

    close(connection.fd);
    atomic_fetch_sub(&connections, 1);
    return NULL;
}

// accetta una connessione e la passa a un thread suo
static void accept_connection(int listener, bool is_http) {

    int fd = accept4(listener, NULL, NULL, SOCK_CLOEXEC);
    if (fd < 0) return;

    connection_t *connection = malloc(sizeof *connection);
    if (connection == NULL || atomic_fetch_add(&connections, 1) >= MAX_CONNECTIONS) {
        if (connection != NULL) atomic_fetch_sub(&connections, 1);
        free(connection);
        close(fd);
        return;
    }
    connection->fd = fd;
    connection->is_http = is_http;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, HANDLER_STACK);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_t thread;
    if (pthread_create(&thread, &attr, serve_connection, connection) != 0) {
        close(fd);
        free(connection);
        atomic_fetch_sub(&connections, 1);
    }
    pthread_attr_destroy(&attr);
}

static int open_local_listener(void) {

    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;

    struct sockaddr_un address = { .sun_family = AF_UNIX };
    snprintf(address.sun_path, sizeof address.sun_path, "%s", socket_path);
    unlink(socket_path);

    if (bind(fd, (struct sockaddr *)&address, sizeof address) != 0 || listen(fd, MAX_CONNECTIONS) != 0) {
        LOG("non riesco ad aprire il socket %s: %s", socket_path, strerror(errno));
        close(fd);
        return -1;
    }
    chmod(socket_path, 0600);
    return fd;
}

static int open_http_listener(void) {

    int fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;

    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);

    struct sockaddr_in address = { .sin_family = AF_INET, .sin_port = htons((uint16_t)http_port) };
    inet_pton(AF_INET, http_address, &address.sin_addr);

    // con la rete del robot appena accesa l'indirizzo potrebbe non esserci ancora: si riprova al giro dopo
    if (bind(fd, (struct sockaddr *)&address, sizeof address) != 0 || listen(fd, MAX_CONNECTIONS) != 0) {
        close(fd);
        return -1;
    }
    LOG("server HTTP su %s:%d", http_address, http_port);
    return fd;
}

static void *listen_loop(void *arg) {

    int local = (int)(intptr_t)arg;
    int http = -1;

    while (true) {

        // la porta HTTP è aperta solo con la rete del robot accesa
        bool is_ap = manager_is_ap();
        if (is_ap && http < 0) {
            http = open_http_listener();
        } else if (!is_ap && http >= 0) {
            close(http);
            http = -1;
            LOG("server HTTP chiuso");
        }

        struct pollfd p[2] = { { .fd = local, .events = POLLIN }, { .fd = http, .events = POLLIN } };
        if (poll(p, http >= 0 ? 2 : 1, LISTEN_POLL_MS) <= 0) continue;

        if (p[0].revents & POLLIN) accept_connection(local, false);
        if (http >= 0 && (p[1].revents & POLLIN)) accept_connection(http, true);
    }
    return NULL;
}

bool start_server(const char *path, const char *address, int port) {

    snprintf(socket_path, sizeof socket_path, "%s", path);
    snprintf(http_address, sizeof http_address, "%s", address);
    http_port = port;

    int local = open_local_listener();
    if (local < 0) return false;

    pthread_t thread;
    if (pthread_create(&thread, NULL, listen_loop, (void *)(intptr_t)local) != 0) return false;
    pthread_detach(thread);
    return true;
}
