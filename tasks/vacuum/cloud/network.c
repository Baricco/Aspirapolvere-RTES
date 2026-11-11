#define _GNU_SOURCE   // per pthread_setaffinity_np
#include <ctype.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <curl/curl.h>

#include "third_party/cjson/cJSON.h"

#include "app_commands.h"
#include "config_check.h"
#include "live_map.h"
#include "network.h"
#include "robot_state.h"
#include "server_roots.h"
#include "../aperiodic.h"

#define HTTP_OK             200
#define NO_SERVER_MAP_VERSION (-1)          // il server non ha nessuna versione della mappa dal vivo
#define URL_MAX             512             // lunghezza massima dell'indirizzo di una richiesta
#define AUTH_HEADER_MAX     (TOKEN_MAX_LENGTH + 32)
#define RESPONSE_MAX_BYTES  MAP_DOWNLOAD_MAX_BYTES  // una risposta più grande di così si rifiuta

// una connessione al server: un handle di libcurl riusato, così la connessione resta aperta (niente handshake TLS a ogni richiesta)
struct http_connection {
    CURL *curl;
    struct curl_slist *json_headers;        // intestazioni delle richieste con il corpo JSON
    struct curl_slist *bytes_headers;       // intestazioni delle richieste con il corpo a byte (mappe)
};

// attese fra un tentativo e l'altro quando la rete cade
static const int RECONNECT_DELAYS[] = RECONNECT_DELAYS_S;
#define RECONNECT_DELAYS_NUM ((int)(sizeof RECONNECT_DELAYS / sizeof RECONNECT_DELAYS[0]))

// indirizzo del server, token e certificato: letti una volta all'avvio, poi solo letti dai due thread
static char server_url[URL_MAX];
static char auth_header[AUTH_HEADER_MAX];
static const char *server_ca_file;      // sul PC: il certificato autofirmato del server di prova. NULL = le radici del programma

static pthread_t commands_thread, state_thread;

// connessione del thread dei comandi: la usa anche per scaricare la mappa durante un map_select
static http_connection_t *commands_connection;

// true quando un cambio di mappa chiede il riavvio del programma
static atomic_bool restart_requested;

void request_restart(void) {
    atomic_store(&restart_requested, true);
}

bool is_restart_requested(void) {
    return atomic_load(&restart_requested);
}

bool append_bytes(void *context, const uint8_t *data, size_t size) {

    byte_buffer_t *buffer = context;

    if (buffer->size + size > buffer->capacity) {
        size_t capacity = buffer->capacity ? buffer->capacity * 2 : 1024;
        while (capacity < buffer->size + size) capacity *= 2;
        uint8_t *grown = realloc(buffer->data, capacity);
        if (grown == NULL) return false;
        buffer->data = grown;
        buffer->capacity = capacity;
    }

    memcpy(buffer->data + buffer->size, data, size);
    buffer->size += size;
    return true;
}

void free_bytes(byte_buffer_t *buffer) {
    free(buffer->data);
    buffer->data = NULL;
    buffer->size = buffer->capacity = 0;
}

// funzione che libcurl chiama con i pezzi del corpo della risposta
static size_t receive_response(char *data, size_t size, size_t count, void *context) {

    byte_buffer_t *response = context;
    size_t total = size * count;

    // senza un buffer la risposta si butta
    if (response == NULL) return total;

    // ritornare un numero diverso da total fa fallire la richiesta
    if (response->size + total > RESPONSE_MAX_BYTES || !append_bytes(response, (const uint8_t *)data, total)) return 0;

    return total;
}

// funzione helper che apre una connessione (senza ancora collegarsi: lo fa la prima richiesta)
static http_connection_t *open_connection(void) {

    http_connection_t *connection = calloc(1, sizeof *connection);
    if (connection == NULL) return NULL;

    connection->curl = curl_easy_init();
    if (connection->curl == NULL) {
        free(connection);
        return NULL;
    }

    // "Expect:" vuoto: il corpo parte subito, senza aspettare il permesso del server
    connection->json_headers = curl_slist_append(NULL, auth_header);
    connection->json_headers = curl_slist_append(connection->json_headers, "Content-Type: application/json");
    connection->json_headers = curl_slist_append(connection->json_headers, "Expect:");
    connection->bytes_headers = curl_slist_append(NULL, auth_header);
    connection->bytes_headers = curl_slist_append(connection->bytes_headers, "Content-Type: application/octet-stream");
    connection->bytes_headers = curl_slist_append(connection->bytes_headers, "Expect:");

    return connection;
}

long send_http_request(http_connection_t *connection, const char *method, const char *path, const uint8_t *body,
                       size_t body_size, bool is_json, byte_buffer_t *response, long timeout_s) {

    char url[URL_MAX];
    if (snprintf(url, sizeof url, "%s%s", server_url, path) >= (int)sizeof url) return HTTP_FAILED;

    CURL *curl = connection->curl;

    // le opzioni si rimettono tutte a ogni richiesta; la connessione aperta resta
    curl_easy_reset(curl);
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, method);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, is_json ? connection->json_headers : connection->bytes_headers);

    // solo HTTPS, anche dopo un reindirizzamento, e il certificato del server si verifica sempre
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

    // niente segnali (ci sono più thread), tempi massimi, connessione tenuta viva
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, (long)CONNECT_TIMEOUT_S);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_s);
    curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE, 1L);

    if (body != NULL || strcmp(method, "GET") != 0) {
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body != NULL ? (const char *)body : "");
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE_LARGE, (curl_off_t)body_size);
    }

    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, receive_response);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, response);

    if (curl_easy_perform(curl) != CURLE_OK) return HTTP_FAILED;

    long code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
    return code;
}

// funzione helper che mette il thread corrente sul core del task aperiodico, con nice più alto: il task aperiodico
// (anche lui con lo scheduling normale sullo stesso core) ha la precedenza quando cerca la posizione
static void setup_network_thread(void) {

    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(APERIODIC_CPU, &set);
    pthread_setaffinity_np(pthread_self(), sizeof set, &set);

    // su Linux il nice è di ogni thread: si imposta con l'id del thread
    setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), NETWORK_NICE);
}

// funzione helper che aspetta prima di riprovare: 1, 2, 5 e poi 10 s
static void wait_before_retry(int *failures) {
    int delay = RECONNECT_DELAYS[*failures < RECONNECT_DELAYS_NUM ? *failures : RECONNECT_DELAYS_NUM - 1];
    if (*failures == 0) fprintf(stderr, "rete: server non raggiungibile, riprovo\n");
    (*failures)++;
    rt_dormi_fino_a(rt_adesso() + RT_S(delay));
}

// funzione che l'interprete chiama per scaricare la mappa da selezionare nel file path
static bool download_map(int map_id, const char *path) {

    char request_path[128];
    byte_buffer_t response = {0};

    snprintf(request_path, sizeof request_path, PATH_MAP_DOWNLOAD, map_id);
    bool ok = send_http_request(commands_connection, "GET", request_path, NULL, 0, false, &response, REQUEST_TIMEOUT_S) == HTTP_OK;

    if (ok) {
        FILE *f = fopen(path, "wb");
        ok = f != NULL && fwrite(response.data, 1, response.size, f) == response.size;
        if (f != NULL && fclose(f) != 0) ok = false;
    }

    free_bytes(&response);
    return ok;
}

// funzione helper che esegue un comando arrivato dal server e ne manda l'esito
static void run_server_command(const cJSON *command) {

    const cJSON *id = cJSON_GetObjectItemCaseSensitive(command, "id");
    char *command_text = cJSON_PrintUnformatted(command);
    if (!cJSON_IsNumber(id) || command_text == NULL) {
        free(command_text);
        return;
    }

    app_command_result_t result;
    run_app_command(command_text, download_map, &result);
    fprintf(stderr, "rete: comando %s: %s\n", command_text, result.ok ? "eseguito" : result.reason);
    free(command_text);

    // esito: {"ok": ..., "reason": ..., "message": ...}
    cJSON *result_json = cJSON_CreateObject();
    cJSON_AddBoolToObject(result_json, "ok", result.ok);
    if (!result.ok) {
        cJSON_AddStringToObject(result_json, "reason", result.reason);
        cJSON_AddStringToObject(result_json, "message", result.message);
    }
    char *result_text = cJSON_PrintUnformatted(result_json);
    cJSON_Delete(result_json);

    if (result_text != NULL) {
        char request_path[128];
        snprintf(request_path, sizeof request_path, PATH_COMMAND_RESULT, (unsigned)id->valuedouble);
        send_http_request(commands_connection, "POST", request_path, (const uint8_t *)result_text, strlen(result_text), true, NULL, REQUEST_TIMEOUT_S);
        free(result_text);
    }

    // cambio di mappa: mandato l'esito, il programma ferma i task ed esce per essere rilanciato
    if (result.is_restart_needed) request_restart();
}

// corpo del thread dei comandi: ripete la richiesta lunga, e per ogni comando chiama l'interprete e manda l'esito
static void *commands_loop(void *arg) {

    (void)arg;

    setup_network_thread();

    int failures = 0;

    while (!is_restart_requested()) {

        byte_buffer_t response = {0};
        long code = send_http_request(commands_connection, "GET", PATH_COMMANDS, NULL, 0, false, &response, COMMANDS_REQUEST_TIMEOUT_S);

        // il terminatore serve a leggere la risposta come testo
        if (code != HTTP_OK || !append_bytes(&response, (const uint8_t *)"", 1)) {
            free_bytes(&response);
            wait_before_retry(&failures);
            continue;
        }

        if (failures > 0) fprintf(stderr, "rete: server raggiunto di nuovo\n");
        failures = 0;

        cJSON *body = cJSON_Parse((const char *)response.data);
        const cJSON *commands = cJSON_GetObjectItemCaseSensitive(body, "commands");
        const cJSON *command;
        cJSON_ArrayForEach(command, commands) {
            run_server_command(command);
            if (is_restart_requested()) break;
        }

        cJSON_Delete(body);
        free_bytes(&response);
    }

    return NULL;
}

// funzione helper che legge la risposta del server a uno stato: {"ok": true, "live_map_version": N} con la versione
// della mappa dal vivo che il server ha (null se non ne ha nessuna), per sapere se va rimandata intera
static void check_state_response(byte_buffer_t *response) {

    // il terminatore serve a leggere la risposta come testo
    if (!append_bytes(response, (const uint8_t *)"", 1)) return;

    cJSON *body = cJSON_Parse((const char *)response->data);
    const cJSON *version = cJSON_GetObjectItemCaseSensitive(body, "live_map_version");

    if (cJSON_IsNumber(version)) check_server_map_version((long)version->valuedouble);
    else if (cJSON_IsNull(version)) check_server_map_version(NO_SERVER_MAP_VERSION);

    cJSON_Delete(body);
}

// corpo del thread dello stato: un ciclo ogni STATE_CYCLE. Stato del robot, mappa dal vivo, mappa salvata
static void *state_loop(void *arg) {

    (void)arg;

    setup_network_thread();

    http_connection_t *connection = open_connection();
    if (connection == NULL) return NULL;

    // all'avvio: mappa salvata, poi mappa dal vivo intera (partono al primo giro di send_live_map)
    init_live_map();
    init_robot_state();

    int failures = 0;
    int64_t retry_t = 0;            // con la rete giù non si riprova prima di questo istante
    int64_t next_map_t = 0;         // prossimo invio della mappa dal vivo
    int64_t next_cycle_t = rt_adesso();

    while (!is_restart_requested()) {

        next_cycle_t += STATE_CYCLE;
        int64_t now = rt_adesso();
        if (next_cycle_t < now) next_cycle_t = now;     // una richiesta lenta ha fatto saltare dei cicli: non si recuperano
        rt_dormi_fino_a(next_cycle_t);
        now = rt_adesso();

        // la posizione si segue sempre, anche con la rete giù: le celle perse partono all'invio dopo
        track_live_map_position();

        char *state_json = build_robot_state_if_due();

        // rete giù: lo stato si butta (nessuna coda, conta solo il più nuovo) e si aspetta prima di riprovare
        if (now < retry_t) {
            free(state_json);
            continue;
        }

        bool is_network_ok = true;

        if (state_json != NULL) {
            byte_buffer_t response = {0};
            long code = send_http_request(connection, "POST", PATH_STATE, (const uint8_t *)state_json, strlen(state_json), true, &response, REQUEST_TIMEOUT_S);
            free(state_json);
            if (code == HTTP_OK) {
                mark_robot_state_sent();
                check_state_response(&response);
            }
            free_bytes(&response);
            is_network_ok = code != HTTP_FAILED;
        }

        if (is_network_ok && now >= next_map_t) {
            next_map_t = now + LIVE_MAP_SEND_INTERVAL;
            is_network_ok = send_live_map(connection);
        }

        if (is_network_ok) {
            failures = 0;
        } else {
            retry_t = rt_adesso() + RT_S(RECONNECT_DELAYS[failures < RECONNECT_DELAYS_NUM ? failures : RECONNECT_DELAYS_NUM - 1]);
            failures++;
        }
    }

    return NULL;
}

// funzione helper che legge il token dal suo file, senza spazi e a capo. Ritorna false se il file non c'è o è vuoto
static bool read_token(char *token, size_t size) {

    const char *path = getenv(TOKEN_FILE_ENV) ? getenv(TOKEN_FILE_ENV) : TOKEN_FILE;

    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "rete: non c'è il file del token %s: la rete non parte, il robot funziona senza app\n", path);
        return false;
    }

    size_t length = fread(token, 1, size - 1, f);
    fclose(f);

    while (length > 0 && isspace((unsigned char)token[length - 1])) length--;
    token[length] = '\0';

    if (length == 0) fprintf(stderr, "rete: il file del token %s è vuoto: la rete non parte\n", path);
    return length > 0;
}

// funzione helper che crea un thread di rete con lo stack piccolo e fisso
static bool create_network_thread(pthread_t *thread, void *(*loop)(void *)) {

    // scheduling normale: il thread viene creato dal main, che non è real-time
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, NETWORK_STACK);
    int r = pthread_create(thread, &attr, loop, NULL);
    pthread_attr_destroy(&attr);

    if (r == 0) pthread_detach(*thread);
    return r == 0;
}

int start_network(void) {

    char token[TOKEN_MAX_LENGTH + 1];
    if (!read_token(token, sizeof token)) return -1;
    snprintf(auth_header, sizeof auth_header, "Authorization: Bearer %s", token);

    const char *url = getenv(SERVER_URL_ENV) ? getenv(SERVER_URL_ENV) : SERVER_URL;
    snprintf(server_url, sizeof server_url, "%s", url);
    server_ca_file = getenv(SERVER_CA_ENV);

    // una connessione chiusa dal server non deve far arrivare SIGPIPE al programma
    signal(SIGPIPE, SIG_IGN);

    if (curl_global_init(CURL_GLOBAL_DEFAULT) != 0) {
        fprintf(stderr, "rete: libcurl non parte, niente rete\n");
        return -1;
    }

    commands_connection = open_connection();
    if (commands_connection == NULL || !create_network_thread(&commands_thread, commands_loop) ||
        !create_network_thread(&state_thread, state_loop)) {
        fprintf(stderr, "rete: non riesco a creare i thread, niente rete\n");
        return -1;
    }

    fprintf(stderr, "rete: collegamento a %s\n", server_url);
    return 0;
}
