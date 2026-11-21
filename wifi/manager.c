#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "manager.h"

#define TICK_MS             500     // giro del gestore senza richieste: controllo del collegamento e dei tempi
#define ADDRESS_POLL_MS     250     // durante un cambio, attesa fra due controlli dell'indirizzo

typedef enum {
    REQUEST_NONE,
    REQUEST_SCAN,
    REQUEST_CONNECT,
} request_type_t;

// tutto quello che il gestore condivide con gli altri thread, protetto da lock
static struct {
    pthread_mutex_t lock;
    pthread_cond_t request_cond;            // il gestore aspetta una richiesta
    pthread_cond_t reply_cond;              // chi ha fatto la richiesta aspetta la risposta

    wifi_status_t status;

    // ultima scansione
    wifi_network_t networks[WIFI_SCAN_MAX_NETWORKS];
    int network_count;
    int64_t scanned_at;                     // secondi dal 1970, 0 = mai

    // riposo detto dal programma dei task
    bool is_resting;
    int64_t resting_ms;                     // ultimo aggiornamento, 0 = mai

    // richiesta in corso: una alla volta
    request_type_t request;
    wifi_connect_request_t connect;
    unsigned request_num;                   // numero dell'ultima richiesta
    unsigned reply_num;                     // numero della richiesta a cui si riferisce reply
    wifi_reply_t reply;
} shared = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .request_cond = PTHREAD_COND_INITIALIZER,
    .reply_cond = PTHREAD_COND_INITIALIZER,
};

static atomic_bool is_stop_requested;

// dati del solo thread del gestore
static const radio_t *radio;
static int64_t lost_since_ms;               // da quando wlan0 è senza indirizzo (0 = c'è)
static int64_t ap_idle_since_ms;            // da quando la rete del robot è senza telefoni (0 = ce ne sono)
static int64_t backend_check_ms;            // prossima prova del backend per spegnere la rete del robot

// funzione helper che scrive un messaggio nel registro (stderr: procd lo manda a logread)
#define LOG(...) do { fprintf(stderr, "rtes-wifi: " __VA_ARGS__); fputc('\n', stderr); } while (0)

// --- Funzioni comuni ---

double time_scale = 1.0;

int64_t now_ms(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

int64_t epoch_s(void) {
    return (int64_t)time(NULL);
}

int64_t scaled_ms(double seconds) {
    return (int64_t)(seconds * 1000.0 * time_scale);
}

const char *wifi_mode_name(wifi_mode_t mode) {
    switch (mode) {
        case WIFI_MODE_CONNECTING:  return "connecting";
        case WIFI_MODE_STATION:     return "station";
        case WIFI_MODE_TESTING:     return "testing";
    }
    return "unknown";
}

const char *wifi_security_name(wifi_security_t security) {
    return security == WIFI_SECURITY_OPEN ? "open" : "wpa";
}

const char *wifi_change_state_name(wifi_change_state_t state) {
    switch (state) {
        case WIFI_CHANGE_NONE:      return "none";
        case WIFI_CHANGE_TESTING:   return "testing";
        case WIFI_CHANGE_DONE:      return "done";
    }
    return "unknown";
}

const char *wifi_result_name(wifi_result_t result) {
    switch (result) {
        case WIFI_RESULT_NONE:                  return "none";
        case WIFI_RESULT_OK:                    return "ok";
        case WIFI_RESULT_BACKEND_UNREACHABLE:   return "backend_unreachable";
        case WIFI_RESULT_INTERNET_UNREACHABLE:  return "internet_unreachable";
        case WIFI_RESULT_NO_ADDRESS:            return "no_address";
        case WIFI_RESULT_WRONG_PASSWORD:        return "wrong_password";
        case WIFI_RESULT_NOT_FOUND:             return "not_found";
    }
    return "unknown";
}

bool is_valid_utf8(const char *text) {

    const unsigned char *s = (const unsigned char *)text;

    while (*s) {
        int extra;
        if (*s < 0x80) extra = 0;
        else if ((*s & 0xE0) == 0xC0 && *s >= 0xC2) extra = 1;
        else if ((*s & 0xF0) == 0xE0) extra = 2;
        else if ((*s & 0xF8) == 0xF0 && *s <= 0xF4) extra = 3;
        else return false;
        s++;
        for (int i = 0; i < extra; i++, s++) {
            if ((*s & 0xC0) != 0x80) return false;
        }
    }
    return true;
}

// --- Stato pubblicato ---

// funzione helper che cambia il modo pubblicato
static void set_mode(wifi_mode_t mode) {
    pthread_mutex_lock(&shared.lock);
    shared.status.mode = mode;
    pthread_mutex_unlock(&shared.lock);
}

static wifi_mode_t get_mode(void) {
    pthread_mutex_lock(&shared.lock);
    wifi_mode_t mode = shared.status.mode;
    pthread_mutex_unlock(&shared.lock);
    return mode;
}

static bool is_ap_on(void) {
    pthread_mutex_lock(&shared.lock);
    bool is_on = shared.status.is_ap_on;
    pthread_mutex_unlock(&shared.lock);
    return is_on;
}

// funzione helper che legge collegamento e telefoni collegati dalla radio e li pubblica. Ritorna true se c'è un
// indirizzo
static bool update_link(void) {

    wifi_link_t link;
    memset(&link, 0, sizeof link);
    bool has_address = radio->get_link(&link);
    int clients = radio->get_ap_clients();

    pthread_mutex_lock(&shared.lock);
    shared.status.link = link;
    shared.status.ap_clients = clients;
    pthread_mutex_unlock(&shared.lock);

    return has_address;
}

// funzione helper che legge la rete salvata e la pubblica. Ritorna true se c'è
static bool update_saved_ssid(void) {

    char ssid[WIFI_SSID_MAX + 1];
    bool is_saved = radio->get_saved_ssid(ssid, sizeof ssid);

    pthread_mutex_lock(&shared.lock);
    snprintf(shared.status.saved_ssid, sizeof shared.status.saved_ssid, "%s", ssid);
    pthread_mutex_unlock(&shared.lock);

    return is_saved;
}

// funzione helper che ordina le reti per segnale e toglie i doppioni (stesso SSID da più punti di accesso: resta il
// più forte)
static int sort_networks(wifi_network_t *list, int count) {

    for (int i = 1; i < count; i++) {
        wifi_network_t n = list[i];
        int j = i - 1;
        while (j >= 0 && list[j].signal_dbm < n.signal_dbm) {
            list[j + 1] = list[j];
            j--;
        }
        list[j + 1] = n;
    }

    int kept = 0;
    for (int i = 0; i < count; i++) {
        bool is_duplicate = false;
        for (int j = 0; j < kept; j++) {
            if (strcmp(list[j].ssid, list[i].ssid) == 0) is_duplicate = true;
        }
        if (!is_duplicate) list[kept++] = list[i];
    }
    return kept;
}

// funzione helper che fa una scansione e, se riesce, la salva come ultima. Ritorna false se non riesce
static bool scan_and_store(int64_t timeout_ms) {

    wifi_network_t list[WIFI_SCAN_MAX_NETWORKS];
    int count = radio->scan(list, WIFI_SCAN_MAX_NETWORKS, timeout_ms);
    if (count < 0) {
        LOG("scansione non riuscita");
        return false;
    }
    count = sort_networks(list, count);

    pthread_mutex_lock(&shared.lock);
    memcpy(shared.networks, list, sizeof(list[0]) * (size_t)count);
    shared.network_count = count;
    shared.scanned_at = epoch_s();
    pthread_mutex_unlock(&shared.lock);

    LOG("scansione: %d reti", count);
    return true;
}

// --- Regole (docs/wifi.md, sezione 4) ---

// funzione helper che accende la rete del robot
static void turn_ap_on(void) {

    if (!radio->start_ap()) LOG("la rete del robot non si accende: riprovo da sola");

    pthread_mutex_lock(&shared.lock);
    shared.status.is_ap_on = true;
    pthread_mutex_unlock(&shared.lock);

    ap_idle_since_ms = now_ms();
    backend_check_ms = 0;
    LOG("rete del robot accesa (%s)", WIFI_AP_SSID);
}

// funzione helper che spegne la rete del robot
static void turn_ap_off(void) {

    pthread_mutex_lock(&shared.lock);
    shared.status.is_ap_on = false;
    shared.status.ap_clients = 0;
    pthread_mutex_unlock(&shared.lock);

    radio->stop_ap();
    LOG("rete del robot spenta");
}

// funzione helper che applica le regole a ogni giro: modo di wlan0, accensione e spegnimento della rete del robot
static void apply_rules(bool has_address, bool is_saved) {

    int64_t now = now_ms();
    wifi_mode_t mode = get_mode();

    if (mode != WIFI_MODE_TESTING) {
        if (has_address && mode != WIFI_MODE_STATION) LOG("collegato alla rete salvata");
        if (!has_address && mode == WIFI_MODE_STATION) LOG("indirizzo perso");
        set_mode(has_address ? WIFI_MODE_STATION : WIFI_MODE_CONNECTING);
    }

    if (has_address) lost_since_ms = 0;
    else if (lost_since_ms == 0) lost_since_ms = now;

    if (!is_ap_on()) {
        // accesa: wlan0 senza indirizzo da WIFI_LOST_TIMEOUT_S, oppure nessuna rete salvata
        if (!is_saved || (lost_since_ms != 0 && now - lost_since_ms >= scaled_ms(WIFI_LOST_TIMEOUT_S))) {
            if (is_saved) LOG("senza indirizzo da %d s", WIFI_LOST_TIMEOUT_S);
            else LOG("nessuna rete salvata");
            turn_ap_on();
        }
        return;
    }

    pthread_mutex_lock(&shared.lock);
    int clients = shared.status.ap_clients;
    pthread_mutex_unlock(&shared.lock);

    if (clients > 0) ap_idle_since_ms = 0;
    else if (ap_idle_since_ms == 0) ap_idle_since_ms = now;

    // spenta: nessun telefono da WIFI_AP_IDLE_S E wlan0 collegato E backend raggiunto
    bool is_idle = ap_idle_since_ms != 0 && now - ap_idle_since_ms >= scaled_ms(WIFI_AP_IDLE_S);
    if (!is_idle || mode != WIFI_MODE_STATION || now < backend_check_ms) return;

    if (radio->check_backend()) {
        LOG("nessun telefono collegato e backend raggiunto");
        turn_ap_off();
    } else {
        backend_check_ms = now_ms() + scaled_ms(WIFI_BACKEND_RECHECK_S);
    }
}

// --- Cambio di rete ---

// funzione helper che aspetta un indirizzo per al massimo timeout_ms. Ritorna true se arriva
static bool wait_address(int64_t timeout_ms) {

    int64_t end = now_ms() + timeout_ms;

    while (now_ms() < end) {
        radio->poll();
        if (update_link()) return true;
        struct timespec pause = { 0, ADDRESS_POLL_MS * 1000000L };
        nanosleep(&pause, NULL);
    }
    return false;
}

// funzione helper che prova la rete appena collegata: backend, poi internet
static wifi_result_t check_connection(void) {

    for (int i = 0; i < WIFI_BACKEND_TRIES; i++) {
        if (radio->check_backend()) return WIFI_RESULT_OK;
    }
    for (int i = 0; i < WIFI_PING_TRIES; i++) {
        if (radio->ping_internet()) return WIFI_RESULT_BACKEND_UNREACHABLE;
    }
    return WIFI_RESULT_INTERNET_UNREACHABLE;
}

// funzione helper che fa il cambio di rete (docs/wifi.md, sezione 5). Il cambio è già pubblicato come in corso
static void run_change(const wifi_connect_request_t *request) {

    LOG("cambio di rete verso «%s»", request->ssid);

    wifi_result_t result;

    if (!radio->try_network_begin(request)) {
        LOG("wpa_supplicant non accetta la rete nuova");
        result = WIFI_RESULT_NOT_FOUND;
    } else {
        switch (radio->try_network_wait(scaled_ms(WIFI_ASSOC_TIMEOUT_S))) {
            case WIFI_ASSOC_WRONG_KEY:
                result = WIFI_RESULT_WRONG_PASSWORD;
                break;
            case WIFI_ASSOC_TIMEOUT:
                result = WIFI_RESULT_NOT_FOUND;
                break;
            case WIFI_ASSOC_OK:
            default:
                result = wait_address(scaled_ms(WIFI_DHCP_TIMEOUT_S)) ? check_connection() : WIFI_RESULT_NO_ADDRESS;
                break;
        }
    }

    bool is_kept = result == WIFI_RESULT_OK || result == WIFI_RESULT_BACKEND_UNREACHABLE;

    if (is_kept) radio->try_network_commit();
    else radio->try_network_abort();

    set_mode(is_kept ? WIFI_MODE_STATION : WIFI_MODE_CONNECTING);
    lost_since_ms = 0;
    update_link();
    update_saved_ssid();

    pthread_mutex_lock(&shared.lock);
    shared.status.change_state = WIFI_CHANGE_DONE;
    shared.status.change_result = result;
    shared.status.change_time = epoch_s();
    pthread_mutex_unlock(&shared.lock);

    LOG("cambio verso «%s»: %s, %s", request->ssid, wifi_result_name(result),
        is_kept ? "rete nuova salvata" : "torno alla rete di prima");
}

// --- Richieste ---

// funzione helper che manda la risposta alla richiesta numero request_num. Si chiama con lock preso
static void send_reply(unsigned request_num, const wifi_reply_t *reply) {
    shared.reply = *reply;
    shared.reply_num = request_num;
    shared.request = REQUEST_NONE;
    pthread_cond_broadcast(&shared.reply_cond);
}

// funzione helper che esegue la scansione chiesta
static void handle_scan(unsigned request_num) {

    bool ok = scan_and_store(scaled_ms(WIFI_SCAN_TIMEOUT_S));

    wifi_reply_t reply = { .ok = ok, .http_code = ok ? 200 : 503 };
    if (!ok) {
        reply.reason = "scan_failed";
        snprintf(reply.message, sizeof reply.message, "Maurice non riesce a cercare le reti Wi-Fi");
    }

    pthread_mutex_lock(&shared.lock);
    reply.network_count = shared.network_count;
    reply.scanned_at = shared.scanned_at;
    send_reply(request_num, &reply);
    pthread_mutex_unlock(&shared.lock);
}

void manager_run(const radio_t *selected_radio) {

    radio = selected_radio;

    if (!radio->init()) LOG("preparazione della radio non riuscita: vado avanti lo stesso");

    radio->start_station();
    set_mode(WIFI_MODE_CONNECTING);
    bool is_saved = update_saved_ssid();

    while (!atomic_load(&is_stop_requested)) {

        // aspetta una richiesta, al massimo un giro
        pthread_mutex_lock(&shared.lock);
        if (shared.request == REQUEST_NONE) {
            struct timespec until;
            clock_gettime(CLOCK_REALTIME, &until);
            until.tv_nsec += TICK_MS * 1000000L;
            until.tv_sec += until.tv_nsec / 1000000000L;
            until.tv_nsec %= 1000000000L;
            pthread_cond_timedwait(&shared.request_cond, &shared.lock, &until);
        }
        request_type_t request = shared.request;
        unsigned request_num = shared.request_num;
        wifi_connect_request_t connect = shared.connect;
        pthread_mutex_unlock(&shared.lock);

        radio->poll();
        bool has_address = update_link();

        if (request == REQUEST_SCAN) {
            handle_scan(request_num);
        } else if (request == REQUEST_CONNECT) {
            // il cambio è accettato: risposta subito, poi il cambio (l'esito arriva nello stato)
            pthread_mutex_lock(&shared.lock);
            shared.status.change_number++;
            snprintf(shared.status.change_ssid, sizeof shared.status.change_ssid, "%s", connect.ssid);
            shared.status.change_state = WIFI_CHANGE_TESTING;
            shared.status.change_result = WIFI_RESULT_NONE;
            shared.status.change_time = 0;
            shared.status.mode = WIFI_MODE_TESTING;
            wifi_reply_t reply = { .ok = true, .http_code = 200, .change_number = shared.status.change_number };
            send_reply(request_num, &reply);
            pthread_mutex_unlock(&shared.lock);

            run_change(&connect);
            memset(&connect, 0, sizeof connect);    // la password non resta in memoria
            is_saved = update_saved_ssid();
        } else {
            apply_rules(has_address, is_saved);
        }
    }

    if (is_ap_on()) radio->stop_ap();
    radio->stop_station();
    LOG("fermo");
}

void manager_stop(void) {
    // solo una variabile atomica: si può chiamare da un gestore di segnale. Il gestore se ne accorge entro un giro
    atomic_store(&is_stop_requested, true);
}

void manager_get_status(wifi_status_t *status) {
    pthread_mutex_lock(&shared.lock);
    *status = shared.status;
    pthread_mutex_unlock(&shared.lock);
}

bool manager_is_ap(void) {
    return is_ap_on();
}

void manager_set_resting(bool is_resting) {
    pthread_mutex_lock(&shared.lock);
    shared.is_resting = is_resting;
    shared.resting_ms = now_ms();
    pthread_mutex_unlock(&shared.lock);
}

// funzione helper che riempie una risposta di rifiuto
static void reject(wifi_reply_t *reply, int http_code, const char *reason, const char *message) {
    memset(reply, 0, sizeof *reply);
    reply->http_code = http_code;
    reply->reason = reason;
    snprintf(reply->message, sizeof reply->message, "%s", message);
}

// funzione helper che mette una richiesta e ne aspetta la risposta per al massimo wait_s secondi. Si chiama con lock
// preso. Ritorna false se la risposta non arriva in tempo
static bool submit_request(request_type_t type, double wait_s, wifi_reply_t *reply) {

    unsigned request_num = ++shared.request_num;
    shared.request = type;
    pthread_cond_signal(&shared.request_cond);

    struct timespec until;
    clock_gettime(CLOCK_REALTIME, &until);
    int64_t wait_ns = (int64_t)(wait_s * 1e9);
    until.tv_sec += wait_ns / 1000000000L;
    until.tv_nsec += wait_ns % 1000000000L;
    until.tv_sec += until.tv_nsec / 1000000000L;
    until.tv_nsec %= 1000000000L;

    while (shared.reply_num != request_num) {
        if (pthread_cond_timedwait(&shared.reply_cond, &shared.lock, &until) != 0 && shared.reply_num != request_num) {
            // il gestore non l'ha presa: la si toglie, così non parte dopo
            if (shared.request_num == request_num && shared.request == type) shared.request = REQUEST_NONE;
            return false;
        }
    }
    *reply = shared.reply;
    return true;
}

// funzione helper che dice se il gestore è occupato (richiesta in corso o cambio in corso). Si chiama con lock preso
static bool is_busy(void) {
    return shared.request != REQUEST_NONE || shared.status.change_state == WIFI_CHANGE_TESTING;
}

void manager_scan(wifi_network_t *networks, wifi_reply_t *reply) {

    pthread_mutex_lock(&shared.lock);

    if (is_busy()) {
        pthread_mutex_unlock(&shared.lock);
        reject(reply, 409, "wifi_busy", "Maurice sta già cambiando rete o cercando le reti");
        return;
    }

    if (!submit_request(REQUEST_SCAN, WIFI_SCAN_REPLY_WAIT_S, reply)) {
        pthread_mutex_unlock(&shared.lock);
        reject(reply, 503, "scan_failed", "La ricerca delle reti non ha risposto in tempo");
        return;
    }

    // le reti dalla scansione salvata; "in uso" si decide adesso, con il collegamento di adesso
    reply->network_count = shared.network_count;
    for (int i = 0; i < shared.network_count; i++) {
        networks[i] = shared.networks[i];
        networks[i].is_in_use = shared.status.mode == WIFI_MODE_STATION &&
                                strcmp(networks[i].ssid, shared.status.link.ssid) == 0;
    }

    pthread_mutex_unlock(&shared.lock);
}

// funzione helper che controlla una password WPA: 8-63 caratteri ASCII stampabili
static bool is_valid_password(const char *password) {

    size_t length = strlen(password);
    if (length < WIFI_PASSWORD_MIN || length > WIFI_PASSWORD_MAX) return false;
    for (size_t i = 0; i < length; i++) {
        if (password[i] < 0x20 || password[i] > 0x7E) return false;
    }
    return true;
}

void manager_connect(const wifi_connect_request_t *request, wifi_reply_t *reply) {

    size_t ssid_length = strlen(request->ssid);
    if (ssid_length == 0 || ssid_length > WIFI_SSID_MAX || !is_valid_utf8(request->ssid)) {
        reject(reply, 400, "invalid_ssid", "Il nome della rete non è valido");
        return;
    }
    if (request->security == WIFI_SECURITY_WPA ? !is_valid_password(request->password) : request->password[0] != '\0') {
        reject(reply, 400, "invalid_password", "La password deve avere da 8 a 63 caratteri");
        return;
    }

    pthread_mutex_lock(&shared.lock);

    // senza notizie dal programma dei task da troppo tempo il robot è fermo: conta come a riposo
    bool is_resting = shared.resting_ms == 0 || now_ms() - shared.resting_ms > WIFI_RESTING_STALE_S * 1000 ||
                      shared.is_resting;

    if (!is_resting) {
        pthread_mutex_unlock(&shared.lock);
        reject(reply, 409, "not_resting", "Maurice deve essere a riposo per cambiare rete");
        return;
    }
    if (is_busy()) {
        pthread_mutex_unlock(&shared.lock);
        reject(reply, 409, "wifi_busy", "Maurice sta già cambiando rete o cercando le reti");
        return;
    }

    shared.connect = *request;
    bool is_answered = submit_request(REQUEST_CONNECT, WIFI_CONNECT_REPLY_WAIT_S, reply);
    memset(&shared.connect, 0, sizeof shared.connect);

    pthread_mutex_unlock(&shared.lock);

    if (!is_answered) reject(reply, 503, "wifi_busy", "Maurice non ha risposto in tempo");
}
