#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "check.h"
#include "radio.h"

// Wi-Fi finto per le prove sul PC (rtes-wifi --simulate, docs/wifi.md sezione 12). Reti fisse con password e
// comportamento noto; SIGUSR1 spegne e riaccende il router (prova del blackout e dell'alternanza). Variabili d'ambiente:
//   RTES_WIFI_SIM_SAVED  rete salvata all'avvio (default "Casa"; vuota = nessuna)
//   RTES_SERVER_URL      se c'è, la prova del backend sulla rete "Casa" è vera (backend di sviluppo), altrimenti finta

#define SIM_SAVED_ENV       "RTES_WIFI_SIM_SAVED"
#define SIM_SAVED_DEFAULT   "Casa"
#define SIM_ASSOC_S         1.0     // tempo per collegarsi a una rete (prima della scala del tempo)
#define SIM_DHCP_S          1.0     // tempo per avere l'indirizzo
#define SIM_SCAN_S          2.0     // durata di una scansione
#define SIM_STEP_MS         50

#define LOG(...) do { fprintf(stderr, "rtes-wifi (sim): " __VA_ARGS__); fputc('\n', stderr); } while (0)

typedef struct {
    const char *ssid;
    wifi_security_t security;
    const char *password;
    int signal_dbm;
    bool is_visible;                // false: rete nascosta, si trova solo scrivendo il nome
    bool has_address;               // il router dà l'indirizzo
    bool has_internet;
    bool has_backend;               // il backend è raggiungibile da questa rete
} sim_network_t;

static const sim_network_t NETWORKS[] = {
    { "Casa",           WIFI_SECURITY_WPA,  "password-casa",     -48, true,  true,  true,  true  },
    { "Casa",           WIFI_SECURITY_WPA,  "password-casa",     -71, true,  true,  true,  true  },  // secondo punto di accesso
    { "Vicini 2.4",     WIFI_SECURITY_WPA,  "segreta123",        -77, true,  true,  true,  true  },
    { "Ufficio",        WIFI_SECURITY_WPA,  "password-ufficio",  -60, true,  true,  true,  false },
    { "Ospiti",         WIFI_SECURITY_OPEN, "",                  -66, true,  true,  false, false },
    { "Senza DHCP",     WIFI_SECURITY_WPA,  "password-dhcp",     -64, true,  false, false, false },
    { "Caffè ☕",       WIFI_SECURITY_OPEN, "",                  -82, true,  true,  true,  true  },
    { "Nascosta",       WIFI_SECURITY_WPA,  "password-nascosta", -58, false, true,  true,  true  },
};
#define NETWORKS_NUM ((int)(sizeof NETWORKS / sizeof NETWORKS[0]))

static atomic_bool is_router_on = true;     // SIGUSR1 lo cambia
static char saved_ssid[WIFI_SSID_MAX + 1];  // rete salvata ("" = nessuna)
static char trying_ssid[WIFI_SSID_MAX + 1]; // rete di un cambio in corso
static char trying_password[WIFI_PASSWORD_MAX + 1];
static bool is_station, is_connecting, is_ap;
static const sim_network_t *current;        // rete a cui la stazione è collegata
static int64_t connect_at_ms;               // la stazione si collega da questo istante
static bool use_real_backend;

static void pause_ms(int ms) {
    struct timespec t = { ms / 1000, (long)(ms % 1000) * 1000000L };
    nanosleep(&t, NULL);
}

static void toggle_router(int signal_number) {
    (void)signal_number;
    atomic_store(&is_router_on, !atomic_load(&is_router_on));
}

// la rete più forte con questo nome, se il router è acceso
static const sim_network_t *find_network(const char *ssid) {
    if (!atomic_load(&is_router_on)) return NULL;
    for (int i = 0; i < NETWORKS_NUM; i++) {
        if (strcmp(NETWORKS[i].ssid, ssid) == 0) return &NETWORKS[i];
    }
    return NULL;
}

static bool sim_init(void) {

    const char *saved = getenv(SIM_SAVED_ENV);
    snprintf(saved_ssid, sizeof saved_ssid, "%s", saved != NULL ? saved : SIM_SAVED_DEFAULT);
    signal(SIGUSR1, toggle_router);

    use_real_backend = getenv(SERVER_URL_ENV) != NULL;
    if (use_real_backend && !init_backend_check()) LOG("manca il token: la prova del backend vera fallirà");

    LOG("rete salvata «%s»; kill -USR1 %d spegne e riaccende il router", saved_ssid, (int)getpid());
    return true;
}

static bool sim_start_station(bool connecting) {
    is_station = true;
    is_connecting = connecting;
    current = NULL;
    connect_at_ms = now_ms() + scaled_ms(SIM_ASSOC_S + SIM_DHCP_S);
    return true;
}

static void sim_stop_station(void) {
    is_station = false;
    current = NULL;
}

static bool sim_start_ap(void) {
    is_ap = true;
    LOG("rete del robot «%s» accesa", WIFI_AP_SSID);
    return true;
}

static void sim_stop_ap(void) {
    is_ap = false;
    LOG("rete del robot spenta");
}

static void sim_poll(void) {

    if (!is_station || !is_connecting || trying_ssid[0] != '\0') return;

    // la stazione segue la rete salvata: si collega dopo un po', cade se il router si spegne
    const sim_network_t *network = find_network(saved_ssid);
    if (network == NULL || !network->has_address) {
        if (current != NULL) LOG("rete «%s» persa", current->ssid);
        current = NULL;
        connect_at_ms = now_ms() + scaled_ms(SIM_ASSOC_S + SIM_DHCP_S);
    } else if (current == NULL && now_ms() >= connect_at_ms) {
        current = network;
    }
}

static bool sim_get_link(wifi_link_t *link) {

    memset(link, 0, sizeof *link);

    if (is_ap) {
        snprintf(link->ssid, sizeof link->ssid, "%s", WIFI_AP_SSID);
        snprintf(link->ip, sizeof link->ip, "%s", WIFI_AP_ADDRESS);
        return false;
    }
    if (!is_station || current == NULL) return false;

    // durante un cambio la rete nuova dà l'indirizzo solo se ha DHCP
    if (!current->has_address) {
        snprintf(link->ssid, sizeof link->ssid, "%s", current->ssid);
        link->signal_dbm = current->signal_dbm;
        return false;
    }
    snprintf(link->ssid, sizeof link->ssid, "%s", current->ssid);
    link->signal_dbm = current->signal_dbm;
    snprintf(link->ip, sizeof link->ip, "192.168.1.%u", 40u + (unsigned)(current - NETWORKS) % 200u);
    return true;
}

static bool sim_has_saved_network(void) {
    return saved_ssid[0] != '\0';
}

static int sim_scan(wifi_network_t *list, int max, int64_t timeout_ms) {

    if (!is_station) return -1;

    int64_t duration = scaled_ms(SIM_SCAN_S);
    pause_ms((int)(duration < timeout_ms ? duration : timeout_ms));

    int count = 0;
    for (int i = 0; i < NETWORKS_NUM && count < max; i++) {
        if (!NETWORKS[i].is_visible || !atomic_load(&is_router_on)) continue;
        memset(&list[count], 0, sizeof list[count]);
        snprintf(list[count].ssid, sizeof list[count].ssid, "%s", NETWORKS[i].ssid);
        list[count].signal_dbm = NETWORKS[i].signal_dbm;
        list[count].security = NETWORKS[i].security;
        count++;
    }
    return count;
}

static bool sim_try_network_begin(const wifi_connect_request_t *request) {
    snprintf(trying_ssid, sizeof trying_ssid, "%s", request->ssid);
    snprintf(trying_password, sizeof trying_password, "%s", request->password);
    current = NULL;
    return true;
}

static wifi_assoc_t sim_try_network_wait(int64_t timeout_ms) {

    const sim_network_t *network = find_network(trying_ssid);
    int64_t assoc_ms = scaled_ms(SIM_ASSOC_S);

    if (network == NULL) {
        pause_ms((int)timeout_ms);
        return WIFI_ASSOC_TIMEOUT;
    }
    pause_ms((int)(assoc_ms < timeout_ms ? assoc_ms : timeout_ms));
    if (network->security == WIFI_SECURITY_WPA && strcmp(network->password, trying_password) != 0) {
        return WIFI_ASSOC_WRONG_KEY;
    }
    current = network;
    pause_ms((int)scaled_ms(SIM_DHCP_S));
    return WIFI_ASSOC_OK;
}

static void sim_try_network_commit(void) {
    snprintf(saved_ssid, sizeof saved_ssid, "%s", trying_ssid);
    trying_ssid[0] = '\0';
    memset(trying_password, 0, sizeof trying_password);
}

static void sim_try_network_abort(void) {
    trying_ssid[0] = '\0';
    memset(trying_password, 0, sizeof trying_password);
    current = NULL;
    connect_at_ms = now_ms() + scaled_ms(SIM_ASSOC_S + SIM_DHCP_S);
}

static bool sim_check_backend(void) {
    if (current == NULL || !current->has_backend) {
        pause_ms(200);
        return false;
    }
    return use_real_backend ? check_backend() : true;
}

static bool sim_ping_internet(void) {
    pause_ms(100);
    return current != NULL && current->has_internet;
}

const radio_t radio_sim = {
    .init = sim_init,
    .start_station = sim_start_station,
    .stop_station = sim_stop_station,
    .start_ap = sim_start_ap,
    .stop_ap = sim_stop_ap,
    .poll = sim_poll,
    .get_link = sim_get_link,
    .has_saved_network = sim_has_saved_network,
    .scan = sim_scan,
    .try_network_begin = sim_try_network_begin,
    .try_network_wait = sim_try_network_wait,
    .try_network_commit = sim_try_network_commit,
    .try_network_abort = sim_try_network_abort,
    .check_backend = sim_check_backend,
    .ping_internet = sim_ping_internet,
};
