#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "check.h"
#include "children.h"
#include "radio.h"
#include "supplicant.h"

// Wi-Fi vero del robot (docs/wifi.md, sezioni 2-6): wlan0 collegato alla rete salvata, wlan1 rete del robot. Le
// funzioni le chiama solo il thread del gestore.

#define STOP_TIMEOUT_MS         2000    // attesa della chiusura di un processo figlio dopo SIGTERM
#define COMMAND_TIMEOUT_MS      3000    // attesa massima di ifconfig
#define SUPPLICANT_START_MS     5000    // attesa massima del socket di controllo dopo l'avvio di wpa_supplicant
#define CHILD_RESTART_MS        5000    // un figlio che si chiude da solo riparte dopo questa attesa
#define START_STEP_MS           100     // attesa fra due tentativi di aprire il socket di controllo
#define HOSTAPD_SETTLE_MS       1000    // dopo l'avvio di hostapd, attesa prima di dare l'indirizzo all'interfaccia
#define EVENT_WAIT_MS           200     // attesa massima di un evento mentre si aspetta un collegamento o una scansione
#define REPLY_MAX               16384   // risposta più lunga di wpa_supplicant (SCAN_RESULTS)
#define DHCP_LEASE              "1h"    // durata degli indirizzi dati dalla rete del robot
#define AP_CLIENTS_MAX          16      // telefoni contati sulla rete del robot
#define MAC_TEXT_MAX            18      // "aa:bb:cc:dd:ee:ff" più il terminatore
#define SUPPLICANT_CTRL_PATH    WIFI_SUPPLICANT_CTRL_DIR "/" WIFI_INTERFACE
#define HOSTAPD_CTRL_DIR        WIFI_RUN_DIR "/hostapd"
#define HOSTAPD_CTRL_PATH       HOSTAPD_CTRL_DIR "/" WIFI_AP_INTERFACE

#define LOG(...) do { fprintf(stderr, "rtes-wifi: " __VA_ARGS__); fputc('\n', stderr); } while (0)

static pid_t supplicant_pid = NO_CHILD, dhcp_pid = NO_CHILD, hostapd_pid = NO_CHILD, dnsmasq_pid = NO_CHILD;
static supplicant_link_t commands = SUPPLICANT_LINK_CLOSED, events = SUPPLICANT_LINK_CLOSED;
static supplicant_link_t ap_events = SUPPLICANT_LINK_CLOSED;

static bool is_station;             // wpa_supplicant deve girare
static bool is_ap;                  // la rete del robot deve essere accesa
static bool is_completed;           // wpa_supplicant collegato a una rete
static bool is_scan_ready;          // arrivato CTRL-EVENT-SCAN-RESULTS (per la scansione chiesta)
static bool has_new_results;        // arrivato CTRL-EVENT-SCAN-RESULTS (per il canale della rete del robot)
static bool is_wrong_key;           // password sbagliata per la rete nuova
static int new_network_id = -1;     // rete nuova di un cambio in corso
static int ap_channel;              // canale della rete del robot accesa
static int wanted_channel;          // canale della rete che wlan0 usa o cerca, 0 se non si sa
static char clients[AP_CLIENTS_MAX][MAC_TEXT_MAX];  // telefoni collegati alla rete del robot
static int client_count;
static int64_t child_retry_ms;      // un figlio chiuso da solo non riparte prima di questo istante

static char reply[REPLY_MAX];

// --- Funzioni helper ---

static void pause_ms(int ms) {
    struct timespec t = { ms / 1000, (long)(ms % 1000) * 1000000L };
    nanosleep(&t, NULL);
}

// toglie l'indirizzo IPv4 dall'interfaccia
static void clear_address(char *interface) {
    char *argv[] = { WIFI_IFCONFIG_BIN, interface, "0.0.0.0", NULL };
    run_command(argv, COMMAND_TIMEOUT_MS);
}

// canale di una frequenza a 2,4 GHz, 0 se non lo è
static int channel_of(int frequency_mhz) {
    if (frequency_mhz == 2484) return 14;
    if (frequency_mhz >= 2412 && frequency_mhz <= 2472) return (frequency_mhz - 2407) / 5;
    return 0;
}

// indirizzo IPv4 dell'interfaccia in ip (vuoto se non c'è). Ritorna true se c'è
static bool read_address(char *ip, size_t size) {

    ip[0] = '\0';
    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return false;

    struct ifreq request;
    memset(&request, 0, sizeof request);
    snprintf(request.ifr_name, sizeof request.ifr_name, "%s", WIFI_INTERFACE);
    bool ok = ioctl(fd, SIOCGIFADDR, &request) == 0;
    close(fd);

    if (!ok) return false;
    struct sockaddr_in *address = (struct sockaddr_in *)&request.ifr_addr;
    if (address->sin_addr.s_addr == 0) return false;
    inet_ntop(AF_INET, &address->sin_addr, ip, (socklen_t)size);
    return true;
}

// riporta un SSID scritto da wpa_supplicant ai byte veri: "\\" "\"" "\e" "\n" "\r" "\t" e "\xNN"
static void decode_ssid(const char *encoded, char *ssid, size_t size) {

    size_t n = 0;

    for (const char *s = encoded; *s && n + 1 < size; s++) {
        if (*s != '\\' || s[1] == '\0') {
            ssid[n++] = *s;
            continue;
        }
        s++;
        switch (*s) {
            case 'n': ssid[n++] = '\n'; break;
            case 'r': ssid[n++] = '\r'; break;
            case 't': ssid[n++] = '\t'; break;
            case 'e': ssid[n++] = '\033'; break;
            case 'x':
                if (s[1] && s[2]) {
                    char hex[3] = { s[1], s[2], '\0' };
                    ssid[n++] = (char)strtol(hex, NULL, 16);
                    s += 2;
                }
                break;
            default: ssid[n++] = *s; break;     // "\\" e "\""
        }
    }
    ssid[n] = '\0';
}

// avvia udhcpc: lo si fa solo a collegamento fatto (udhcpc 1.24 avviato prima riceve "Network is down" e non manda
// più richieste, visto all'avvio il 2026-11-13)
static void start_dhcp(void) {
    char *argv[] = { WIFI_UDHCPC_BIN, "-f", "-i", WIFI_INTERFACE, "-S", "-T", "10", NULL };
    dhcp_pid = start_child(argv);
}

// toglie il collegamento in corso: udhcpc fermo, nessun indirizzo
static void drop_connection(void) {
    stop_child(&dhcp_pid, STOP_TIMEOUT_MS);
    clear_address(WIFI_INTERFACE);
    is_completed = false;
}

// legge un evento di wpa_supplicant e aggiorna i flag
static void handle_event(const char *event) {

    // gli eventi iniziano con il livello: "<3>CTRL-EVENT-CONNECTED - ..."
    const char *text = event[0] == '<' && strchr(event, '>') ? strchr(event, '>') + 1 : event;

    if (strncmp(text, "CTRL-EVENT-CONNECTED", 20) == 0) {
        is_completed = true;
        // nuovo collegamento senza indirizzo (per esempio dopo un riavvio del router): udhcpc potrebbe essersi bloccato
        char ip[WIFI_IP_MAX];
        if (!read_address(ip, sizeof ip)) stop_child(&dhcp_pid, STOP_TIMEOUT_MS);
    } else if (strncmp(text, "CTRL-EVENT-DISCONNECTED", 23) == 0) {
        is_completed = false;
    } else if (strncmp(text, "CTRL-EVENT-SCAN-RESULTS", 23) == 0) {
        is_scan_ready = true;
        has_new_results = true;
    } else if (strncmp(text, "CTRL-EVENT-SSID-TEMP-DISABLED", 29) == 0 && strstr(text, "reason=WRONG_KEY") != NULL) {
        char id[16];
        snprintf(id, sizeof id, "id=%d ", new_network_id);
        if (new_network_id >= 0 && strstr(text, id) != NULL) is_wrong_key = true;
    }
}

// legge tutti gli eventi già arrivati, oppure aspetta il primo per al massimo timeout_ms
static void read_events(int64_t timeout_ms) {
    char event[512];
    while (supplicant_event(&events, event, sizeof event, timeout_ms)) {
        handle_event(event);
        timeout_ms = 0;
    }
}

// --- Stazione ---

static bool real_start_station(void) {

    char *argv[] = { WIFI_SUPPLICANT_BIN, "-i", WIFI_INTERFACE, "-D", "nl80211", "-c", WIFI_SUPPLICANT_CONF, NULL };
    supplicant_pid = start_child(argv);
    is_station = true;
    is_completed = false;

    int64_t end = now_ms() + SUPPLICANT_START_MS;
    while (now_ms() < end) {
        if (supplicant_open(&commands, SUPPLICANT_CTRL_PATH, false)) {
            if (supplicant_open(&events, SUPPLICANT_CTRL_PATH, true)) break;
            supplicant_close(&commands);
        }
        pause_ms(START_STEP_MS);
    }
    if (commands.fd < 0) {
        LOG("wpa_supplicant non risponde");
        return false;
    }

    // senza rete, una ricerca della rete salvata ogni WIFI_RETRY_SCAN_S
    char command[32];
    snprintf(command, sizeof command, "SCAN_INTERVAL %d", WIFI_RETRY_SCAN_S);
    if (!supplicant_ok(&commands, command)) LOG("%s non accettato", command);
    return true;
}

static void real_stop_station(void) {
    supplicant_close(&commands);
    supplicant_close(&events);
    stop_child(&dhcp_pid, STOP_TIMEOUT_MS);
    stop_child(&supplicant_pid, STOP_TIMEOUT_MS);
    clear_address(WIFI_INTERFACE);
    is_station = false;
    is_completed = false;
}

// --- Rete del robot ---

// scrive un file di configurazione leggibile solo da root (contiene la password)
static bool write_private_file(const char *path, const char *text) {

    FILE *f = fopen(path, "w");
    if (f == NULL) return false;
    chmod(path, 0600);
    bool ok = fputs(text, f) >= 0;
    return fclose(f) == 0 && ok;
}

static bool real_start_ap(void) {

    char text[1024];

    ap_channel = wanted_channel != 0 ? wanted_channel : WIFI_AP_CHANNEL;
    client_count = 0;

    snprintf(text, sizeof text,
             "interface=%s\ndriver=nl80211\nctrl_interface=%s\nssid=%s\nhw_mode=g\nchannel=%d\n"
             "ignore_broadcast_ssid=0\nauth_algs=1\nap_max_inactivity=%d\n"
             "wpa=2\nwpa_passphrase=%s\nwpa_key_mgmt=WPA-PSK\nrsn_pairwise=CCMP\n",
             WIFI_AP_INTERFACE, HOSTAPD_CTRL_DIR, WIFI_AP_SSID, ap_channel, WIFI_AP_INACTIVITY_S, WIFI_AP_PASSWORD);
    bool ok = write_private_file(WIFI_RUN_DIR "/hostapd.conf", text);

    // solo DHCP: niente DNS (port=0), niente gateway e niente DNS dati ai telefoni (option 3 e 6 vuote), così il
    // telefono non cerca internet su questa rete
    snprintf(text, sizeof text,
             "interface=%s\nbind-interfaces\nport=0\nuser=root\ndhcp-authoritative\n"
             "dhcp-range=%s,%s,%s,%s\ndhcp-option=3\ndhcp-option=6\ndhcp-leasefile=%s/dnsmasq.leases\n",
             WIFI_AP_INTERFACE, WIFI_AP_DHCP_FIRST, WIFI_AP_DHCP_LAST, WIFI_AP_NETMASK, DHCP_LEASE, WIFI_RUN_DIR);
    ok = write_private_file(WIFI_RUN_DIR "/dnsmasq.conf", text) && ok;

    if (!ok) {
        LOG("non riesco a scrivere la configurazione della rete del robot in %s", WIFI_RUN_DIR);
        return false;
    }

    char *hostapd_argv[] = { WIFI_HOSTAPD_BIN, WIFI_RUN_DIR "/hostapd.conf", NULL };
    hostapd_pid = start_child(hostapd_argv);
    pause_ms(HOSTAPD_SETTLE_MS);

    char *ifconfig_argv[] = { WIFI_IFCONFIG_BIN, WIFI_AP_INTERFACE, WIFI_AP_ADDRESS, "netmask", WIFI_AP_NETMASK, "up", NULL };
    run_command(ifconfig_argv, COMMAND_TIMEOUT_MS);

    char *dnsmasq_argv[] = { WIFI_DNSMASQ_BIN, "-k", "-C", WIFI_RUN_DIR "/dnsmasq.conf", NULL };
    dnsmasq_pid = start_child(dnsmasq_argv);

    is_ap = true;
    LOG("rete del robot sul canale %d", ap_channel);
    return hostapd_pid != NO_CHILD && dnsmasq_pid != NO_CHILD;
}

static void real_stop_ap(void) {
    supplicant_close(&ap_events);
    stop_child(&dnsmasq_pid, STOP_TIMEOUT_MS);
    stop_child(&hostapd_pid, STOP_TIMEOUT_MS);
    clear_address(WIFI_AP_INTERFACE);
    char *argv[] = { WIFI_IFCONFIG_BIN, WIFI_AP_INTERFACE, "down", NULL };
    run_command(argv, COMMAND_TIMEOUT_MS);
    is_ap = false;
    client_count = 0;
}

// legge un evento di hostapd e aggiorna i telefoni collegati
static void handle_ap_event(const char *event) {

    const char *text = event[0] == '<' && strchr(event, '>') ? strchr(event, '>') + 1 : event;
    bool is_connected = strncmp(text, "AP-STA-CONNECTED ", 17) == 0;
    bool is_disconnected = strncmp(text, "AP-STA-DISCONNECTED ", 20) == 0;
    if (!is_connected && !is_disconnected) return;

    char mac[MAC_TEXT_MAX];
    snprintf(mac, sizeof mac, "%.17s", strchr(text, ' ') + 1);

    int found = -1;
    for (int i = 0; i < client_count; i++) {
        if (strcmp(clients[i], mac) == 0) found = i;
    }
    if (is_connected && found < 0 && client_count < AP_CLIENTS_MAX) {
        snprintf(clients[client_count++], MAC_TEXT_MAX, "%s", mac);
    } else if (is_disconnected && found >= 0) {
        memmove(clients[found], clients[--client_count], MAC_TEXT_MAX);
    }
    LOG("telefono %s %s: %d collegati", mac, is_connected ? "collegato" : "uscito", client_count);
}

// --- Canale della rete del robot ---

// SSID (come lo scrive wpa_supplicant) della rete che wlan0 usa o cerca: la prima non disabilitata. Ritorna false se
// non ce n'è
static bool read_wanted_ssid(char *ssid, size_t size) {

    char list[2048];
    if (!supplicant_command(&commands, "LIST_NETWORKS", list, sizeof list)) return false;

    // righe "id\tssid\tbssid\tflag", la prima è l'intestazione
    char *rest = list;
    char *line = strsep(&rest, "\n");
    while ((line = strsep(&rest, "\n")) != NULL) {
        char *fields[4];
        int count = 0;
        while (count < 4 && (fields[count] = strsep(&line, "\t")) != NULL) count++;
        if (count >= 2 && (count < 4 || strstr(fields[3], "[DISABLED]") == NULL)) {
            snprintf(ssid, size, "%s", fields[1]);
            return true;
        }
    }
    return false;
}

// canale della rete che wlan0 cerca nei risultati dell'ultima scansione (punto di accesso più forte)
static void read_wanted_channel_from_scan(void) {

    char wanted[4 * WIFI_SSID_MAX + 1];
    if (!read_wanted_ssid(wanted, sizeof wanted)) return;
    if (!supplicant_command(&commands, "SCAN_RESULTS", reply, sizeof reply)) return;

    // righe "bssid\tfrequenza\tsegnale\tflag\tssid", la prima è l'intestazione
    int best_signal = -1000;
    char *rest = reply;
    char *line = strsep(&rest, "\n");
    while ((line = strsep(&rest, "\n")) != NULL) {
        char *fields[5];
        int count = 0;
        while (count < 5 && (fields[count] = strsep(&line, "\t")) != NULL) count++;
        if (count < 5 || strcmp(fields[4], wanted) != 0 || atoi(fields[2]) <= best_signal) continue;
        int channel = channel_of(atoi(fields[1]));
        if (channel == 0) continue;
        best_signal = atoi(fields[2]);
        wanted_channel = channel;
    }
}

// rete del robot riaccesa sul canale della rete che wlan0 usa o cerca, se è diverso
static void follow_wanted_channel(void) {

    if (has_new_results && is_station) {
        has_new_results = false;
        read_wanted_channel_from_scan();
    }
    if (is_ap && wanted_channel != 0 && wanted_channel != ap_channel) {
        LOG("la rete cercata è sul canale %d: sposto la rete del robot", wanted_channel);
        real_stop_ap();
        real_start_ap();
    }
}

// --- Giro del gestore ---

static void real_poll(void) {

    int64_t now = now_ms();

    if (is_station) {
        if (!is_child_running(&supplicant_pid) && now >= child_retry_ms) {
            LOG("wpa_supplicant si è chiuso: lo riavvio");
            child_retry_ms = now + CHILD_RESTART_MS;
            supplicant_close(&commands);
            supplicant_close(&events);
            stop_child(&dhcp_pid, STOP_TIMEOUT_MS);
            real_start_station();
        }
        read_events(0);
        if (is_completed && !is_child_running(&dhcp_pid)) start_dhcp();
    }

    if (is_ap && now >= child_retry_ms) {
        if (!is_child_running(&hostapd_pid) || !is_child_running(&dnsmasq_pid)) {
            LOG("hostapd o dnsmasq si è chiuso: riaccendo la rete del robot");
            child_retry_ms = now + CHILD_RESTART_MS;
            real_stop_ap();
            real_start_ap();
        }
    }

    if (is_ap) {
        if (ap_events.fd < 0) supplicant_open(&ap_events, HOSTAPD_CTRL_PATH, true);
        char event[256];
        while (supplicant_event(&ap_events, event, sizeof event, 0)) handle_ap_event(event);
    }

    follow_wanted_channel();
}

static bool real_get_link(wifi_link_t *link) {

    memset(link, 0, sizeof *link);

    // collegato = wpa_supplicant collegato a una rete E un indirizzo. Solo l'indirizzo non basta: quando la rete cade
    // (hotspot spento, router spento) l'indirizzo dato dal router resta sull'interfaccia finché scade, anche un'ora
    // (visto il 2026-11-17). Lo stato si chiede a ogni giro, così non dipende da un evento perso
    char value[128];
    is_completed = supplicant_command(&commands, "STATUS", reply, sizeof reply) &&
                   supplicant_value(reply, "wpa_state", value, sizeof value) && strcmp(value, "COMPLETED") == 0;
    if (!is_completed) return false;

    if (supplicant_value(reply, "freq", value, sizeof value) && channel_of(atoi(value)) != 0) {
        wanted_channel = channel_of(atoi(value));
    }
    if (supplicant_value(reply, "ssid", value, sizeof value)) decode_ssid(value, link->ssid, sizeof link->ssid);
    if (supplicant_command(&commands, "SIGNAL_POLL", reply, sizeof reply) &&
        supplicant_value(reply, "RSSI", value, sizeof value)) {
        link->signal_dbm = atoi(value);
    }
    return read_address(link->ip, sizeof link->ip);
}

static int real_get_ap_clients(void) {
    return is_ap ? client_count : 0;
}

// riporta l'SSID scritto da SAVE_CONFIG ai byte veri: fra virgolette, oppure in esadecimale
static void decode_saved_ssid(const char *value, char *ssid, size_t size) {

    size_t n = 0;

    if (value[0] == '"') {
        const char *end = strrchr(value + 1, '"');
        size_t length = end != NULL ? (size_t)(end - value - 1) : strlen(value + 1);
        n = length < size - 1 ? length : size - 1;
        memcpy(ssid, value + 1, n);
    } else {
        for (const char *s = value; s[0] && s[1] && n + 1 < size; s += 2) {
            char hex[3] = { s[0], s[1], '\0' };
            ssid[n++] = (char)strtol(hex, NULL, 16);
        }
    }
    ssid[n] = '\0';
}

static bool real_get_saved_ssid(char *ssid, size_t size) {

    ssid[0] = '\0';

    // il file salvato, non la memoria di wpa_supplicant: durante un cambio la rete nuova c'è ma non è salvata
    FILE *f = fopen(WIFI_SUPPLICANT_CONF, "r");
    if (f == NULL) return false;

    char line[256];
    bool is_in_network = false, found = false;
    while (!found && fgets(line, sizeof line, f) != NULL) {
        line[strcspn(line, "\r\n")] = '\0';
        const char *text = line + strspn(line, " \t");
        if (strncmp(text, "network={", 9) == 0) {
            is_in_network = true;
        } else if (is_in_network && strncmp(text, "ssid=", 5) == 0) {
            decode_saved_ssid(text + 5, ssid, size);
            found = true;
        }
    }
    fclose(f);
    return found;
}

// --- Scansione ---

// legge una riga di SCAN_RESULTS ("bssid\tfrequenza\tsegnale\tflag\tssid"). Ritorna false se la rete si scarta
static bool parse_scan_line(char *line, wifi_network_t *network) {

    char *fields[5];
    for (int i = 0; i < 5; i++) {
        fields[i] = strsep(&line, "\t");
        if (fields[i] == NULL) return false;
    }
    const char *flags = fields[3];

    // reti nascoste (nome vuoto), la rete del robot e sicurezze che il robot non usa: WEP, Enterprise, solo WPA3
    if (fields[4][0] == '\0') return false;
    if (strstr(flags, "WEP") != NULL) return false;
    bool has_psk = strstr(flags, "-PSK") != NULL;
    if (!has_psk && (strstr(flags, "WPA") != NULL || strstr(flags, "RSN") != NULL || strstr(flags, "SAE") != NULL)) return false;

    memset(network, 0, sizeof *network);
    decode_ssid(fields[4], network->ssid, sizeof network->ssid);
    if (network->ssid[0] == '\0' || !is_valid_utf8(network->ssid) || strcmp(network->ssid, WIFI_AP_SSID) == 0) return false;

    network->signal_dbm = atoi(fields[2]);
    network->security = has_psk ? WIFI_SECURITY_WPA : WIFI_SECURITY_OPEN;
    return true;
}

static int real_scan(wifi_network_t *list, int max, int64_t timeout_ms) {

    if (!is_station || commands.fd < 0) return -1;

    is_scan_ready = false;
    char answer[64];
    if (!supplicant_command(&commands, "SCAN", answer, sizeof answer)) return -1;
    // FAIL-BUSY: una scansione è già in corso, si aspetta quella
    if (strncmp(answer, "OK", 2) != 0 && strncmp(answer, "FAIL-BUSY", 9) != 0) return -1;

    int64_t end = now_ms() + timeout_ms;
    while (!is_scan_ready && now_ms() < end) read_events(EVENT_WAIT_MS);
    if (!is_scan_ready) LOG("scansione senza risposta in %lld ms: uso i risultati che ci sono", (long long)timeout_ms);

    if (!supplicant_command(&commands, "SCAN_RESULTS", reply, sizeof reply)) return -1;

    int count = 0;
    char *rest = reply;
    char *line = strsep(&rest, "\n");      // la prima riga è l'intestazione
    while ((line = strsep(&rest, "\n")) != NULL && count < max) {
        if (parse_scan_line(line, &list[count])) count++;
    }
    return count;
}

// --- Cambio di rete ---

// manda "SET_NETWORK id ..." e ritorna true se risponde OK
static bool set_network(int id, const char *name, const char *value) {
    char command[256];
    snprintf(command, sizeof command, "SET_NETWORK %d %s %s", id, name, value);
    return supplicant_ok(&commands, command);
}

static bool real_try_network_begin(const wifi_connect_request_t *request) {

    if (commands.fd < 0) return false;

    if (!supplicant_command(&commands, "ADD_NETWORK", reply, sizeof reply)) return false;
    char *end;
    long id = strtol(reply, &end, 10);
    if (end == reply) return false;

    // SSID in esadecimale: va bene per qualsiasi byte, senza problemi di virgolette
    char ssid_hex[2 * WIFI_SSID_MAX + 1];
    for (size_t i = 0; request->ssid[i]; i++) sprintf(ssid_hex + 2 * i, "%02x", (unsigned char)request->ssid[i]);

    // la password va fra virgolette: wpa_supplicant prende tutto fino all'ultima, anche se ce ne sono dentro
    char quoted[WIFI_PASSWORD_MAX + 3];
    snprintf(quoted, sizeof quoted, "\"%s\"", request->password);

    bool ok = set_network((int)id, "ssid", ssid_hex) && set_network((int)id, "scan_ssid", "1");
    if (request->security == WIFI_SECURITY_WPA) {
        ok = ok && set_network((int)id, "key_mgmt", "WPA-PSK") && set_network((int)id, "psk", quoted);
    } else {
        ok = ok && set_network((int)id, "key_mgmt", "NONE");
    }
    memset(quoted, 0, sizeof quoted);

    if (!ok) {
        char command[64];
        snprintf(command, sizeof command, "REMOVE_NETWORK %ld", id);
        supplicant_ok(&commands, command);
        return false;
    }

    // la rete di prima resta in wpa_supplicant (disabilitata da SELECT_NETWORK) finché la nuova non va
    drop_connection();
    new_network_id = (int)id;
    is_wrong_key = false;

    char command[64];
    snprintf(command, sizeof command, "SELECT_NETWORK %ld", id);
    return supplicant_ok(&commands, command);
}

static wifi_assoc_t real_try_network_wait(int64_t timeout_ms) {

    int64_t end = now_ms() + timeout_ms;

    while (now_ms() < end) {
        read_events(EVENT_WAIT_MS);
        follow_wanted_channel();
        if (is_wrong_key) return WIFI_ASSOC_WRONG_KEY;

        // collegato, e alla rete nuova
        char value[16];
        if (is_completed && supplicant_command(&commands, "STATUS", reply, sizeof reply) &&
            supplicant_value(reply, "id", value, sizeof value) && atoi(value) == new_network_id) {
            if (!is_child_running(&dhcp_pid)) start_dhcp();
            return WIFI_ASSOC_OK;
        }
    }
    return WIFI_ASSOC_TIMEOUT;
}

// copia del file di configurazione prima della prima modifica
static void backup_config(void) {

    const char *backup = WIFI_SUPPLICANT_CONF ".bak";
    if (access(backup, F_OK) == 0) return;

    FILE *in = fopen(WIFI_SUPPLICANT_CONF, "r");
    FILE *out = fopen(backup, "w");
    if (in != NULL && out != NULL) {
        char buffer[1024];
        size_t n;
        while ((n = fread(buffer, 1, sizeof buffer, in)) > 0) fwrite(buffer, 1, n, out);
    }
    if (in != NULL) fclose(in);
    if (out != NULL) {
        fclose(out);
        chmod(backup, 0600);
    }
}

static void real_try_network_commit(void) {

    // tolte tutte le reti tranne la nuova, poi salvate
    if (supplicant_command(&commands, "LIST_NETWORKS", reply, sizeof reply)) {
        int ids[32], count = 0;
        char *rest = reply;
        char *line = strsep(&rest, "\n");
        while ((line = strsep(&rest, "\n")) != NULL && count < 32) {
            char *end;
            long id = strtol(line, &end, 10);
            if (end != line && id != new_network_id) ids[count++] = (int)id;
        }
        for (int i = 0; i < count; i++) {
            char command[64];
            snprintf(command, sizeof command, "REMOVE_NETWORK %d", ids[i]);
            supplicant_ok(&commands, command);
        }
    }

    backup_config();
    if (!supplicant_ok(&commands, "SAVE_CONFIG")) LOG("SAVE_CONFIG non riuscito: la rete nuova non è salvata");
    new_network_id = -1;
}

static void real_try_network_abort(void) {

    char command[64];
    snprintf(command, sizeof command, "REMOVE_NETWORK %d", new_network_id);
    supplicant_ok(&commands, command);
    new_network_id = -1;

    drop_connection();
    supplicant_ok(&commands, "ENABLE_NETWORK all");
    supplicant_ok(&commands, "REASSOCIATE");
}

// --- Preparazione ---

static bool real_init(void) {

    mkdir(WIFI_RUN_DIR, 0700);

    // processi rimasti dagli script di prima o da un avvio precedente
    kill_by_name("wpa_cli");
    kill_by_name("udhcpc");
    kill_by_name("wpa_supplicant");
    kill_by_name("hostapd");
    kill_by_name("dnsmasq");
    pause_ms(STOP_TIMEOUT_MS / 2);

    char *argv[] = { WIFI_IFCONFIG_BIN, WIFI_INTERFACE, "up", NULL };
    run_command(argv, COMMAND_TIMEOUT_MS);

    // l'indirizzo lasciato dal Wi-Fi di prima farebbe credere per un attimo di essere già collegati
    clear_address(WIFI_INTERFACE);
    clear_address(WIFI_AP_INTERFACE);

    if (!init_backend_check()) LOG("manca il token del robot: la prova del backend fallirà sempre");
    return true;
}

const radio_t radio_real = {
    .init = real_init,
    .start_station = real_start_station,
    .stop_station = real_stop_station,
    .start_ap = real_start_ap,
    .stop_ap = real_stop_ap,
    .poll = real_poll,
    .get_link = real_get_link,
    .get_ap_clients = real_get_ap_clients,
    .get_saved_ssid = real_get_saved_ssid,
    .scan = real_scan,
    .try_network_begin = real_try_network_begin,
    .try_network_wait = real_try_network_wait,
    .try_network_commit = real_try_network_commit,
    .try_network_abort = real_try_network_abort,
    .check_backend = check_backend,
    .ping_internet = ping_internet,
};
