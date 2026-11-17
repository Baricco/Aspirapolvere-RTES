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

// Wi-Fi vero del robot (docs/wifi.md, sezioni 4-6). Le funzioni le chiama solo il thread del gestore.

#define STOP_TIMEOUT_MS         2000    // attesa della chiusura di un processo figlio dopo SIGTERM
#define COMMAND_TIMEOUT_MS      3000    // attesa massima di ifconfig
#define SUPPLICANT_START_MS     5000    // attesa massima del socket di controllo dopo l'avvio di wpa_supplicant
#define CHILD_RESTART_MS        5000    // un figlio che si chiude da solo riparte dopo questa attesa
#define START_STEP_MS           100     // attesa fra due tentativi di aprire il socket di controllo
#define HOSTAPD_SETTLE_MS       1000    // dopo l'avvio di hostapd, attesa prima di dare l'indirizzo all'interfaccia
#define EVENT_WAIT_MS           200     // attesa massima di un evento mentre si aspetta un collegamento o una scansione
#define REPLY_MAX               16384   // risposta più lunga di wpa_supplicant (SCAN_RESULTS)
#define DHCP_LEASE              "1h"    // durata degli indirizzi dati dalla rete del robot

#define LOG(...) do { fprintf(stderr, "rtes-wifi: " __VA_ARGS__); fputc('\n', stderr); } while (0)

static pid_t supplicant_pid = NO_CHILD, dhcp_pid = NO_CHILD, hostapd_pid = NO_CHILD, dnsmasq_pid = NO_CHILD;
static supplicant_link_t commands = SUPPLICANT_LINK_CLOSED, events = SUPPLICANT_LINK_CLOSED;

static bool is_station;             // wpa_supplicant deve girare
static bool is_station_connecting;  // false: rete salvata disabilitata (solo scansioni)
static bool is_ap;                  // la rete del robot deve essere accesa
static bool is_completed;           // wpa_supplicant collegato a una rete
static bool is_scan_ready;          // arrivato CTRL-EVENT-SCAN-RESULTS
static bool is_wrong_key;           // password sbagliata per la rete nuova
static int new_network_id = -1;     // rete nuova di un cambio in corso
static int64_t child_retry_ms;      // un figlio chiuso da solo non riparte prima di questo istante

static char reply[REPLY_MAX];

// --- Funzioni helper ---

static void pause_ms(int ms) {
    struct timespec t = { ms / 1000, (long)(ms % 1000) * 1000000L };
    nanosleep(&t, NULL);
}

// toglie l'indirizzo IPv4 dall'interfaccia
static void clear_address(void) {
    char *argv[] = { WIFI_IFCONFIG_BIN, WIFI_INTERFACE, "0.0.0.0", NULL };
    run_command(argv, COMMAND_TIMEOUT_MS);
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
    clear_address();
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

static bool real_start_station(bool is_connecting) {

    char *argv[] = { WIFI_SUPPLICANT_BIN, "-i", WIFI_INTERFACE, "-D", "nl80211", "-c", WIFI_SUPPLICANT_CONF, NULL };
    supplicant_pid = start_child(argv);
    is_station = true;
    is_station_connecting = is_connecting;
    is_completed = false;

    int64_t end = now_ms() + SUPPLICANT_START_MS;
    while (now_ms() < end) {
        if (supplicant_open(&commands, false)) {
            if (supplicant_open(&events, true)) break;
            supplicant_close(&commands);
        }
        pause_ms(START_STEP_MS);
    }
    if (commands.fd < 0) {
        LOG("wpa_supplicant non risponde");
        return false;
    }

    // solo scansioni: la rete salvata non si collega (non si salva mai in questo stato)
    if (!is_connecting) supplicant_ok(&commands, "DISABLE_NETWORK all");
    return true;
}

static void real_stop_station(void) {
    supplicant_close(&commands);
    supplicant_close(&events);
    stop_child(&dhcp_pid, STOP_TIMEOUT_MS);
    stop_child(&supplicant_pid, STOP_TIMEOUT_MS);
    clear_address();
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

    snprintf(text, sizeof text,
             "interface=%s\ndriver=nl80211\nssid=%s\nhw_mode=g\nchannel=%d\nignore_broadcast_ssid=0\nauth_algs=1\n"
             "wpa=2\nwpa_passphrase=%s\nwpa_key_mgmt=WPA-PSK\nrsn_pairwise=CCMP\n",
             WIFI_INTERFACE, WIFI_AP_SSID, WIFI_AP_CHANNEL, WIFI_AP_PASSWORD);
    bool ok = write_private_file(WIFI_RUN_DIR "/hostapd.conf", text);

    // solo DHCP: niente DNS (port=0), niente gateway e niente DNS dati ai telefoni (option 3 e 6 vuote), così il
    // telefono non cerca internet su questa rete
    snprintf(text, sizeof text,
             "interface=%s\nbind-interfaces\nport=0\nuser=root\ndhcp-authoritative\n"
             "dhcp-range=%s,%s,%s,%s\ndhcp-option=3\ndhcp-option=6\ndhcp-leasefile=%s/dnsmasq.leases\n",
             WIFI_INTERFACE, WIFI_AP_DHCP_FIRST, WIFI_AP_DHCP_LAST, WIFI_AP_NETMASK, DHCP_LEASE, WIFI_RUN_DIR);
    ok = write_private_file(WIFI_RUN_DIR "/dnsmasq.conf", text) && ok;

    if (!ok) {
        LOG("non riesco a scrivere la configurazione della rete del robot in %s", WIFI_RUN_DIR);
        return false;
    }

    char *hostapd_argv[] = { WIFI_HOSTAPD_BIN, WIFI_RUN_DIR "/hostapd.conf", NULL };
    hostapd_pid = start_child(hostapd_argv);
    pause_ms(HOSTAPD_SETTLE_MS);

    char *ifconfig_argv[] = { WIFI_IFCONFIG_BIN, WIFI_INTERFACE, WIFI_AP_ADDRESS, "netmask", WIFI_AP_NETMASK, "up", NULL };
    run_command(ifconfig_argv, COMMAND_TIMEOUT_MS);

    char *dnsmasq_argv[] = { WIFI_DNSMASQ_BIN, "-k", "-C", WIFI_RUN_DIR "/dnsmasq.conf", NULL };
    dnsmasq_pid = start_child(dnsmasq_argv);

    is_ap = true;
    return hostapd_pid != NO_CHILD && dnsmasq_pid != NO_CHILD;
}

static void real_stop_ap(void) {
    stop_child(&dnsmasq_pid, STOP_TIMEOUT_MS);
    stop_child(&hostapd_pid, STOP_TIMEOUT_MS);
    clear_address();
    is_ap = false;
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
            real_start_station(is_station_connecting);
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
}

static bool real_get_link(wifi_link_t *link) {

    memset(link, 0, sizeof *link);

    if (is_ap) {
        snprintf(link->ssid, sizeof link->ssid, "%s", WIFI_AP_SSID);
        snprintf(link->ip, sizeof link->ip, "%s", WIFI_AP_ADDRESS);
        return false;
    }

    // collegato = wpa_supplicant collegato a una rete E un indirizzo. Solo l'indirizzo non basta: quando la rete cade
    // (hotspot spento, router spento) l'indirizzo dato dal router resta sull'interfaccia finché scade, anche un'ora
    // (visto il 2026-11-17). Lo stato si chiede a ogni giro, così non dipende da un evento perso
    char value[128];
    is_completed = supplicant_command(&commands, "STATUS", reply, sizeof reply) &&
                   supplicant_value(reply, "wpa_state", value, sizeof value) && strcmp(value, "COMPLETED") == 0;
    if (!is_completed) return false;

    if (supplicant_value(reply, "ssid", value, sizeof value)) decode_ssid(value, link->ssid, sizeof link->ssid);
    if (supplicant_command(&commands, "SIGNAL_POLL", reply, sizeof reply) &&
        supplicant_value(reply, "RSSI", value, sizeof value)) {
        link->signal_dbm = atoi(value);
    }
    return read_address(link->ip, sizeof link->ip);
}

static bool real_has_saved_network(void) {

    // il file salvato, non la memoria di wpa_supplicant: durante un cambio la rete nuova c'è ma non è salvata
    FILE *f = fopen(WIFI_SUPPLICANT_CONF, "r");
    if (f == NULL) return false;

    char line[256];
    bool found = false;
    while (!found && fgets(line, sizeof line, f) != NULL) {
        found = strstr(line, "network={") != NULL;
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

    // reti nascoste (nome vuoto) e sicurezze che il robot non usa: WEP, Enterprise, solo WPA3
    if (fields[4][0] == '\0') return false;
    if (strstr(flags, "WEP") != NULL) return false;
    bool has_psk = strstr(flags, "-PSK") != NULL;
    if (!has_psk && (strstr(flags, "WPA") != NULL || strstr(flags, "RSN") != NULL || strstr(flags, "SAE") != NULL)) return false;

    memset(network, 0, sizeof *network);
    decode_ssid(fields[4], network->ssid, sizeof network->ssid);
    if (network->ssid[0] == '\0' || !is_valid_utf8(network->ssid)) return false;

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
    clear_address();

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
    .has_saved_network = real_has_saved_network,
    .scan = real_scan,
    .try_network_begin = real_try_network_begin,
    .try_network_wait = real_try_network_wait,
    .try_network_commit = real_try_network_commit,
    .try_network_abort = real_try_network_abort,
    .check_backend = check_backend,
    .ping_internet = ping_internet,
};
