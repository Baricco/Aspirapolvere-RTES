#ifndef WIFI_WIFI_H
#define WIFI_WIFI_H

// rtes-wifi: il Wi-Fi del robot (docs/wifi.md). Tipi comuni a tutti i file della cartella.

#include <stdbool.h>
#include <stdint.h>

#include "tasks/vacuum/cloud/config.h"

#define WIFI_SSID_MAX       32      // lunghezza massima di un SSID, in byte (802.11)
#define WIFI_PASSWORD_MIN   8       // lunghezza minima di una password WPA
#define WIFI_PASSWORD_MAX   63      // lunghezza massima di una password WPA (oltre sarebbe la chiave in esadecimale)
#define WIFI_IP_MAX         16      // "255.255.255.255" più il terminatore

// modo del Wi-Fi (docs/wifi.md, sezione 4)
typedef enum {
    WIFI_MODE_CONNECTING,           // prova la rete salvata
    WIFI_MODE_STATION,              // collegato alla rete salvata
    WIFI_MODE_TESTING,              // cambio di rete in corso
    WIFI_MODE_AP,                   // rete del robot accesa
} wifi_mode_t;

// sicurezza di una rete: solo quelle che il robot sa usare
typedef enum {
    WIFI_SECURITY_OPEN,
    WIFI_SECURITY_WPA,              // WPA o WPA2 con password
} wifi_security_t;

// fase dell'ultimo cambio di rete
typedef enum {
    WIFI_CHANGE_NONE,               // nessun cambio da quando rtes-wifi è partito
    WIFI_CHANGE_TESTING,
    WIFI_CHANGE_DONE,
} wifi_change_state_t;

// esito di un cambio di rete (docs/wifi.md, sezione 5)
typedef enum {
    WIFI_RESULT_NONE,
    WIFI_RESULT_OK,
    WIFI_RESULT_BACKEND_UNREACHABLE,
    WIFI_RESULT_INTERNET_UNREACHABLE,
    WIFI_RESULT_NO_ADDRESS,
    WIFI_RESULT_WRONG_PASSWORD,
    WIFI_RESULT_NOT_FOUND,
} wifi_result_t;

// una rete trovata dalla scansione
typedef struct {
    char ssid[WIFI_SSID_MAX + 1];   // byte veri dell'SSID, in UTF-8 valido (le altre si scartano)
    int signal_dbm;
    wifi_security_t security;
    bool is_in_use;
} wifi_network_t;

// richiesta di cambio di rete
typedef struct {
    char ssid[WIFI_SSID_MAX + 1];
    wifi_security_t security;
    char password[WIFI_PASSWORD_MAX + 1];   // vuota per le reti aperte
} wifi_connect_request_t;

// collegamento di adesso
typedef struct {
    char ssid[WIFI_SSID_MAX + 1];   // rete in uso (in AP: quella del robot), vuoto se nessuna
    int signal_dbm;                 // 0 se non si sa
    char ip[WIFI_IP_MAX];           // indirizzo IPv4 di wlan0, vuoto se non c'è
} wifi_link_t;

// esito dell'attesa del collegamento a una rete nuova
typedef enum {
    WIFI_ASSOC_OK,
    WIFI_ASSOC_WRONG_KEY,
    WIFI_ASSOC_TIMEOUT,
} wifi_assoc_t;

// nomi usati nel JSON
const char *wifi_mode_name(wifi_mode_t mode);
const char *wifi_security_name(wifi_security_t security);
const char *wifi_change_state_name(wifi_change_state_t state);
const char *wifi_result_name(wifi_result_t result);

// true se i byte sono UTF-8 valido
bool is_valid_utf8(const char *text);

// istante di adesso in millisecondi (orologio monotono)
int64_t now_ms(void);

// istante di adesso in secondi dal 1970 (per l'app)
int64_t epoch_s(void);

// tempi della configurazione in millisecondi, moltiplicati per la scala del tempo (1 sul robot, più piccola nella
// simulazione per provare l'alternanza senza aspettare minuti)
int64_t scaled_ms(double seconds);
extern double time_scale;

#endif
