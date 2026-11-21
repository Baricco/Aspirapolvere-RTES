#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "manager.h"
#include "server.h"

// rtes-wifi: il Wi-Fi del robot (docs/wifi.md). Sul robot lo avvia /etc/init.d/rtes-wifi attraverso rtes-wifi-run.
//   rtes-wifi              Wi-Fi vero (wpa_supplicant, udhcpc, hostapd, dnsmasq), serve root
//   rtes-wifi --simulate   reti finte sul PC (radio_sim.c); variabili d'ambiente:
//       RTES_WIFI_SIM_SCALE  scala dei tempi (per esempio 0.1: i 30 s senza indirizzo diventano 3 s)
//       RTES_WIFI_HTTP_PORT  porta del server HTTP locale su 127.0.0.1 (default SIM_HTTP_PORT)
//       RTES_WIFI_SOCKET     socket per il programma dei task (vale anche sul robot)

#define SIM_SCALE_ENV       "RTES_WIFI_SIM_SCALE"
#define SIM_HTTP_PORT_ENV   "RTES_WIFI_HTTP_PORT"
#define SIM_HTTP_ADDRESS    "127.0.0.1"
#define SIM_HTTP_PORT       8090

static void stop(int signal_number) {
    (void)signal_number;
    manager_stop();
}

int main(int argc, char **argv) {

    bool is_simulated = argc > 1 && strcmp(argv[1], "--simulate") == 0;
    if (argc > 1 && !is_simulated) {
        fprintf(stderr, "uso: %s [--simulate]\n", argv[0]);
        return 2;
    }

    // una riga alla volta nel registro (procd la passa a logread)
    setvbuf(stderr, NULL, _IOLBF, 0);

    struct sigaction action = { .sa_handler = stop };
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGINT, &action, NULL);
    signal(SIGPIPE, SIG_IGN);

    const char *socket_path = getenv(WIFI_SOCKET_ENV) ? getenv(WIFI_SOCKET_ENV) : WIFI_SOCKET_PATH;
    const char *http_address = WIFI_AP_ADDRESS;
    int http_port = WIFI_HTTP_PORT;

    if (is_simulated) {
        if (getenv(SIM_SCALE_ENV)) time_scale = atof(getenv(SIM_SCALE_ENV));
        if (time_scale <= 0) time_scale = 1.0;
        http_address = SIM_HTTP_ADDRESS;
        http_port = getenv(SIM_HTTP_PORT_ENV) ? atoi(getenv(SIM_HTTP_PORT_ENV)) : SIM_HTTP_PORT;
        fprintf(stderr, "rtes-wifi: simulazione, scala dei tempi %.3f\n", time_scale);
    }

    if (!start_server(socket_path, http_address, http_port)) {
        fprintf(stderr, "rtes-wifi: il server locale non parte\n");
        return 1;
    }
    fprintf(stderr, "rtes-wifi: avviato, socket %s\n", socket_path);

    manager_run(is_simulated ? &radio_sim : &radio_real);
    return 0;
}
