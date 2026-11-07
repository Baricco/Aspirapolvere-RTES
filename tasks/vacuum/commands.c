#define _GNU_SOURCE   // per pthread_setaffinity_np
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "commands.h"
#include "aperiodic.h"
#include "mapping.h"

#define COMMANDS_PATH_DEFAULT   "/tmp/rtes/comandi-vacuum"  // file speciale dei comandi sul robot (sul PC si sceglie con RTES_COMANDI)
#define COMMANDS_PATH_ENV       "RTES_COMANDI"              // variabile d'ambiente con il percorso del file dei comandi
#define COMMAND_LINE_MAX        128                         // lunghezza massima di una riga di comando
#define COMMANDS_STACK          (64 * 1024)                 // stack del thread: con mlockall ogni byte di stack resta in RAM (il default è 8 MB)

static char commands_path[256];
static pthread_t commands_thread;

// zone dell'utente volute: le tiene solo questo thread, e le prende dalla lista pubblicata dalla mappatura al primo comando
static mapping_forbidden_zone_t zones[MAX_FORBIDDEN_ZONES - 1];
static int num_zones;
static bool zones_ready;

// funzione helper che prende le zone dell'utente dalla lista pubblicata dalla mappatura (il primo posto è della base).
// Ritorna false se la sessione non è partita dalla base: le coordinate delle zone hanno senso solo lì
static bool load_zones(void) {

    if (!read_started_on_base()) {
        fprintf(stderr, "comandi: zone rifiutate, la sessione non è partita dalla base (la mappa non ha l'origine lì)\n");
        return false;
    }
    if (zones_ready) return true;

    mapping_forbidden_zone_list_t list = read_forbidden_zone_list();
    num_zones = 0;
    for (int i = 1; i < list.num_zones; i++) zones[num_zones++] = list.forbidden_zones[i];
    zones_ready = true;
    return true;
}

// esegue una riga di comando
static void run_command(const char *line) {

    double x, y, w, h;
    int index;

    if (sscanf(line, "zona aggiungi %lf %lf %lf %lf", &x, &y, &w, &h) == 4) {
        if (!load_zones()) return;
        if (num_zones >= MAX_FORBIDDEN_ZONES - 1) {
            fprintf(stderr, "comandi: zona rifiutata, ci sono già %d zone\n", num_zones);
            return;
        }
        zones[num_zones++] = (mapping_forbidden_zone_t){ .x = x, .y = y, .w = w, .h = h };
        request_user_zones(zones, num_zones);
        fprintf(stderr, "comandi: zona %d aggiunta, da (%.2f; %.2f) di %.2f x %.2f m\n", num_zones - 1, x, y, w, h);

    } else if (sscanf(line, "zona togli %d", &index) == 1) {
        if (!load_zones()) return;
        if (index < 0 || index >= num_zones) {
            fprintf(stderr, "comandi: la zona %d non c'è (zone: %d)\n", index, num_zones);
            return;
        }
        for (int i = index; i < num_zones - 1; i++) zones[i] = zones[i + 1];
        num_zones--;
        request_user_zones(zones, num_zones);
        fprintf(stderr, "comandi: zona %d tolta\n", index);

    } else if (strncmp(line, "zone", 4) == 0) {
        if (!load_zones()) return;
        fprintf(stderr, "comandi: %d zone\n", num_zones);
        for (int i = 0; i < num_zones; i++)
            fprintf(stderr, "  zona %d: da (%.2f; %.2f) di %.2f x %.2f m\n", i, zones[i].x, zones[i].y, zones[i].w, zones[i].h);

    } else if (line[0] != '\0') {
        fprintf(stderr, "comandi: comando sconosciuto \"%s\"\n", line);
    }
}

// il corpo del thread: apre il file speciale (resta fermo finché qualcuno ci scrive), legge le righe, e quando chi
// scrive lo chiude lo riapre
static void *commands_loop(void *arg) {

    (void)arg;

    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(APERIODIC_CPU, &set);
    pthread_setaffinity_np(pthread_self(), sizeof set, &set);

    char line[COMMAND_LINE_MAX];
    for (;;) {
        FILE *f = fopen(commands_path, "r");
        if (!f) {
            fprintf(stderr, "comandi: non riesco ad aprire %s (%s), niente comandi\n", commands_path, strerror(errno));
            return NULL;
        }
        while (fgets(line, sizeof line, f)) {
            line[strcspn(line, "\r\n")] = '\0';
            run_command(line);
        }
        fclose(f);
    }
    return NULL;
}

int start_commands(void) {

    const char *path = getenv(COMMANDS_PATH_ENV) ? getenv(COMMANDS_PATH_ENV) : COMMANDS_PATH_DEFAULT;
    snprintf(commands_path, sizeof commands_path, "%s", path);

    // il file speciale: se c'è già (anche di una sessione prima) va bene così
    if (mkfifo(commands_path, 0600) != 0 && errno != EEXIST) {
        fprintf(stderr, "comandi: non riesco a creare %s (%s), niente comandi\n", commands_path, strerror(errno));
        return -1;
    }

    // scheduling normale: il thread viene creato dal main, che non è real-time
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, COMMANDS_STACK);
    int r = pthread_create(&commands_thread, &attr, commands_loop, NULL);
    pthread_attr_destroy(&attr);
    if (r != 0) {
        fprintf(stderr, "comandi: non riesco a creare il thread, niente comandi\n");
        return -1;
    }
    pthread_detach(commands_thread);
    fprintf(stderr, "comandi: in ascolto su %s\n", commands_path);
    return 0;
}
