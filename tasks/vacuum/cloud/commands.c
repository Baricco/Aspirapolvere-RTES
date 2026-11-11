#define _GNU_SOURCE   // per pthread_setaffinity_np
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "app_commands.h"
#include "commands.h"
#include "config_check.h"
#include "network.h"
#include "../aperiodic.h"

static char commands_path[256];
static pthread_t commands_thread;

// il corpo del thread: apre il file speciale (resta fermo finché qualcuno ci scrive), legge le righe, e quando chi scrive lo chiude lo riapre
static void *commands_loop(void *arg) {

    (void)arg;

    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(APERIODIC_CPU, &set);
    pthread_setaffinity_np(pthread_self(), sizeof set, &set);

    static char line[COMMAND_LINE_MAX];
    for (;;) {
        FILE *f = fopen(commands_path, "r");
        if (!f) {
            fprintf(stderr, "comandi: non riesco ad aprire %s (%s), niente comandi\n", commands_path, strerror(errno));
            return NULL;
        }
        while (fgets(line, sizeof line, f)) {
            line[strcspn(line, "\r\n")] = '\0';
            if (line[0] == '\0') continue;

            // lo stesso interprete del thread di rete. Dalla FIFO non si può scaricare una mappa dal server
            app_command_result_t result;
            run_app_command(line, NULL, &result);
            if (result.ok) fprintf(stderr, "comandi: %s: eseguito\n", line);
            else fprintf(stderr, "comandi: %s: rifiutato (%s: %s)\n", line, result.reason, result.message);

            // cambio di mappa: il programma ferma i task ed esce per essere rilanciato
            if (result.is_restart_needed) request_restart();
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
    pthread_attr_setstacksize(&attr, COMMANDS_FIFO_STACK);
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
