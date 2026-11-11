#define _GNU_SOURCE   // per pthread_setaffinity_np
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

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

        // se nel frattempo qualcuno l'ha sostituito con un file normale, rileggerlo all'infinito ripeterebbe gli stessi
        // comandi a tutta velocità sul core del task aperiodico: il thread si ferma
        struct stat st;
        if (fstat(fileno(f), &st) != 0 || !S_ISFIFO(st.st_mode)) {
            fprintf(stderr, "comandi: %s non è più un file speciale, niente comandi\n", commands_path);
            fclose(f);
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

// funzione helper che prepara il file speciale. Uno che c'è già (di una sessione prima, o del programma rilanciato dopo
// un cambio di mappa) si riusa solo se è un file speciale di questo utente con i permessi giusti; qualsiasi altra cosa
// (file normale, collegamento, file speciale di un altro utente o scrivibile da altri) si cancella e si ricrea.
// Ritorna 0, oppure -1 se non ci riesce
static int prepare_fifo(void) {

    struct stat st;
    if (lstat(commands_path, &st) == 0) {
        if (S_ISFIFO(st.st_mode) && st.st_uid == geteuid() && (st.st_mode & 0777) == COMMANDS_FIFO_MODE) return 0;
        if (unlink(commands_path) != 0) {
            fprintf(stderr, "comandi: %s non è un file speciale del programma e non riesco a cancellarlo (%s), niente comandi\n",
                    commands_path, strerror(errno));
            return -1;
        }
        fprintf(stderr, "comandi: %s non era un file speciale del programma: cancellato e ricreato\n", commands_path);
    } else if (errno != ENOENT) {
        fprintf(stderr, "comandi: non riesco a controllare %s (%s), niente comandi\n", commands_path, strerror(errno));
        return -1;
    }

    // chmod dopo mkfifo: i permessi di mkfifo dipendono dalla umask di chi ha lanciato il programma
    if (mkfifo(commands_path, COMMANDS_FIFO_MODE) != 0 || chmod(commands_path, COMMANDS_FIFO_MODE) != 0) {
        fprintf(stderr, "comandi: non riesco a creare %s (%s), niente comandi\n", commands_path, strerror(errno));
        return -1;
    }
    return 0;
}

int start_commands(void) {

    // il file speciale serve solo per le prove: senza la variabile d'ambiente non c'è, e nessuno può comandare il robot da lì
    const char *path = getenv(COMMANDS_PATH_ENV);
    if (path == NULL || path[0] == '\0') {
        fprintf(stderr, "comandi: %s non impostata, niente file dei comandi\n", COMMANDS_PATH_ENV);
        return 0;
    }
    if (strlen(path) >= sizeof commands_path) {
        fprintf(stderr, "comandi: percorso %s troppo lungo, niente comandi\n", path);
        return -1;
    }
    snprintf(commands_path, sizeof commands_path, "%s", path);

    if (prepare_fifo() != 0) return -1;

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
