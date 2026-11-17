#include <dirent.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "children.h"
#include "wifi.h"

#define WAIT_STEP_MS        50      // attesa fra due controlli della fine di un figlio
#define KILL_WAIT_MS        1000    // dopo SIGKILL, attesa massima della fine

// funzione helper che aspetta qualche millisecondo
static void pause_ms(int ms) {
    struct timespec t = { ms / 1000, (long)(ms % 1000) * 1000000L };
    nanosleep(&t, NULL);
}

pid_t start_child(char *const argv[]) {

    pid_t pid = fork();

    if (pid == 0) {
        // nel figlio: chiuso da solo se rtes-wifi muore, segnali come di default
        prctl(PR_SET_PDEATHSIG, SIGTERM);
        signal(SIGPIPE, SIG_DFL);
        sigset_t none;
        sigemptyset(&none);
        sigprocmask(SIG_SETMASK, &none, NULL);
        execv(argv[0], argv);
        // dopo fork in un programma con più thread solo funzioni sicure: write, non fprintf
        static const char message[] = "rtes-wifi: execv non riuscita\n";
        if (write(STDERR_FILENO, message, sizeof message - 1) < 0) _exit(127);
        _exit(127);
    }

    if (pid < 0) {
        fprintf(stderr, "rtes-wifi: fork non riuscita per %s\n", argv[0]);
        return NO_CHILD;
    }
    return pid;
}

bool is_child_running(pid_t *pid) {

    if (*pid == NO_CHILD) return false;

    int status;
    if (waitpid(*pid, &status, WNOHANG) == 0) return true;

    *pid = NO_CHILD;
    return false;
}

// funzione helper che aspetta la fine di un figlio per al massimo timeout_ms. Ritorna true se è finito
static bool wait_child(pid_t *pid, int64_t timeout_ms, int *exit_status) {

    int64_t end = now_ms() + timeout_ms;

    while (true) {
        int status;
        pid_t r = waitpid(*pid, &status, WNOHANG);
        if (r == *pid || r < 0) {
            if (exit_status != NULL) *exit_status = r == *pid ? status : -1;
            *pid = NO_CHILD;
            return true;
        }
        if (now_ms() >= end) return false;
        pause_ms(WAIT_STEP_MS);
    }
}

void stop_child(pid_t *pid, int64_t timeout_ms) {

    if (*pid == NO_CHILD) return;

    kill(*pid, SIGTERM);
    if (wait_child(pid, timeout_ms, NULL)) return;

    kill(*pid, SIGKILL);
    wait_child(pid, KILL_WAIT_MS, NULL);
    *pid = NO_CHILD;
}

bool run_command(char *const argv[], int64_t timeout_ms) {

    pid_t pid = start_child(argv);
    if (pid == NO_CHILD) return false;

    int status;
    if (!wait_child(&pid, timeout_ms, &status)) {
        stop_child(&pid, KILL_WAIT_MS);
        return false;
    }
    return status != -1 && WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

void kill_by_name(const char *name) {

    DIR *proc = opendir("/proc");
    if (proc == NULL) return;

    struct dirent *entry;
    while ((entry = readdir(proc)) != NULL) {

        char *end;
        long pid = strtol(entry->d_name, &end, 10);
        if (*end != '\0' || pid <= 1 || pid == getpid()) continue;

        char path[64], comm[64] = "";
        snprintf(path, sizeof path, "/proc/%ld/comm", pid);
        FILE *f = fopen(path, "r");
        if (f == NULL) continue;
        if (fgets(comm, sizeof comm, f) == NULL) comm[0] = '\0';
        fclose(f);
        comm[strcspn(comm, "\n")] = '\0';

        if (strcmp(comm, name) == 0) {
            fprintf(stderr, "rtes-wifi: chiudo %s (%ld) rimasto da prima\n", name, pid);
            kill((pid_t)pid, SIGTERM);
        }
    }
    closedir(proc);
}
