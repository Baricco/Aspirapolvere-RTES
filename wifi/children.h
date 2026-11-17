#ifndef WIFI_CHILDREN_H
#define WIFI_CHILDREN_H

// Processi figli di rtes-wifi (wpa_supplicant, udhcpc, hostapd, dnsmasq) e comandi brevi (ifconfig). Un figlio si
// chiude da solo se rtes-wifi muore (PR_SET_PDEATHSIG), così procd non trova processi rimasti a metà.

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#define NO_CHILD 0

// avvia un programma (argv terminato da NULL) e ritorna il suo pid, oppure NO_CHILD
pid_t start_child(char *const argv[]);

// true se il figlio gira ancora; se è uscito lo raccoglie e mette *pid = NO_CHILD
bool is_child_running(pid_t *pid);

// chiude un figlio: SIGTERM, poi SIGKILL dopo timeout_ms. Mette *pid = NO_CHILD
void stop_child(pid_t *pid, int64_t timeout_ms);

// esegue un comando breve e ne aspetta la fine. Ritorna true se esce con 0
bool run_command(char *const argv[], int64_t timeout_ms);

// chiude tutti i processi con questo nome (da un avvio precedente di rtes-wifi o dagli script di prima)
void kill_by_name(const char *name);

#endif
