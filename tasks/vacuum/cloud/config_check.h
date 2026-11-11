#ifndef CLOUD_CONFIG_CHECK_H
#define CLOUD_CONFIG_CHECK_H

// Include la configurazione privata robot-server. Se manca, la compilazione si ferma con un messaggio che dice cosa fare.

#if !__has_include("config.h")
#error "manca tasks/vacuum/cloud/config.h: copialo da tasks/vacuum/cloud/config.example.h e metti i valori veri (git lo ignora)"
#endif

#include "rt/rt.h"
#include "config.h"

#endif
