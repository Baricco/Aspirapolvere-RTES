#!/bin/bash
# Compila e lancia la prova delle funzioni della pulizia (tools/prove-nav/funzioni-pulizia.c) sul codice di
# tasks/vacuum così com'è, senza simulatore. Uso (dalla cartella del progetto): tools/prove-nav/funzioni.sh
set -e
cd "$(dirname "$0")/../.."
mkdir -p build/prove-nav
gcc -std=gnu11 -O2 -g -Wall -Wextra -Wno-unused-function -pthread -I. -Itasks/vacuum -o build/prove-nav/funzioni \
    tools/prove-nav/funzioni-pulizia.c $(ls tasks/vacuum/*.c | grep -v -E '/(main|navigation)\.c$') \
    driver/driver.c driver/scambio.c driver/batteria.c driver/lidar_giro.c driver/contatti.c spia/registro.c rt/rt.c \
    gd32/gd32.c gd32/sicurezza.c gd32/descrizione.c lidar/lidar.c -lm
exec build/prove-nav/funzioni
