#!/bin/bash
# Copia tasks/vacuum in build/prove-nav/vac, aggiunge una stampa a ogni cambio di stato della navigazione
# (riga "NAV t da a x y theta maxv c2c lunghezza_percorso celle_chiuse"), le righe PERCORSO e PULIZIA per la
# pulizia (sotto), e la compila in build/prove-nav/vacdbg.
# Il codice in tasks/ non viene toccato.
set -e
cd "$(dirname "$0")/../.."
D=build/prove-nav
rm -rf $D/vac; mkdir -p $D; cp -r tasks/vacuum $D/vac; F=$D/vac/navigation.c
sed -i '5a #include <stdio.h>' $F
sed -i 's|^    nav_state.state = next_state;\(.*\)$|    { movement_position_data_t p = read_movement(); safety_alarm_t a = read_safety(); fprintf(stderr, "NAV %.2f %d %d %.3f %.3f %.0f %.3f %d %u %u\\n", rt_adesso() / 1e9, nav_state.state, next_state, p.x, p.y, p.theta * 57.29578, a.max_v_forw, nav_state.center_to_center, nav_state.path.length, nav_state.closed_cells.num); }\n    nav_state.state = next_state;\1|' $F
grep -q 'NAV %.2f' $F || { echo "stampa non inserita: in set_state non trovo 'nav_state.state = next_state;'" >&2; exit 1; }
# per la pulizia: "PERCORSO t lunghezza scostamento dal_primo_punto" a ogni partenza di un percorso, e
# "PULIZIA t libere N meta_pulite M celle_con_meta_sporca K" quando la navigazione va a riposo (fine della pulizia)
if grep -q '^static void start_following_path(.*lat_shift.*is_first_point.*{$' $F; then
    sed -i '/^static void start_following_path(.*{$/a\    fprintf(stderr, "PERCORSO %.2f %u %.3f %d\\n", rt_adesso() / 1e9, nav_state.path.length, lat_shift, (int)is_first_point);' $F
else echo "avviso: start_following_path(..., lat_shift, is_first_point) non trovata, niente righe PERCORSO" >&2; fi
if grep -q 'is_half_cell_clean' $F; then
    sed -i 's|^\(    { movement_position_data_t p = read_movement(); safety_alarm_t a = read_safety(); fprintf(stderr, "NAV \)|    if (next_state == NAV_STATE_SLEEP) { uint32_t lib = 0, pul = 0, sp = 0; for (uint32_t c = 0; c < NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER; c++) if (nav_state.map[c] == NAV_CELL_FREE) { int h = is_half_cell_clean(c, 0) + is_half_cell_clean(c, 1); lib++; pul += h; if (h < 2) sp++; } fprintf(stderr, "PULIZIA %.2f libere %u meta_pulite %u celle_con_meta_sporca %u\\n", rt_adesso() / 1e9, lib, pul, sp); }\n\1|' $F
else echo "avviso: is_half_cell_clean non trovata, niente riga PULIZIA" >&2; fi
# per provare una modifica senza toccare tasks/: MODIFICA=script.py, chiamato con la cartella della copia
if [ -n "$MODIFICA" ]; then python3 -I "$MODIFICA" $D/vac; fi
gcc -std=gnu11 -O2 -g -Wall -Wextra -pthread -I. -o $D/vacdbg $D/vac/*.c driver/driver.c driver/scambio.c \
    driver/batteria.c driver/lidar_giro.c driver/contatti.c spia/registro.c rt/rt.c gd32/gd32.c gd32/sicurezza.c \
    gd32/descrizione.c lidar/lidar.c -lm
