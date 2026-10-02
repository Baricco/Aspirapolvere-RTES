#!/bin/bash
# Riproduce la prova del 2026-10-02: FreeRTOS ufficiale (port POSIX) con il tick corretto, sul robot.
# Scarica il kernel V11.1.0, applica port-tick-rt.diff, compila main.c per ARM e lo esegue sul robot
# (core 3, SCHED_FIFO), con un timer di emergenza che lo termina dopo 60 s.
set -eu
DIR="$(cd "$(dirname "$0")" && pwd)"; RTES="$DIR/../.."
B="$RTES/build/freertos-posix"; mkdir -p "$B"; cd "$B"
[ -d FreeRTOS-Kernel-11.1.0 ] || { curl -sL https://github.com/FreeRTOS/FreeRTOS-Kernel/archive/refs/tags/V11.1.0.tar.gz | tar xz
  (cd FreeRTOS-Kernel-11.1.0 && patch -p1 < "$DIR/port-tick-rt.diff"); }
K=FreeRTOS-Kernel-11.1.0; P=$K/portable/ThirdParty/GCC/Posix
"$RTES/toolchain/bin/arm-linux-gcc" -O2 -static -pthread -D_GNU_SOURCE -include pthread.h \
  -mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -I"$DIR" -I$K/include -I$P -I$P/utils -o frt-demo \
  "$DIR/main.c" $K/tasks.c $K/list.c $K/queue.c $K/timers.c $K/event_groups.c $K/portable/MemMang/heap_3.c \
  $P/port.c $P/utils/wait_for_event.c
"$RTES/../ssh-robot.sh" 'mkdir -p /tmp/rtes && cat > /tmp/rtes/frt-demo && chmod +x /tmp/rtes/frt-demo' < frt-demo
"$RTES/../ssh-robot.sh" '/tmp/rtes/frt-demo & p=$!; (sleep 60; kill -9 $p 2>/dev/null && echo "TERMINATO DAL TIMER DI EMERGENZA") & wait $p'
