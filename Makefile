# Makefile del progetto. Di solito non si usa direttamente: passa da ./rtes.sh.
#
# Ogni programma si compila in due versioni dallo stesso sorgente:
#   build/host/  per il PC (gcc di sistema): test e simulazione
#   build/arm/   per il robot (cross-compilatore Bootlin, ARMv7, statico)
#
# "Statico" vuol dire che l'eseguibile contiene già la libreria C (musl):
# sul robot non dipende dalle librerie installate, basta copiarlo.

TOOLCHAIN ?= toolchain
CC_HOST   ?= gcc
CC_ARM    ?= $(TOOLCHAIN)/bin/arm-linux-gcc

# -std=gnu11: C11 più le estensioni POSIX/GNU (clock_nanosleep, CPU_SET, ...)
CFLAGS_COMUNI = -std=gnu11 -O2 -g -Wall -Wextra -pthread
CFLAGS_HOST   = $(CFLAGS_COMUNI)
# Il robot ha un Allwinner A33: 4 core Cortex-A7 con FPU VFPv4 e NEON.
CFLAGS_ARM    = $(CFLAGS_COMUNI) -mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard
LDFLAGS_ARM   = -static

RT_SRC  = rt/rt.c
RT_HDR  = rt/rt.h rt/rt_tempo.h

PROGRAMMI = latenza carico
TEST      = test_rt_tempo test_rt_task

.PHONY: all host arm test test-arm clean

all: host arm

host: $(addprefix build/host/,$(PROGRAMMI) $(TEST))
arm:  $(addprefix build/arm/,$(PROGRAMMI) $(TEST))

# Programmi in bench/: un file bench/<nome>.c diventa l'eseguibile <nome>.
build/host/%: bench/%.c $(RT_SRC) $(RT_HDR)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -o $@ $< $(RT_SRC)

build/arm/%: bench/%.c $(RT_SRC) $(RT_HDR)
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -o $@ $< $(RT_SRC) $(LDFLAGS_ARM)

build/host/test_%: test/test_%.c $(RT_SRC) $(RT_HDR) test/test.h
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -o $@ $< $(RT_SRC)

build/arm/test_%: test/test_%.c $(RT_SRC) $(RT_HDR) test/test.h
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -o $@ $< $(RT_SRC) $(LDFLAGS_ARM)

# Esegue tutti i test sul PC.
test: $(addprefix build/host/,$(TEST))
	@set -e; for t in $^; do echo "== $$t"; ./$$t; done

# Esegue i test compilati per ARM sul PC tramite l'emulatore qemu-arm-static.
# Verifica che il binario del robot funzioni, ma i tempi non sono significativi.
test-arm: $(addprefix build/arm/,$(TEST))
	@set -e; for t in $^; do echo "== $$t (qemu)"; qemu-arm-static ./$$t; done

clean:
	rm -rf build
