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
GD32_SRC = gd32/gd32.c
GD32_HDR = gd32/gd32.h
LIDAR_SRC = lidar/lidar.c
LIDAR_HDR = lidar/lidar.h
# Librerie dei protocolli: ogni test e ogni strumento le compila tutte (sono piccole).
PROTO_SRC = $(GD32_SRC) $(LIDAR_SRC)
PROTO_HDR = $(GD32_HDR) $(LIDAR_HDR)

PROGRAMMI = latenza carico
STRUMENTI = spia
TEST      = test_rt_tempo test_rt_task test_gd32 test_lidar test_registro

SPIA_SRC = spia/spia.c spia/registro.c spia/everest.c
SPIA_HDR = spia/registro.h spia/everest.h

.PHONY: all host arm test test-arm clean

all: host arm

host: $(addprefix build/host/,$(PROGRAMMI) $(STRUMENTI) $(TEST))
arm:  $(addprefix build/arm/,$(PROGRAMMI) $(STRUMENTI) $(TEST))

# Programmi in bench/: un file bench/<nome>.c diventa l'eseguibile <nome>.
build/host/%: bench/%.c $(RT_SRC) $(RT_HDR)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -o $@ $< $(RT_SRC)

build/arm/%: bench/%.c $(RT_SRC) $(RT_HDR)
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -o $@ $< $(RT_SRC) $(LDFLAGS_ARM)

# La spia (spia/): intercetta e registra il traffico con GD32 e lidar.
build/host/spia: $(SPIA_SRC) $(SPIA_HDR) $(RT_SRC) $(RT_HDR) $(PROTO_SRC) $(PROTO_HDR)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -o $@ $(SPIA_SRC) $(RT_SRC) $(PROTO_SRC)

build/arm/spia: $(SPIA_SRC) $(SPIA_HDR) $(RT_SRC) $(RT_HDR) $(PROTO_SRC) $(PROTO_HDR)
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -o $@ $(SPIA_SRC) $(RT_SRC) $(PROTO_SRC) $(LDFLAGS_ARM)

build/host/test_%: test/test_%.c $(RT_SRC) $(RT_HDR) $(PROTO_SRC) $(PROTO_HDR) $(SPIA_HDR) spia/registro.c test/test.h
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -o $@ $< $(RT_SRC) $(PROTO_SRC) spia/registro.c

build/arm/test_%: test/test_%.c $(RT_SRC) $(RT_HDR) $(PROTO_SRC) $(PROTO_HDR) $(SPIA_HDR) spia/registro.c test/test.h
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -o $@ $< $(RT_SRC) $(PROTO_SRC) spia/registro.c $(LDFLAGS_ARM)

# Esegue tutti i test sul PC.
# Poi le prove complete della spia con periferiche e socket finti (tools/prova-spia*.py, ~3 s).
test: $(addprefix build/host/,$(TEST)) build/host/spia
	@set -e; for t in $(addprefix build/host/,$(TEST)); do echo "== $$t"; ./$$t; done
	@echo "== tools/prova-spia.py"; python3 tools/prova-spia.py build/host/spia 2 | tail -1
	@echo "== tools/prova-spia-socket.py"; python3 tools/prova-spia-socket.py build/host/spia | tail -1

# Esegue i test compilati per ARM sul PC tramite l'emulatore qemu-arm-static.
# Verifica che il binario del robot funzioni, ma i tempi non sono significativi.
test-arm: $(addprefix build/arm/,$(TEST))
	@set -e; for t in $^; do echo "== $$t (qemu)"; qemu-arm-static ./$$t; done

clean:
	rm -rf build
