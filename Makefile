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
GD32_SRC = gd32/gd32.c gd32/sicurezza.c gd32/descrizione.c
GD32_HDR = gd32/gd32.h gd32/sicurezza.h gd32/descrizione.h
LIDAR_SRC = lidar/lidar.c
LIDAR_HDR = lidar/lidar.h
# Librerie dei protocolli: ogni test e ogni strumento le compila tutte (sono piccole).
PROTO_SRC = $(GD32_SRC) $(LIDAR_SRC)
PROTO_HDR = $(GD32_HDR) $(LIDAR_HDR)

PROGRAMMI = latenza carico
STRUMENTI = spia rtes-driver
TEST      = test_rt_tempo test_rt_task test_gd32 test_sicurezza test_lidar test_registro test_driver_logica test_sim test_descrizione

SPIA_SRC = spia/spia.c spia/registro.c spia/everest.c
SPIA_HDR = spia/registro.h spia/everest.h

# Driver (driver/): il nostro sostituto di AuxCtrl. rtes-driver è il programma per provarlo da solo.
DRIVER_SRC = driver/driver.c driver/scambio.c driver/batteria.c driver/lidar_giro.c driver/contatti.c spia/registro.c
DRIVER_HDR = driver/driver.h driver/scambio.h driver/batteria.h driver/lidar_giro.h driver/contatti.h spia/registro.h

# Simulatore (sim/): GD32 e lidar finti su pseudo-seriali, pagina web con la stanza. Solo per il PC.
SIM_LIB = sim/mondo.c sim/gd32sim.c sim/lidarsim.c sim/parametri.c
SIM_SRC = sim/sim.c $(SIM_LIB)
SIM_HDR = sim/mondo.h sim/gd32sim.h sim/lidarsim.h sim/parametri.h sim/robot.h

.PHONY: all host arm test test-arm clean

all: host arm

host: $(addprefix build/host/,$(PROGRAMMI) $(STRUMENTI) $(TEST)) build/host/sim build/host/osserva
arm:  $(addprefix build/arm/,$(PROGRAMMI) $(STRUMENTI) $(TEST))

# Programmi in bench/: un file bench/<nome>.c diventa l'eseguibile <nome>.
build/host/%: bench/%.c $(RT_SRC) $(RT_HDR)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -o $@ $< $(RT_SRC)

build/arm/%: bench/%.c $(RT_SRC) $(RT_HDR)
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -o $@ $< $(RT_SRC) $(LDFLAGS_ARM)

# I programmi con i task (fase 4): tasks/<nome>.c diventa build/host/<nome> (per il simulatore) e build/arm/<nome>
# (per il robot), già collegato a driver, rt e protocolli. Es.: make build/host/pulizia build/arm/pulizia
TASK_LIB = $(DRIVER_SRC) $(RT_SRC) $(PROTO_SRC)
TASK_HDR = $(DRIVER_HDR) $(RT_HDR) $(PROTO_HDR)

build/host/%: tasks/%.c $(TASK_LIB) $(TASK_HDR)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -I. -o $@ $< $(TASK_LIB) -lm

build/arm/%: tasks/%.c $(TASK_LIB) $(TASK_HDR)
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -I. -o $@ $< $(TASK_LIB) -lm $(LDFLAGS_ARM)

# Un programma diviso in più file: la cartella tasks/<nome>/ con main.c e gli altri .c e .h (per esempio uno per
# task) diventa build/host/<nome> e build/arm/<nome>. Si compilano tutti i .c della cartella.
.SECONDEXPANSION:
build/host/%: tasks/%/main.c $$(wildcard tasks/$$*/*.c tasks/$$*/*.h) $(TASK_LIB) $(TASK_HDR)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -I. -o $@ $(wildcard tasks/$*/*.c) $(TASK_LIB) -lm

build/arm/%: tasks/%/main.c $$(wildcard tasks/$$*/*.c tasks/$$*/*.h) $(TASK_LIB) $(TASK_HDR)
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -I. -o $@ $(wildcard tasks/$*/*.c) $(TASK_LIB) -lm $(LDFLAGS_ARM)

# La spia (spia/): intercetta e registra il traffico con GD32 e lidar.
build/host/spia: $(SPIA_SRC) $(SPIA_HDR) $(RT_SRC) $(RT_HDR) $(PROTO_SRC) $(PROTO_HDR)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -o $@ $(SPIA_SRC) $(RT_SRC) $(PROTO_SRC)

build/arm/spia: $(SPIA_SRC) $(SPIA_HDR) $(RT_SRC) $(RT_HDR) $(PROTO_SRC) $(PROTO_HDR)
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -o $@ $(SPIA_SRC) $(RT_SRC) $(PROTO_SRC) $(LDFLAGS_ARM)

build/host/rtes-driver: driver/rtes-driver.c $(DRIVER_SRC) $(DRIVER_HDR) $(RT_SRC) $(RT_HDR) $(PROTO_SRC) $(PROTO_HDR)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -o $@ driver/rtes-driver.c $(DRIVER_SRC) $(RT_SRC) $(PROTO_SRC) -lm

build/arm/rtes-driver: driver/rtes-driver.c $(DRIVER_SRC) $(DRIVER_HDR) $(RT_SRC) $(RT_HDR) $(PROTO_SRC) $(PROTO_HDR)
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -o $@ driver/rtes-driver.c $(DRIVER_SRC) $(RT_SRC) $(PROTO_SRC) -lm $(LDFLAGS_ARM)

build/host/sim: $(SIM_SRC) $(SIM_HDR) $(PROTO_SRC) $(PROTO_HDR)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -o $@ $(SIM_SRC) $(PROTO_SRC) -lm

# Lancia un programma dei task e ne legge la griglia di navigazione per la pagina del simulatore (sim/osserva.c).
build/host/osserva: sim/osserva.c
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -o $@ sim/osserva.c

# Ogni test si collega a tutte le librerie (sono piccole): protocolli, rt, driver (senza il thread) e simulatore.
TEST_LIB = $(RT_SRC) $(PROTO_SRC) spia/registro.c driver/scambio.c driver/batteria.c driver/lidar_giro.c driver/contatti.c $(SIM_LIB)
TEST_HDR = $(RT_HDR) $(PROTO_HDR) $(SPIA_HDR) $(DRIVER_HDR) $(SIM_HDR) test/test.h

build/host/test_%: test/test_%.c $(TEST_LIB) $(TEST_HDR)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -o $@ $< $(TEST_LIB) -lm

build/arm/test_%: test/test_%.c $(TEST_LIB) $(TEST_HDR)
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -o $@ $< $(TEST_LIB) -lm $(LDFLAGS_ARM)

# Esegue tutti i test sul PC.
# Poi le prove complete della spia con periferiche e socket finti (tools/prova-spia*.py, ~3 s),
# del banco di prova "spia guida" con un GD32 finto (tools/prova-guida.py, ~35 s)
# del driver contro il simulatore (tools/prova-driver.py, ~60 s) e del simulatore contro una registrazione del robot vero
# (tools/valida-sim.py c2, ~30 s; saltata se le registrazioni non ci sono).
test: $(addprefix build/host/,$(TEST)) build/host/spia build/host/sim build/host/rtes-driver
	@set -e; for t in $(addprefix build/host/,$(TEST)); do echo "== $$t"; ./$$t; done
	@echo "== tools/prova-spia.py"; python3 tools/prova-spia.py build/host/spia 2 | tail -1
	@echo "== tools/prova-spia-socket.py"; python3 tools/prova-spia-socket.py build/host/spia | tail -1
	@echo "== tools/prova-guida.py"; python3 tools/prova-guida.py build/host/spia | tail -1
	@echo "== tools/prova-driver.py"; python3 tools/prova-driver.py build/host | tail -1
	@echo "== tools/valida-sim.py c2"; python3 tools/valida-sim.py c2 | tail -1

# Esegue i test compilati per ARM sul PC tramite l'emulatore qemu-arm-static.
# Verifica che il binario del robot funzioni, ma i tempi non sono significativi.
test-arm: $(addprefix build/arm/,$(TEST))
	@set -e; for t in $^; do echo "== $$t (qemu)"; qemu-arm-static ./$$t; done

clean:
	rm -rf build
