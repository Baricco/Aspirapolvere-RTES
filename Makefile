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
RT_HDR  = rt/rt.h rt/rt_time.h
GD32_SRC = gd32/gd32.c gd32/interlock.c gd32/description.c
GD32_HDR = gd32/gd32.h gd32/interlock.h gd32/description.h
LIDAR_SRC = lidar/lidar.c
LIDAR_HDR = lidar/lidar.h
# Librerie dei protocolli: ogni test e ogni strumento le compila tutte (sono piccole).
PROTO_SRC = $(GD32_SRC) $(LIDAR_SRC)
PROTO_HDR = $(GD32_HDR) $(LIDAR_HDR)

PROGRAMMI = latenza carico
STRUMENTI = spia rtes-driver
TEST      = test_rt_tempo test_rt_task test_gd32 test_sicurezza test_lidar test_registro test_driver_logica test_sim test_descrizione test_nav_command test_live_map test_app_commands

SPIA_SRC = spia/spia.c spia/registro.c spia/everest.c
SPIA_HDR = spia/registro.h spia/everest.h

# Driver (driver/): il nostro sostituto di AuxCtrl. rtes-driver è il programma per provarlo da solo.
DRIVER_SRC = driver/driver.c driver/exchange.c driver/battery.c driver/lidar_scan.c driver/contacts.c driver/docking.c spia/registro.c
DRIVER_HDR = driver/driver.h driver/exchange.h driver/battery.h driver/lidar_scan.h driver/contacts.h driver/docking.h spia/registro.h

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

# Il programma del robot (tasks/vacuum/): oltre ai file dei task ha la cartella cloud/ (collegamento al server dell'app,
# docs/app.md sezione 9), cJSON e libcurl. Sul PC usa la libcurl del sistema; per il robot quella statica con mbedTLS
# compilata da tools/build-net-libs.sh in build/arm/netlibs/. Queste due regole vincono su quelle generiche qui sopra.
VACUUM_SRC = $(wildcard tasks/vacuum/*.c tasks/vacuum/cloud/*.c) third_party/cjson/cJSON.c
VACUUM_HDR = $(wildcard tasks/vacuum/*.h tasks/vacuum/cloud/*.h) third_party/cjson/cJSON.h
CLOUD_CONFIG = tasks/vacuum/cloud/config.h
NETLIBS = build/arm/netlibs

build/host/vacuum: $(VACUUM_SRC) $(VACUUM_HDR) $(TASK_LIB) $(TASK_HDR) | $(CLOUD_CONFIG)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -I. -o $@ $(VACUUM_SRC) $(TASK_LIB) -lm -lcurl

build/arm/vacuum: $(VACUUM_SRC) $(VACUUM_HDR) $(TASK_LIB) $(TASK_HDR) $(NETLIBS)/lib/libcurl.a | $(CLOUD_CONFIG)
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -I. -I$(NETLIBS)/include -o $@ $(VACUUM_SRC) $(TASK_LIB) -lm \
		-L$(NETLIBS)/lib -lcurl -lmbedtls -lmbedx509 -lmbedcrypto $(LDFLAGS_ARM)

# la configurazione privata robot-server non è versionata: va copiata dall'esempio
$(CLOUD_CONFIG):
	@echo "manca $(CLOUD_CONFIG): copialo da tasks/vacuum/cloud/config.example.h e metti i valori veri (git lo ignora)" >&2; exit 1

$(NETLIBS)/lib/libcurl.a:
	@echo "mancano le librerie di rete per il robot: lancia tools/build-net-libs.sh (scarica e compila mbedTLS e libcurl)" >&2; exit 1

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

# Sul robot: legge da fuori stato, posizione e mappa del programma dei task (tools/driver-robot.sh missione).
build/arm/osserva-robot: tools/osserva-robot.c
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -o $@ tools/osserva-robot.c $(LDFLAGS_ARM)

# Lancia un programma dei task e ne legge la griglia di navigazione per la pagina del simulatore (sim/osserva.c).
build/host/osserva: sim/osserva.c
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -o $@ sim/osserva.c

# Ogni test si collega a tutte le librerie (sono piccole): protocolli, rt, driver (senza il thread) e simulatore.
TEST_LIB = $(RT_SRC) $(PROTO_SRC) spia/registro.c driver/exchange.c driver/battery.c driver/lidar_scan.c driver/contacts.c driver/docking.c $(SIM_LIB)
TEST_HDR = $(RT_HDR) $(PROTO_HDR) $(SPIA_HDR) $(DRIVER_HDR) $(SIM_HDR) test/test.h

build/host/test_%: test/test_%.c $(TEST_LIB) $(TEST_HDR)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -o $@ $< $(TEST_LIB) -lm

build/arm/test_%: test/test_%.c $(TEST_LIB) $(TEST_HDR)
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -o $@ $< $(TEST_LIB) -lm $(LDFLAGS_ARM)

# Test della parte dell'app (docs/app.md, sezione 11): si collegano ai file veri di tasks/vacuum che provano, con il
# resto del programma (task, driver, rete) sostituito da funzioni finte dentro il test.
TEST_APP = test_nav_command test_live_map test_app_commands
TEST_LIVE_MAP_SRC = tasks/vacuum/cloud/live_map.c tasks/vacuum/aperiodic.c
TEST_APP_COMMANDS_SRC = tasks/vacuum/cloud/app_commands.c third_party/cjson/cJSON.c

build/host/test_live_map: test/test_live_map.c $(TEST_LIVE_MAP_SRC) $(VACUUM_HDR) $(TEST_LIB) $(TEST_HDR) | $(CLOUD_CONFIG)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -I. -o $@ $< $(TEST_LIVE_MAP_SRC) $(TEST_LIB) -lm

build/arm/test_live_map: test/test_live_map.c $(TEST_LIVE_MAP_SRC) $(VACUUM_HDR) $(TEST_LIB) $(TEST_HDR) | $(CLOUD_CONFIG)
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -I. -o $@ $< $(TEST_LIVE_MAP_SRC) $(TEST_LIB) -lm $(LDFLAGS_ARM)

build/host/test_app_commands: test/test_app_commands.c $(TEST_APP_COMMANDS_SRC) $(VACUUM_HDR) $(TEST_LIB) $(TEST_HDR) | $(CLOUD_CONFIG)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -I. -o $@ $< $(TEST_APP_COMMANDS_SRC) $(TEST_LIB) -lm

build/arm/test_app_commands: test/test_app_commands.c $(TEST_APP_COMMANDS_SRC) $(VACUUM_HDR) $(TEST_LIB) $(TEST_HDR) | $(CLOUD_CONFIG)
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -I. -o $@ $< $(TEST_APP_COMMANDS_SRC) $(TEST_LIB) -lm $(LDFLAGS_ARM)

build/host/test_nav_command: test/test_nav_command.c tasks/vacuum/navigation.h $(TEST_LIB) $(TEST_HDR)
	@mkdir -p $(@D)
	$(CC_HOST) $(CFLAGS_HOST) -I. -o $@ $< $(TEST_LIB) -lm

build/arm/test_nav_command: test/test_nav_command.c tasks/vacuum/navigation.h $(TEST_LIB) $(TEST_HDR)
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS_ARM) -I. -o $@ $< $(TEST_LIB) -lm $(LDFLAGS_ARM)

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
