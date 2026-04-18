CC      = gcc
CFLAGS  = -O2 -Wall -I. -std=c11
LD_COMMON = -lm -lpthread
LD_CURL   = $(LD_COMMON) -lcurl

SRCS_COMMON = tv_controller_2_1.c tv_controller_2_1_data.c

# ── Targets ────────────────────────────────────────────────────────────────
# tv_main       — Prop A2 degree commands (1 deg resolution, round to nearest)
# tv_main_propa       — Prop A percent commands (0.9 deg effective resolution)
# tv_main_propa_nodaq — Prop A percent commands + PT sensor input over CAN

all: tv_main tv_main_propa tv_main_propa_nodaq

tv_main: tv_main.c $(SRCS_COMMON)
	$(CC) $(CFLAGS) -o $@ $^ $(LD_CURL)

tv_main_propa: tv_main_propa.c $(SRCS_COMMON)
	$(CC) $(CFLAGS) -o $@ $^ $(LD_CURL)

tv_main_propa_nodaq: tv_main_propa_nodaq.c $(SRCS_COMMON)
	$(CC) $(CFLAGS) -o $@ $^ $(LD_COMMON)

# ── Mock stack helpers ──────────────────────────────────────────────────────
vcan:
	sudo modprobe vcan
	sudo ip link add dev can1 type vcan 2>/dev/null || true
	sudo ip link set up can1

mock:
	python3 mock_can_bridge.py &
	python3 mock_pt.py &

# ── Run targets ─────────────────────────────────────────────────────────────
run: tv_main
	sudo ./tv_main

run-propa: tv_main_propa
	sudo ./tv_main_propa

run-propa-nodaq: tv_main_propa_nodaq
	sudo ./tv_main_propa_nodaq

clean:
	rm -f tv_main tv_main_propa tv_main_propa_nodaq
