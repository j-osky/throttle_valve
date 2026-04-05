CC      = gcc
CFLAGS  = -O2 -Wall -I. -std=c11
LDFLAGS = -lm -lpthread -lcurl

SRCS_COMMON = tv_controller_2_1.c tv_controller_2_1_data.c

# ── Targets ────────────────────────────────────────────────────────────────
# tv_main       — Prop A2 degree commands (1 deg resolution, round to nearest)
# tv_main_propa — Prop A percent commands (0.9 deg effective resolution)

all: tv_main tv_main_propa

tv_main: tv_main.c $(SRCS_COMMON)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

tv_main_propa: tv_main_propa.c $(SRCS_COMMON)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# ── Mock stack helpers ──────────────────────────────────────────────────────
vcan:
	sudo modprobe vcan
	sudo ip link add dev can1 type vcan 2>/dev/null || true
	sudo ip link set up can1

mock:
	python3 mock_daq.py &
	sleep 1
	python3 mock_can_bridge.py &

# ── Run targets ─────────────────────────────────────────────────────────────
run: tv_main
	sudo ./tv_main

run-propa: tv_main_propa
	sudo ./tv_main_propa

clean:
	rm -f tv_main tv_main_propa
