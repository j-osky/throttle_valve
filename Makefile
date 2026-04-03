CC=gcc
CFLAGS=-O2 -Wall -I. -std=c11
LDFLAGS=-lm -lpthread -lcurl
TARGET=tv_main
SRCS=tv_main.c tv_controller_2_1.c tv_controller_2_1_data.c

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDFLAGS)

vcan:
	sudo modprobe vcan
	sudo ip link add dev can0 type vcan 2>/dev/null || true
	sudo ip link set up can0

mock:
	python3 mock_daq.py &
	sleep 1
	python3 mock_can_bridge.py &

run: $(TARGET)
	sudo ./$(TARGET)

clean:
	rm -f $(TARGET)
