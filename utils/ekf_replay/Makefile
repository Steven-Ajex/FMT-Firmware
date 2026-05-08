# Host build for the ekf_ins replay tool.
#
#   make                 build ./ekf_replay
#   make run             run a smoke test against the bundled scenario
#   make clean
#
# The tool feeds time-sorted CSV samples of every sensor bus into an
# instance of the ekf_ins library compiled for the host, and emits a
# CSV of the resulting INS_Out_Bus.  Pair with compare.py to diff the
# output against the recorded cf_ins INS_Out for the same flight.

LIB_DIR := ../../src/model/ins/ekf_ins/lib
SRC_LIB := $(wildcard $(LIB_DIR)/*.c)
SRC     := replay.c $(SRC_LIB)
OBJ     := $(SRC:.c=.o)

CFLAGS  := -O2 -std=c11 -Wall -Wextra -Wno-unused-parameter -I$(LIB_DIR) -I.
LDFLAGS := -lm

.PHONY: all run clean

all: ekf_replay

ekf_replay: $(SRC)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

run: ekf_replay
	./ekf_replay --selftest

clean:
	rm -f ekf_replay *.o $(LIB_DIR)/*.o
