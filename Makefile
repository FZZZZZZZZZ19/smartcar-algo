CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -g -Wall -Wextra -Wshadow            -Wno-unused-parameter -Iinclude
LDFLAGS ?=
LDLIBS  ?= -lm

SRC := $(wildcard src/*.c)
OBJ := $(patsubst src/%.c,build/%.o,$(SRC))

TEST_OBJ := build/test_main.o
SIM_OBJ  := build/sim_main.o
SIM2_OBJ := build/sim_compare.o
SIM_COMMON_OBJ := build/sim_common.o

.PHONY: all test sim compare plot clean run-test run-sim

all: test sim plot

build:
	@mkdir -p build out

build/%.o: src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/%.o: tests/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/%.o: sim/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/test_main: $(TEST_OBJ) $(OBJ)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)

build/sim_main: $(SIM_OBJ) $(SIM_COMMON_OBJ) $(OBJ)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)

build/sim_compare: $(SIM2_OBJ) $(SIM_COMMON_OBJ) $(OBJ)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)

test: build/test_main
	./build/test_main

sim: build/sim_main build/sim_compare
	./build/sim_main
	./build/sim_compare

compare: build/sim_compare
	./build/sim_compare

plot: sim
	python3 tools/plot_sim.py

run-test: test
run-sim: sim

clean:
	rm -rf build out/*.csv out/*.svg out/*.html out/*.png
