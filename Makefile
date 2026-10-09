# netcam-app-lab -- build, test and end-to-end simulation
#
# Requirements: a C99 compiler.  Nothing else.  No subdirectories are
# created by make itself: the object directory is the only one, and the
# results/ directory used by the simulation is created by the C code
# (ntc_mkdir), because make.exe on Windows has no portable mkdir -p.
#
# Targets:
#   make        build build/sim and build/test
#   make test   build and run the self-checking test suite
#   make sim    build and run the end-to-end simulation (writes results/)
#   make clean  remove build artifacts and results/

CC      ?= cc
CSTD    ?= -std=c99
WARN    ?= -Wall -Wextra -Wpedantic -Werror
OPT     ?= -O2
# -ffp-contract=off keeps FMA formation off, so floating point results are
# bit-identical on macOS clang / Linux gcc / Windows g++.  Without it a
# double assertion can pass on one host and fail on another.
FPFLAGS ?= -ffp-contract=off
CFLAGS  ?= $(CSTD) $(WARN) $(OPT) $(FPFLAGS)
CPPFLAGS ?= -Isrc
LDFLAGS ?=
LDLIBS  ?= -lm

ifeq ($(OS),Windows_NT)
EXE   := .exe
MKDIR := mkdir
else
EXE   :=
MKDIR := mkdir -p
endif

# The object directory is created at parse time.  GNU make 3.80 (which ships
# with some Windows toolchains) mishandles an order-only prerequisite on a
# pattern rule -- it reports "Circular ... dependency dropped" and then runs
# the compile with no directory present -- so we do not rely on that.  Note
# the deliberately plain "mkdir": redirections are not portable across the
# shells make may use on Windows.
BUILD := build
ifeq ($(wildcard $(BUILD)),)
$(shell $(MKDIR) $(BUILD))
endif

# A C++ frontend (g++/clang++) can compile C, but it needs to be told to, and
# it rejects -std=c99 otherwise.  Detect it and switch the language mode.
ifeq ($(findstring ++,$(notdir $(CC))),++)
CMODE := -x c
else
CMODE :=
endif

BUILD   := build
SRCDIR  := src
TESTDIR := test
LIB_SRCS  := $(wildcard $(SRCDIR)/*.c)
LIB_SRCS  := $(filter-out $(SRCDIR)/main_sim.c,$(LIB_SRCS))
SIM_SRCS  := $(SRCDIR)/main_sim.c
TEST_SRCS := $(wildcard $(TESTDIR)/*.c)

LIB_OBJS  := $(patsubst $(SRCDIR)/%.c,$(BUILD)/%.o,$(LIB_SRCS))
SIM_OBJS  := $(patsubst $(SRCDIR)/%.c,$(BUILD)/%.o,$(SIM_SRCS))
TEST_OBJS := $(patsubst $(TESTDIR)/%.c,$(BUILD)/%.o,$(TEST_SRCS))

SIM  := $(BUILD)/sim$(EXE)
TEST := $(BUILD)/test_netcam$(EXE)

.PHONY: all build test sim clean

all: build

build: $(SIM) $(TEST)

$(BUILD)/%.o: $(SRCDIR)/%.c src/netcam.h
	$(CC) $(CMODE) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: $(TESTDIR)/%.c src/netcam.h
	$(CC) $(CMODE) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(SIM): $(LIB_OBJS) $(SIM_OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(TEST): $(LIB_OBJS) $(TEST_OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

test: $(TEST)
	$(TEST)

sim: $(SIM)
	$(SIM)

clean:
	-rm -rf $(BUILD)
	-rm -rf results
