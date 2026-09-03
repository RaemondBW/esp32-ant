# Host build of the ANT/ANT+ stack unit tests. No hardware, runs on the dev box.
CC      ?= cc
ROOT    := ../../components/ant
CFLAGS  := -std=c99 -Wall -Wextra -Werror -O2 -g \
           -I$(ROOT)/include
SRC     := $(ROOT)/src/ant_message.c \
           $(ROOT)/src/ant_channel.c \
           $(ROOT)/src/antplus_profiles.c \
           $(ROOT)/src/ant_phy_shockburst.c \
           $(ROOT)/src/ant_sb_link.c \
           $(ROOT)/src/ant_mac.c \
           $(ROOT)/src/ant_known.c \
           $(ROOT)/src/ant_air.c \
           $(ROOT)/src/ant_embedded.c \
           $(ROOT)/src/ant_selftest.c \
           $(ROOT)/radio/ant_radio_loopback.c
TESTSRC := test_main.c test_message.c test_profiles.c test_channel.c test_phy.c \
           test_link.c test_mac.c test_embedded.c test_known.c
BIN     := ant_tests

all: test

$(BIN): $(SRC) $(TESTSRC) test.h
	$(CC) $(CFLAGS) -o $(BIN) $(SRC) $(TESTSRC)

test: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)

.PHONY: all test clean
