
TOPDIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
SKETCH_FILE ?= sketch.cpp
SKETCH := $(abspath $(SKETCH_FILE))
BOARD ?= arduino:avr:mega
PORT ?= /dev/ttyACM1

.PHONY: all upload verify monitor

all: verify; @:

verify:
	arduino --verify --board "$(BOARD)" "$(SKETCH)"

upload:
	arduino --upload --port "$(PORT)" --board "$(BOARD)" "$(SKETCH)"

monitor:
	picocom -b 9600 "$(PORT)"
