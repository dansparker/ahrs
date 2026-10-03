CC ?= gcc
CFLAGS = -std=c11 -O2 -g -Wall -Wextra -Werror -Wshadow -fsanitize=address,undefined -fno-omit-frame-pointer
SRC = test_main.c test_units.c test_sim.c \
      ../core/eskf.c ../core/magcal.c ../core/ahrs.c ../core/ubx.c ../core/canas.c ../core/can_out.c ../core/baro.c

all: run
ahrs_tests: $(SRC) $(wildcard ../core/*.h) test.h
	$(CC) $(CFLAGS) $(SRC) -lm -o $@
run: ahrs_tests
	./ahrs_tests
clean:
	rm -f ahrs_tests
.PHONY: all run clean
