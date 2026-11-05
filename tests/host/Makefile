# Host-side unit tests for pure-C firmware modules (no ESP-IDF needed).
CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined
SRC = test_main.c ../../main/edge_util.c ../../main/mesh/mesh_radio.c ../../main/mesh/mesh_pb.c
INC = -I../../main -I../../main/mesh

build/test: $(SRC) Makefile
	@mkdir -p build
	$(CC) $(CFLAGS) $(INC) $(SRC) -lm -o $@

.PHONY: test clean
test: build/test
	./build/test

clean:
	rm -rf build
