CC      ?= cc
CXX     ?= c++
CFLAGS  ?= -O2 -g -std=c99 -Wall -Wextra -Werror -pedantic
PYTHON  ?= python3

all: build/libsnoop.so

build/libsnoop.so: core/snoop.c core/snoop.h tests/shim.c | build
	$(CC) $(CFLAGS) -fPIC -shared -o $@ core/snoop.c tests/shim.c

build:
	mkdir -p build

# the firmware compiles the core as C++
check-cxx: | build
	$(CXX) -std=gnu++17 -Wall -Wextra -Werror -x c++ -fsyntax-only core/snoop.c

# the firmware glue against host stubs (tests/fw_host)
build/inc/snoop: | build
	mkdir -p build/inc && ln -sfn ../../core build/inc/snoop

test-fw: | build/inc/snoop
	$(CXX) -std=gnu++17 -Wall -Wextra -Werror -Ibuild/inc -o build/fw_host tests/fw_host/main.cpp
	./build/fw_host
	$(PYTHON) -m unittest -v tests/test_fw.py

test-core: build/libsnoop.so
	$(PYTHON) -m unittest -v tests/test_core.py

test: check-cxx test-core test-fw

clean:
	rm -rf build

.PHONY: all check-cxx test-core test-fw test clean
