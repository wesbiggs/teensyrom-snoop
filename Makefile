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

# the SID emulator in the glue, against reSID 0.16 (GPL: fetched, never committed). Skipped without
# build/resid, which `make resid` fetches.
resid:
	$(PYTHON) tools/fetch_resid.py --dest build/resid

RESID_SRC = $(wildcard build/resid/*.cpp)
RESID_OBJ = $(patsubst build/resid/%.cpp,build/resid-obj/%.o,$(RESID_SRC))
build/resid-obj/%.o: build/resid/%.cpp
	@mkdir -p build/resid-obj
	$(CXX) -std=gnu++17 -O2 -w -c $< -o $@

ifeq ($(RESID_SRC),)
test-sid:
	@echo "test-sid: skipped (no build/resid: make resid)"
else
test-sid: $(RESID_OBJ) | build/inc/snoop
	$(CXX) -std=gnu++17 -O2 -Wall -Wextra -Werror -Ibuild/inc -isystem build/resid -DSNOOP_RESID_H='"sid.h"' \
	  -o build/fw_host_sid tests/fw_host/main.cpp $(RESID_OBJ)
	./build/fw_host_sid
	$(PYTHON) -m unittest -v tests/test_fw.py
endif

test-core: build/libsnoop.so
	$(PYTHON) -m unittest -v tests/test_core.py

# the reSIDfp player (skipped without pip install pyresidfp numpy)
test-player:
	$(PYTHON) -m unittest -v tests/test_sidplay.py

# real tunes end to end (skipped without py65 + pyresidfp + a TeensyROM checkout: TEENSYROM=...)
TEENSYROM ?= ../../3p/TeensyROM
test-e2e: build/libsnoop.so
	TEENSYROM=$(TEENSYROM) $(PYTHON) -m unittest -v tests/test_e2e.py

test: check-cxx test-core test-fw test-sid test-player test-e2e

# the live path in real time against tools/fake_tr.py (timing-dependent: not in make test)
test-live: demo
	$(PYTHON) tests/live_tcp.py

# WAVs of a few of the TeensyROM's bundled tunes, through capture -> wire -> reSIDfp (no hardware)
DEMO_TUNES ?= Aces_High Echoes Switch_625 Popcorn
demo: build/libsnoop.so
	mkdir -p build/demo
	for t in $(DEMO_TUNES); do \
	  $(PYTHON) tools/sidtrace.py $(TEENSYROM)/Source/Teensy/TRMenuFiles/SIDs/$$t.sid.h --seconds 30 --gap 0 --snoop build/demo/$$t.snoop && \
	  $(PYTHON) host/trsid.py --replay build/demo/$$t.snoop --wav build/demo/$$t.wav; done

clean:
	rm -rf build

.PHONY: all resid test-sid check-cxx test-core test-fw test-player test-e2e test test-live demo clean
