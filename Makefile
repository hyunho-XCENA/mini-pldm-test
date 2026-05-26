# Override on the command line or via the environment:
#   make LIBPLDM_SRC=/path/to/libpldm
#   LIBPLDM_SRC=/path/to/libpldm make
LIBPLDM_SRC ?= ../libpldm
LIBPLDM_BUILD ?= $(LIBPLDM_SRC)/build

CFLAGS := -Wall -Wextra -O0 -g \
          -I$(LIBPLDM_SRC)/include \
          -I$(LIBPLDM_BUILD)/include
LDFLAGS := -L$(LIBPLDM_BUILD)/src -lpldm \
           -Wl,-rpath,$(LIBPLDM_BUILD)/src

BINS := responder requester daemon_responder

all: check-libpldm $(BINS)

check-libpldm:
	@test -f $(LIBPLDM_SRC)/include/libpldm/base.h \
	    || { echo "error: libpldm headers not found under LIBPLDM_SRC=$(LIBPLDM_SRC)"; \
	         echo "       set LIBPLDM_SRC to your libpldm source tree, e.g.:"; \
	         echo "         make LIBPLDM_SRC=/path/to/libpldm"; \
	         exit 1; }
	@test -f $(LIBPLDM_BUILD)/src/libpldm.so \
	    || { echo "error: libpldm.so not found under LIBPLDM_BUILD=$(LIBPLDM_BUILD)/src"; \
	         echo "       build libpldm first (meson setup build && meson compile -C build)"; \
	         exit 1; }

responder: responder.c
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

requester: requester.c
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

daemon_responder: daemon_responder.c
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

clean:
	rm -f $(BINS)

.PHONY: all clean check-libpldm
