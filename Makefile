LIBPLDM_SRC := /home/hhlee/libpldm
LIBPLDM_BUILD := $(LIBPLDM_SRC)/build

CFLAGS := -Wall -Wextra -O0 -g \
          -I$(LIBPLDM_SRC)/include \
          -I$(LIBPLDM_BUILD)/include
LDFLAGS := -L$(LIBPLDM_BUILD)/src -lpldm \
           -Wl,-rpath,$(LIBPLDM_BUILD)/src

BINS := responder requester

all: $(BINS)

responder: responder.c
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

requester: requester.c
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

clean:
	rm -f $(BINS)

.PHONY: all clean
