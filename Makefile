# libpldm and libmctp are vendored as git submodules (./libpldm, ./libmctp).
# One-time setup (checkout + install meson + build both libs):
#   git submodule update --init
#   ./lib_build.sh
# Then build the loopback binaries:
#   make
#
# To build against an external checkout instead of the submodule, override:
#   make LIBPLDM_SRC=/path/to/libpldm LIBMCTP_SRC=/path/to/libmctp
LIBPLDM_SRC ?= libpldm
LIBPLDM_BUILD ?= $(LIBPLDM_SRC)/build

LIBMCTP_SRC ?= libmctp
LIBMCTP_BUILD ?= $(LIBMCTP_SRC)/build-meson

CFLAGS := -Wall -Wextra -O0 -g \
          -I$(LIBPLDM_SRC)/include \
          -I$(LIBPLDM_BUILD)/include
LDFLAGS := -L$(LIBPLDM_BUILD)/src -lpldm \
           -Wl,-rpath,$(LIBPLDM_BUILD)/src

MCTP_CFLAGS := -I$(LIBMCTP_SRC) -I$(LIBMCTP_BUILD)
MCTP_LDFLAGS := -L$(LIBMCTP_BUILD) -lmctp \
                -Wl,-rpath,$(LIBMCTP_BUILD)

BINS := mctp_i2c_send mctp_validator pldm_validator pldm_type2_validator

all: check $(BINS)

# The submodule libraries are built by ./lib_build.sh, not here. Just verify
# they're present and point at the setup steps otherwise.
check:
	@test -f $(LIBPLDM_SRC)/include/libpldm/base.h \
	    || { echo "error: submodules not checked out"; \
	         echo "       run: git submodule update --init"; exit 1; }
	@test -f $(LIBPLDM_BUILD)/src/libpldm.so && test -f $(LIBMCTP_BUILD)/libmctp.so \
	    || { echo "error: submodule libraries not built"; \
	         echo "       run: ./lib_build.sh"; exit 1; }

mctp_i2c_send: mctp_i2c_send.c
	$(CC) $(CFLAGS) $(MCTP_CFLAGS) $< -o $@ $(LDFLAGS) $(MCTP_LDFLAGS)

# Pure MCTP control protocol -- no libpldm dependency.
mctp_validator: mctp_validator.c
	$(CC) $(CFLAGS) $(MCTP_CFLAGS) $< -o $@ $(MCTP_LDFLAGS)

# PLDM base protocol over MCTP -- needs both libpldm and libmctp.
pldm_validator: pldm_validator.c
	$(CC) $(CFLAGS) $(MCTP_CFLAGS) $< -o $@ $(LDFLAGS) $(MCTP_LDFLAGS)

# PLDM Type 2 (platform monitoring & control) over MCTP -- both libs.
pldm_type2_validator: pldm_type2_validator.c
	$(CC) $(CFLAGS) $(MCTP_CFLAGS) $< -o $@ $(LDFLAGS) $(MCTP_LDFLAGS)

clean:
	rm -f $(BINS)

.PHONY: all check clean
