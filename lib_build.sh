#!/bin/bash

sudo apt install pipx
pipx install meson
pipx ensurepath
cd libpldm && meson setup build && meson compile -C build
meson test -C build

cd ../libmctp
meson setup build-meson
meson compile -C build-meson