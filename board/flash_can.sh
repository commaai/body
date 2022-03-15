#!/usr/bin/env sh
set -e

scons -u

./canloader.py obj/body.bin.signed
