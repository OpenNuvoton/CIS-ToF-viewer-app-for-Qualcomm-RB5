#!/bin/bash -
#
# Add "-e" Mean Exit Immediately If A Command Exits With A Non-Zero Status.
# Add "-v  Mean Prints Shell Input Lines As They Are Read.
# Add "-x  Mean Print Command xTraces Before Executing Command.
#

rm -rf build
mkdir build
cd build

cmake \
  ..

make -j$(nproc)
