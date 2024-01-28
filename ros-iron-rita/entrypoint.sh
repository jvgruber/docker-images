#!/bin/bash

set -e # enable error signals

cd /vol/dev_ws

echo "Provided arguments: $@" # print provided arguments

exec $@ # execute provided arguments