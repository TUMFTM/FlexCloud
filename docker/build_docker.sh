#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

docker build -f $SCRIPT_DIR/Dockerfile -t flexcloud:latest $SCRIPT_DIR/..