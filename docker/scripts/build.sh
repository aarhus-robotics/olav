#!/bin/bash

set -e

PROGRESS=plain
VERSION=$(git rev-parse --short HEAD)
BUILD_TYPE=RelWithDebInfo

cd "$(dirname "${BASH_SOURCE[0]}")"./../

docker build --progress ${PROGRESS} \
             --build-arg TARGET=base \
             --build-arg BUILD_TYPE=${BUILD_TYPE} \
             --tag ghcr.io/aarhus-robotics/olav:devel \
             --tag ghcr.io/aarhus-robotics/olav:devel-${VERSION} \
             --tag ghcr.io/aarhus-robotics/olav:devel-stable \
             --file docker/build/devel/Dockerfile \
            .
