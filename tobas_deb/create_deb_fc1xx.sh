#!/bin/bash

set -e

TOBAS_DEB=$(realpath $(dirname "$0"))
WORKSPACE=$(realpath ${TOBAS_DEB}/fc1xx)
fakeroot dpkg-deb --build ${WORKSPACE} ${TOBAS_DEB}
