#!/bin/bash

set -e

TOBAS_DEB=$(realpath $(dirname "$0"))
WORKSPACE=$(realpath ${TOBAS_DEB}/raspbian)
fakeroot dpkg-deb --build ${WORKSPACE} ${TOBAS_DEB}
