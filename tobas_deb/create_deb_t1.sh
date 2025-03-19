#!/bin/bash

TOBAS_DEB=$(realpath $(dirname "$0"))
WORKSPACE=$(realpath ${TOBAS_DEB}/t1)
fakeroot dpkg-deb --build ${WORKSPACE} ${TOBAS_DEB}
