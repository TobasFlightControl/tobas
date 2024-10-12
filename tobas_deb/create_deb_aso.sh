#!/bin/bash

TOBAS_DEB=$(realpath $(dirname "$0"))
ASO_WS=$(realpath ${TOBAS_DEB}/aso)
fakeroot dpkg-deb --build ${ASO_WS} ${TOBAS_DEB}
