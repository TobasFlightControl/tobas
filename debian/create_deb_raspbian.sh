#!/bin/bash

set -e

pardir=$(realpath $(dirname "$0"))
workspace=$(realpath ${pardir}/raspbian)
fakeroot dpkg-deb --build ${workspace} ${pardir}
