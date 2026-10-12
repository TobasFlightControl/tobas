#!/bin/bash

set -e

tobas_deb=$(realpath $(dirname "$0"))
workspace=$(realpath ${tobas_deb}/raspbian)
fakeroot dpkg-deb --build ${workspace} ${tobas_deb}
