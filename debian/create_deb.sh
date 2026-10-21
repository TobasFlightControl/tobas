#!/bin/bash

set -e

pardir=$(realpath $(dirname "$0"))
bash ${pardir}/create_deb_ubuntu.sh
bash ${pardir}/create_deb_fc1xx.sh
