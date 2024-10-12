#!/bin/bash

TOBAS_DEB=$(realpath $(dirname "$0"))
bash ${TOBAS_DEB}/create_deb_ubuntu.sh
bash ${TOBAS_DEB}/create_deb_aso.sh
