#!/bin/bash

set -e

tobas_deb=$(realpath $(dirname "$0"))
bash ${tobas_deb}/create_deb_ubuntu.sh
bash ${tobas_deb}/create_deb_fc1xx.sh
