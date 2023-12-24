#!/bin/bash

$ cd ~
$ git clone https://github.com/ArduPilot/ardupilot.git  # Anywhere
$ cd ardupilot
$ git checkout ArduCopter-stable
$ Tools/environment_install/install-prereqs-ubuntu.sh -y
$ . ~/.profile
$ git submodule update --init --recursive
$ ./waf configure --board sitl
$ ./waf copter
