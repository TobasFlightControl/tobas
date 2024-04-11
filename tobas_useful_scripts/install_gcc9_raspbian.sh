#!/bin/bash

sudo apt update && sudo apt upgrade -y

git clone https://bitbucket.org/sol_prog/raspberry-pi-gcc-binary.git
cd raspberry-pi-gcc-binary
tar -xjvf gcc-9.1.0-armhf-raspbian.tar.bz2
sudo mv gcc-9.1.0 /opt
cd ..
rm -rf raspberry-pi-gcc-binary

cd ~
echo 'export PATH=/opt/gcc-9.1.0/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/opt/gcc-9.1.0/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
. ~/.bashrc
sudo ln -s /usr/include/arm-linux-gnueabihf/sys /usr/include/sys
sudo ln -s /usr/include/arm-linux-gnueabihf/bits /usr/include/bits
sudo ln -s /usr/include/arm-linux-gnueabihf/gnu /usr/include/gnu
sudo ln -s /usr/include/arm-linux-gnueabihf/asm /usr/include/asm
sudo ln -s /usr/lib/arm-linux-gnueabihf/crti.o /usr/lib/crti.o
sudo ln -s /usr/lib/arm-linux-gnueabihf/crt1.o /usr/lib/crt1.o
sudo ln -s /usr/lib/arm-linux-gnueabihf/crtn.o /usr/lib/crtn.o

g++-9.1 -std=c++17 -Wall -pedantic test_fs.cpp -o test_fs
./test_fs
