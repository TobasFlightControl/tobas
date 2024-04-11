#!/bin/bash

# cf. https://solarianprogrammer.com/2017/12/08/raspberry-pi-raspbian-install-gcc-compile-cpp-17-programs/
# 異なるバージョンでもほぼ同じ
# 削除コマンド: $ sudo rm -rf /opt/gcc-9.1.0

sudo apt update
sudo apt upgrade -y
sudo apt install git -y

git clone https://bitbucket.org/sol_prog/raspberry-pi-gcc-binary.git
cd raspberry-pi-gcc-binary
tar -xjvf gcc-9.1.0-armhf-raspbian.tar.bz2
sudo mv gcc-9.1.0 /opt
cd ..
rm -rf raspberry-pi-gcc-binary

# echo 'export PATH=/opt/gcc-9.1.0/bin:$PATH' >> ~/.bashrc
# echo 'export LD_LIBRARY_PATH=/opt/gcc-9.1.0/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
# exec bash

sudo ln -s /usr/include/arm-linux-gnueabihf/sys /usr/include/sys
sudo ln -s /usr/include/arm-linux-gnueabihf/bits /usr/include/bits
sudo ln -s /usr/include/arm-linux-gnueabihf/gnu /usr/include/gnu
sudo ln -s /usr/include/arm-linux-gnueabihf/asm /usr/include/asm
sudo ln -s /usr/lib/arm-linux-gnueabihf/crti.o /usr/lib/crti.o
sudo ln -s /usr/lib/arm-linux-gnueabihf/crt1.o /usr/lib/crt1.o
sudo ln -s /usr/lib/arm-linux-gnueabihf/crtn.o /usr/lib/crtn.o

# sudo ln -s /opt/gcc-9.1.0/bin/gcc-9.1 /usr/bin/gcc
