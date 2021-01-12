#!/usr/bin/env bash

# update system
sudo apt-get update
sudo apt-get upgrade -y

# setup ttyusb
# https://askubuntu.com/questions/935350/ubuntu-16-04-1-usbserial-missing/984031
sudo apt-get install -y linux-image-extra-virtual wget unzip tar

# ... and giving permissions
sudo usermod -a -G dialout $USER

# get prerequisites
sudo apt-get install -y git wget flex bison gperf python3 python3-pip python3-setuptools cmake ninja-build ccache libffi-dev libssl-dev dfu-util

# set python3 as default interpreter
sudo apt-get install -y python3 python3-pip python3-setuptools
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10

# clone the repository
cd /home/vagrant
git clone -b v4.2 --recursive https://github.com/espressif/esp-idf.git

# install the tools
cd /home/vagrant/esp-idf
./install.sh

mkdir /home/vagrant/esp

sudo reboot now

echo "ESP-IDF installed you can now use ssh remote and espressif extension in visual code!"



