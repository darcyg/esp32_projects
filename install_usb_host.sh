#!/usr/bin/env bash

sudo apt install virtualbox-ext-pack

echo "Adding host user to 'vboxusers' group to foward to guest..."
sudo usermod -a -G vboxusers $USER

echo "Reboot or Logout and back in to make changes effective!" 


