# esp32_vagrant_test
project to test the esp32 workbench using vagrant and visual studio code with remote ssh and espressif extension 

## Purpose
Create an environment for embedded development with the espressif esp32 board using Visual Studio Code (VSCode) on a remote ssh connection into an ubuntu (20.04LTS) based virtual box hosted via 
vagrant. Currently only linux-based host machines (implemented on pop-os 20.04) are supported. 


# Installation 
1. Install VirtualBox 
2. Install Vagrant `sudo apt install vagrant`
3. Install Visual Studio Code. e.g.`sudo snap install --classic code`
X. Run `./install_usb_host.sh` to enable usb port forwarding to the guest machine
X. Run `vagrant up` (first time will take some time. grab a coffee) 


# Setting up the workbench 
1. In Visual Studio Code install remote ssh extension
2. When installed, press F1 and enter `>Remote-SSH: Open Configuration File...`
3. Open a Terminal in the project root folder and enter `vagrant ssh-config` 
4. Copy the returned text (usually beginning with "Host default") and paste into the Configuration file in VSCode (you might want to rename "default" to "Vagrant")
5. Save the file 
6. You can now start a remote session in the vagrant box with pressing F1 and entering `>Remote-SSH: Connect to Host...` choosing default (or Vagrant if you renamed it) from the list 
7. In the remote session, install the "Espressif-IDF" extension 
8. Configure the extension by pressing F1 and entering `ESP-IDF: Configure ESP-IDF extension` 
9. Choose "Express" since prerequisites and the tools are already installed in the vagrant box 
10. From "Select ESP-IDF version:" choose: "Find ESP-IDF in your system" 
11. Enter ESP-IDF directory: `/home/vagrant/esp-idf`
12. Select Python version: `/usr/bin/python` 
13. Hit "Install" 

You're ready to go. You can try out by copying the hello-world project from the examples: 

- In VSCode, press F1 and enter `ESP-IDF: Show Example Projects` (Use current ESP-IDF (/home/vagrant/esp-idf)
- Choose the hello-world project from the get-started category and press "Create project using example hello_world" 
- You might, have to set the appropriate port of your esp32 board (e.g. /dev/ttyUSB0). In VSCode, press F1 and enter `ESP-IDF: Select port to use`. 
- To build, flash and monitor press F1 and enter `ESP-IDF: Build, Flash and start a monitor on your device`




