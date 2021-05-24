#!/bin/bash
cd src/cepheus_robot/DM7820_Linux_v03.00.00/driver/
sudo make clean
sudo make unload
sudo make
sudo make load
cd ../../../..