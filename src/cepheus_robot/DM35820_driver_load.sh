#!/bin/bash          
#
# Script to load RTD DM35820 encoder and IO card drivers module
# 
# Ilias Patsiaouras
# Mo 3 Nov 2015 
#
# 
#

printf "\033[1;34m \nDM35820 IO card drivers loading script started\033[0m\n"
make -C $(rospack find cepheus_robot)/DM7820*/driver unload
make -C $(rospack find cepheus_robot)/DM7820*/driver load
printf "\033[1;34m DM35820 IO card drivers loading script ended\033[0m\n\n"
