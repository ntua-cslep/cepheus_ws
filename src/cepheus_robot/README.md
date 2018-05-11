# DESCREPTION
A ROS package that include all the source code that runs on Space robot's PC.

#REQUIRMENTS
- RTD DM95820 card for acquisition of encoder and PWM's 
- 2 or 3 mouse's for localization

# Installation
## DM95820 Drivers
- Source your workspace to find the package

    ```source devel/setup.bash```

- Copy kernel module to the drivers directory.

    ```sudo cp $(rospack find cepheus_robot)/DM7820_Linux_*/driver/rtd-dm7820.ko /lib/modules/$(uname -r)/kernel/drivers/```

- Add the name of the module to the file /etc/modules. You can edit the file or just append to it as shown here.

    ```echo 'rtd_dm7820.ko' | sudo tee -a /etc/modules```

- Update the list of module dependencies.

    ```sudo depmod```

- Reboot the computer and voila, it worked.

- TIP: in case apt-get upgrade the kernel you have to move the driven to the new folder or just do this process again

## Mouse's
- In the launch file of the package modify the parameters of the mouse_odom node to correspond to the correct mouses like that:

    _/dev/input/by-path/ "correct mouse name"_

## Wifi Adapter T4UH 5G
- Just run the following commands in order to install. If you upgrade the kernel you must run the last command again.

	```mkdir ~/Downloads/t4uh_drivers```

	```cd ~/Downloads/t4uh_drivers```

	```sudo apt-get install git```

	```git clone https://github.com/abperiasamy/rtl8812AU_8821AU_linux.git```

	```cd rtl8812AU_8821AU_linux/```

	```sudo make clean```

	```sudo make```

	```sudo make install```
	
	```sudo modprobe -a 8812au```

#Instructions
## In every boot run the following commands to launch robot:     
- ```cd ~/catkin_ws```
- ```su```
- password: _root_
- ```soure devel/setup.bash```
- ```export ROS_MASTER_URI=http://cepheus.local:11311```
- ```export ROS_IP=192.168.1.165```