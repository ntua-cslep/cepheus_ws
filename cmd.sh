sudo apt-get update
sudo apt-get upgrade
sudo apt autoremove
sudo apt-get dist-upgrade
cd Downloads/t4uh_drivers/
cd rtl8812AU_8821AU_linux/
sudo make clean
sudo make
sudo make install
sudo modprobe -a rtl8812au
cd
cd catkin_ws/
source devel/setup.bash 
sudo cp $(rospack find cepheus_robot)/DM7820_Linux_*/driver/rtd-dm7820.ko /lib/modules/$(uname -r)/kernel/drivers/
sudo depmod
cd
ls
cd catkin_ws/
ls
source devel/setup.bash 
ls /lib/modules/$(uname -r)/kernel/drivers/
echo 'rtd_dm7820.ko' | sudo tee -a /etc/modules
sudo depmod
cd src/cepheus_robot/DM7820_Linux_v03.00.00/
ls
cd driver/
ls
make unload
make clean
sudo make unload
sudo make clean
sudo make
sudo make load
ls
cd ..
ls
cd lib/
ls
make clean
make all
sudo make clean
sudo make all
cd
cd catkin_ws/src/cepheus_robot/DM7820_Linux_v03.00.00/
ls
cd driver/
ls
sudo make unload
sudo make clean
sudo make
sudo make load
sudo make unload
sudo make clean
sudo make
sudo make load
cd ../lib/
sudo make clean
sudo make
cd
su
reset
cd catkin_ws/
ls
source devel/setup.bash 
ls
cd src/
ls
cd cepheus_robot
cd DM7820_Linux_v03.00.00/
ls
cd driver/
ls
sudo make unload
sudo make clean
sudo make
sudo make load
sudo make unload
sudo make load
sudo cp rtd-dm7820.ko /lib/modules/$(uname -r)/kernel/drivers/
echo 'rtd_dm7820' | sudo tee -a /etc/modules
sudo depmod
cd ../lib/
sudo make clean
sudo make
sudo vim /etc/modules
