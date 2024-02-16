#!/bin/bash
#Install on raspberry

# fra https://imaginghub.com/projects/144-installing-opencv-3-on-raspberry-pi-3?gclid=Cj0KCQiAj4biBRC-ARIsAA4WaFjQHDPMrUrej-gzweV4YMatExH7ighI44Ksx1T4AoMV1WzlxvkiRkgaAnlUEALw_wcB#documentation

# remove existing version
# sudo apt remove libopencv-dev


#sudo apt-get install build-essential cmake pkg-config
sudo apt -y install cmake
sudo apt -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt -y install libxvidcore-dev libx264-dev
sudo apt -y install libgtk2.0-dev
sudo apt -y install libatlas-base-dev gfortran
#sudo apt -y install python2.7-dev python3-dev
sudo apt -y install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt -y install htpdate

# Set time (only relevant if on an internal DTU net, where time server is blocked
htpdate -q www.linux.org www.freebsd.org 
 
# maybe needed by cmake to find video4l package
# - failed to see effect
#cd /usr/include/linux
#sudo ln -s -f ../libv4l1-videodev.h videodev.h

cd
mkdir -p git
cd git
wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.4.zip
unzip opencv.zip
wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.4.zip
unzip opencv_contrib.zip

pip install numpy
# install opencv and contributions to opencv
cd ~/git/opencv-3.4/
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/git/opencv_contrib-3.4/modules \
    -D BUILD_EXAMPLES=OFF \
    -D WITH_V4L=ON ..
make -j2
sudo make install
sudo ldconfig

# get raspicam - unpack later
cd ~/git
wget --no-check-certificate -O raspicam-0.1.6.zip https://downloads.sourceforge.net/project/raspicam/raspicam-0.1.6.zip?r=https%3A%2F%2Fsourceforge.net%2Fprojects%2Fraspicam%2F%3Fsource%3Dtyp_redirect&ts=1486483484&use_mirror=netix


# install other stuff
sudo apt -y install subversion
sudo apt -y install libreadline-dev
sudo apt -y install dnsmasq
# sound - music and speak
sudo apt -y install espeak
sudo apt -y install sox libsox-fmt-all
# 
# unpack and build raspicam
unzip raspicam-0.1.6.zip
cd raspicam-0.1.6
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig

cd
mkdir -p Music
cd Music
ln -s radetzky-marsch_Schloss-Schoenbrunn-Konzerte_Wien_full-length.mp3 music.mp3

cd
ln -s regbot/mission
ln -s regbot/robobot_bridge

cd robobot_bridge
mkdir -p build
cd build
cmake ..
make
sudo make install

cd mission
mkdir -p build
cd build
cmake ..
make

echo "export CMAKE_PREFIX_PATH=/usr/local/lib" >> ~/.bashrc

echo "--------------------"
echo "almost finished: ..."
echo "1. Modify /etc/rc.local - add 'cd /home/local/robobot_bridge/build && ./robobot_bridge -a &'"
echo "2. Modify /etc/dhcpcd.conf as in http://rsewiki.elektro.dtu.dk/index.php/Full_installation_instructions"
echo "3. Modifi /etc/dnsmasq.conf as in the same page as above."
echo "4. Copy radetzky to Music"
echo "5. reboot and test"
