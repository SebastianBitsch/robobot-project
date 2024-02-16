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


# maybe needed by cmake to find video4l package
# - failed to see effect
cd /usr/include/linux
sudo ln -s -f ../libv4l1-videodev.h videodev.h

cd
mkdir -p git
cd git
wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.4.zip
unzip opencv.zip
wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.4.zip
unzip opencv_contrib.zip

pip install numpy

cd ~/git/opencv-3.4/
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/git/opencv_contrib-3.4/modules \
    -D BUILD_EXAMPLES=ON ..
make -j2
sudo make install
sudo ldconfig
