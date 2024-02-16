#!/bin/bash
# needed packages
sudo apt update
sudo apt dist-upgrade -y
sudo apt install -y ntp ntpdate libreadline-dev subversion cmake libopencv-dev htpdate python3-pyqtgraph python3-scipy pyqt5-dev pyqt5-dev-tools festival sox libsox-fmt-all
sudo apt autoremove -y
# set date from DTU server
sudo ntpdate -u ntp.ait.dtu.dk
# set SSID
#sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
# sound speech
# sudo apt install festival
echo "The brown fox jumps over lazy dog" > aaa.txt
text2wave aaa.txt -o aaa.wav
# play
#sudo apt-get install sox libsox-fmt-all
#
# mkdir
mkdir Downloads
mkdir Music
cd Music
ln -s radetzky-marsch_Schloss-Schoenbrunn-Konzerte_Wien_full-length.mp3 music.mp3
cd
# software
svn co svn://repos.gbar.dtu.dk/jcan/regbot 
ln -s regbot/mission 
ln -s regbot/robobot_bridge 
ln -s regbot/regbotgui
#
cd mission
mkdir -p build
cd build
cmake ..
make -j3
#
cd
cd robobot_bridge
mkdir -p build
cd build
cmake ..
make -j3
#
#
# set output device to jack
echo "defaults.pcm.card 0" >.asoundrc
echo "defaults.ctl.card 0" >>.asoundrc
# works after next login
#
# implement /etc/rc.local
cd
sudo mv /etc/rc.local /etc/rc.local.org
sudo cp regbot/robobot/doc/rc.local /etc/rc.local
#
