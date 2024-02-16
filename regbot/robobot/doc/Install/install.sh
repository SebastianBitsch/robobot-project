#!/bin/bash
# Script to setup robobot software
# Author: jbepe
# Date: 2022-01-18

### Installer must be run as root
# Check if being run as root:
if [ "$(whoami)" != "root" ]; then
	echo "You must run this install-script as root!"
	echo "Example: sudo ./install.sh"
	exit 1
fi

# Get current working dir
wrkdir=$(pwd)
echo "Working from: ${wrkdir}"

### Common functions:
# echoes in bold
# Use: bold "Text"
bold() {
	echo -e "\033[1m${1}\033[0m"
}

# Will prompt for a yes or no: [y/n]
# Use: prompt-yn "Prompt" y
prompt-yn() {
	local input_prompt=$1
	local default=$2
	local y="y" #The case for Y
	local n="n" #The case for N
	if [ "${default,,}" == "y" ]; then
		y="Y"
	elif [ "${default,,}" == "n" ]; then
		n="N"
	fi

	local status=2
	local response
	while [ $status -eq 2 ]; do
		read -p "$input_prompt [$y/$n]: " response

		if [ "${response,,}" == "" ]; then
			response=$default
		fi

		if [ "${response,,}" == "y" ]; then
			status=0
		elif [ "${response,,}" == "n" ]; then
			status=1	
		else
			echo "I did not understand that, please try again!"
			status=2
		fi
	done
	return $status
}

# Prompt for text
# Use: prompt-txt "Please enter" y output
prompt-txt() {
	local input_prompt=$1
	local default=$2
	local status=1
	local response
	while [ $status -eq 1 ]; do
		read -p "$input_prompt: " response

		prompt-yn "Is \"$response\" ok?" $default
		status=$?
	done

	eval "$3='$response'"
}

### Install Script
bold "Welcome to the install-script!"

# hostname
prompt-txt "Enter hostname for the robobot" y host
echo "$host" > /etc/hostname
sed -i "s/raspberrypi/${host}/" /etc/hosts

# Add local user
bold "Adding user local..."
# requires password and other settings manually
useradd -m -U -G adm,cdrom,sudo,audio,video,plugdev,games,users,netdev,input,spi,gpio,i2c,dialout -s /bin/bash local
read -s -p "Please enter new password for pi and local (will not be printed on screen): " pass
yes "${pass}" | passwd local
yes "${pass}" | passwd pi

# - setup of Robobot
echo "Installing updates and packages..."
apt update
apt dist-upgrade -y
apt autoremove -y

apt install -y ntp ntpdate libreadline-dev subversion cmake libopencv-dev htpdate python3-pyqtgraph python3-scipy pyqt5-dev pyqt5-dev-tools festival sox libsox-fmt-all samba
apt autoremove -y

# Adjust timezone to Copenhagen:
echo "Adjusting time zone..."
timezone="Europe/Copenhagen"
ln -snf /usr/share/zoneinfo/${timezone} /etc/localtime && echo "${timezone}" > /etc/timezone
# set date from DTU server
ntpdate -u ntp.ait.dtu.dk
# Generate DK locale
echo "Generating locale..."
sed -i 's/#en_DK.UTF-8 UTF-8/en_DK.UTF-8 UTF-8/' /etc/locale.gen
locale-gen
echo "LANG=en_DK.UTF-8" > /etc/locale.conf
echo "KEYMAP=dk-latin1" > /etc/vconsole.conf
# Enable SSH
echo "Enabling SSH..."
systemctl enable ssh
# Set WiFi region to DK
echo "Setting wireless region to DK..."
iw reg set DK
# Set Samba access
sed -i 's/read only = [A-Za-z]*/read only = no/' /etc/samba/smb.conf
sed -i 's/create mask = [0-9]*/create mask = 0644/' test /etc/samba/smb.conf
sed -i 's/directory mask = [0-9]*/directory mask = 0755/' /etc/samba/smb.conf
yes "${pass}" | smbpasswd -a local
yes "${pass}" | smbpasswd -a pi

# test text to speech:
echo "Testing text to speech..."
echo "The brown fox jumps over lazy dog" > /home/local/aaa.txt
text2wave /home/local/aaa.txt -o /home/local/aaa.wav
# Remove piwiz autostart
rm -f /etc/xdg/autostart/piwiz.desktop

# Create home dirs and populate a music track
mkdir -p /home/local/Downloads
mkdir -p /home/local/Music
cp ${wrkdir}/radetzky-marsch_Schloss-Schoenbrunn-Konzerte_Wien_full-length.mp3 /home/local/Music
ln -sf /home/local/Music/radetzky-marsch_Schloss-Schoenbrunn-Konzerte_Wien_full-length.mp3 /home/local/Music/music.mp3

# Checkout the SVN repo
bold "Checking out SVN repos, and building robobot software..."
svn co svn://repos.gbar.dtu.dk/jcan/regbot /home/local/regbot
ln -sf /home/local/regbot/mission /home/local/mission 
ln -sf /home/local/regbot/robobot_bridge /home/local/robobot_bridge 
ln -sf /home/local/regbot/regbotgui /home/local/regbotgui

# Build regbot bridge
bold "Building bridge software..."
mkdir -p /home/local/regbot/robobot_bridge/build
cmake -S /home/local/regbot/robobot_bridge -B /home/local/regbot/robobot_bridge/build
make -C /home/local/regbot/robobot_bridge/build -j3

# Build mission
bold "Building mission software..."
mkdir -p /home/local/regbot/mission/build
cmake -S /home/local/regbot/mission -B /home/local/regbot/mission/build
make -C /home/local/regbot/mission/build -j3

# set output device to jack
# works after next login
echo "defaults.pcm.card 0" > /home/local/.asoundrc
echo "defaults.ctl.card 0" >> /home/local/.asoundrc

# implement /etc/rc.local
mv /etc/rc.local /etc/rc.local.bak
cp ${wrkdir}/rc.local /etc/rc.local
chmod 754 /etc/rc.local

# Set up wifi settings:
mv /etc/wpa_supplicant/wpa_supplicant.conf /etc/wpa_supplicant/wpa_supplicant.conf.bak
cp ${wrkdir}/wpa_supplicant.conf /etc/wpa_supplicant/
cp ${wrkdir}/DTUsecure.pem /etc/wpa_supplicant/
cp ${wrkdir}/setup-dtusecure.sh /etc/wpa_supplicant/
chmod 754 /etc/wpa_supplicant/setup-dtusecure.sh

# Give back the rights to local for everything generated in home dir
chown -R local:local /home/local

# Setting up the robobot board to defaults:
bold "Setting robobot to defaults."
source ${wrkdir}/robobot-settings.sh

# Prompt for setting DTUsecure access
bold "It is recommended to get access to DTUsecure WiFi before rebooting."
prompt-yn "Would you like to set DTUsecure access?" y
status=$?
if [ $status -eq 0 ]; then
	source /etc/wpa_supplicant/setup-dtusecure.sh
fi

# Tell user to reboot
bold "Installer done! Please perform: sudo reboot"