#!/bin/bash
# Script to setup robobot software

wrkdir=$(pwd)
echo "Working from: ${wrkdir}"

### Installer must be run as root
# Check if being run as root:
if [ "$(whoami)" != "root" ]; then
	echo "You must run this install-script as root!"
	echo "Example: sudo ./install.sh"
	exit 1
fi

### Common prompt functions:
# Will prompt for a yes or no: [y/n]
# Use: prompt-yn y
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

### Part 1:
# - setup of Robobot
echo "Installing updates..."
apt update
apt dist-upgrade -y

# hostname
prompt-txt "Enter hostname for the robobot" y host
echo "$host" > /etc/hostname
linetoedit=$(grep -n "raspberrypi" /etc/hosts | awk -F ':' '{print $1}')

# Add local user
echo "Adding user local..."
adduser local
# requires password and other settings manually
usermod -a --groups adm,cdrom,sudo,audio,video,plugdev,games,users,netdev,input,spi,gpio,i2c,dialout local 
passwd local

### Part 2:
# needed packages
apt install -y ntp ntpdate libreadline-dev subversion cmake libopencv-dev htpdate python3-pyqtgraph python3-scipy pyqt5-dev pyqt5-dev-tools festival sox libsox-fmt-all
apt autoremove -y

# set date from DTU server
ntpdate -u ntp.ait.dtu.dk

### Part 3:
# su - local
# test text to speech:
echo "The brown fox jumps over lazy dog" > /home/local/aaa.txt
text2wave /home/local/aaa.txt -o /home/local/aaa.wav

# Create home dirs and populate a music track
mkdir -p /home/local/Downloads
mkdir -p /home/local/Music
cp ${wrkdir}/radetzky-marsch_Schloss-Schoenbrunn-Konzerte_Wien_full-length.mp3 /home/local/Music
ln -sf /home/local/Music/radetzky-marsch_Schloss-Schoenbrunn-Konzerte_Wien_full-length.mp3 /home/local/Music/music.mp3
if [ $host == "eva" ]; then
	cp ${wrkdir}/a-cruel-angels-thesis.mp3 /home/local/Music
	ln -sf /home/local/Music/a-cruel-angels-thesis.mp3 /home/local/Music/music.mp3
fi

svn co svn://repos.gbar.dtu.dk/jcan/regbot /home/local/regbot
ln -sf /home/local/regbot/mission /home/local/mission 
ln -sf /home/local/regbot/robobot_bridge /home/local/robobot_bridge 
ln -sf /home/local/regbot/regbotgui /home/local/regbotgui

# Build mission
mkdir -p /home/local/regbot/mission/build
cmake -S /home/local/regbot/mission -B /home/local/regbot/mission/build
make -C /home/local/regbot/mission/build -j3

# Build regbot bridge
mkdir -p /home/local/regbot/robobot_bridge/build
make -S /home/local/regbot/robobot_bridge -B /home/local/regbot/robobot_bridge/build
make -C /home/local/regbot/mission/build -j3

# Give back the rights to local for everything generated in home dir
chown -R local:local /home/local