#!/bin/bash
# Setup robobot settings on the regbot boards
# Author: jbepe
# Date: 2022-01-18

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

# Find the id of current robobot from the hostname
echo "Getting id of robobot..."
host=$(cat /etc/hostname)
hostdata=$(grep -i "${host}" ${wrkdir}/robobot-id.txt)
id=$(echo "${hostdata}" | awk -F "\t" '{print $1}')
if [ $id == "" ]; then
	id="0"
fi
#id=$(echo ${id})
revmotor=$(echo "${hostdata}" | awk -F "\t" '{print $3}')
if [ $revmotor == "" ]; then
	revmotor="0"
fi

if [ $revmotor -eq "1" ]; then
	motorval="15"
else
	motorval="1"
fi

#revmotor=$(echo ${revmotor})
bold "Hostname: ${host} - id: ${id} - rev. motor: ${revmotor}"

# Kill service if running
echo "Killing robobot bridge service..!"
pkill robobot_bridge

# Write robobot settings to EEPROM on board:
bold "Writing regbot settings for ${host}..:"
tee /dev/ttyACM0 << EOF1
sub 0 0 0
s=15 1
log+motV
log+motA
log+mvel
log+pose
log+bat
log+mis
log+motR
rid=$id 0.265 13.275 48 0.0324 0.0333 0 $motorval 9.9 6
cbap 0 1.8 0 1 9999.99 1 0 1 2 0 0.5 0.05 0 1 1 0 1 9999.99 0 0 1 0 1 1 0 1
cbav 0 1 0 1 9999.99 1 0 1 1 0 1 1 0 1 1 0 1 9999.99 1 0 1 0 1 1 0 9999.99
cbal 0 1 0 1 9999.99 1 0 1 1 0 1 1 0 1 1 0 1 9999.99 1 0 1 0 1 1 0 9999.99
cwve 0 1.5 0 1 9999.99 1 0 1 1 0 1 1 0 1 1 0 1 9999.99 1 0 1 0 1 1 0 9999.99
cwth 1 3.1 0 1 9999.99 1 0 1 1 0 1 1 0 1 1 0 1 9999.99 1 0 1 0 1 1 1 0.1
cedg 1 0.2 0 1 9999.99 1 1 0.3 1.5 1 0.25 0.04 0 1 1 0 1 9999.99 1 0 1 0 1 1 1 0.5
cpos 1 44.229 1 0.249 9999.99 1 1 0.24 0.028 1 0.24 0.028 0 1 1 0 1 9999.99 1 0 1 0 1 1 1 9999.99
ctrn 1 0.9 1 0.1 0.2 1 0 1 1 1 0.1 0.02 1 0 0.01 0 1 9999.99 1 0 1 0 1 1 1 0.5
cvel 1 5 0 1 9999.99 1 1 1 0.2 1 0 0.005 0 1 1 0 1 9999.99 1 1 4 0 1 1 1 9
<clear>
<add thread=1
<add vel=0.0, log=1.0: time=0.1
<add vel=0.3: dist=1.0
<add vel=0.0: log=0
eew

EOF1

# Print the response
bold "Response:"
timeout 0.1 cat /dev/ttyACM0

# Restart the service
echo "Starting robobot bridge service..."
nohup /home/local/robobot_bridge/build/robobot_bridge > /dev/null 2>&1 &

bold "Done!"