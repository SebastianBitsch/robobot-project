#!/bin/bash
# Script to setup login to DTUsecure

# Check if being run as root:
if [ "$(whoami)" != "root" ]; then
	echo "You must run this install-script as root!"
	echo "Example: sudo ./connect-dtusecure.sh"
	exit 1
fi

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

# Check wpa_supplicant.conf for existing DTUsecure configuration:
grep '\s*ssid\s*=\s*"DTUsecure"' /etc/wpa_supplicant/wpa_supplicant.conf >/dev/null 2>&1
retval=$?
exists=n
if [ $retval -eq 0 ]; then
	echo "DTUsecure settings exist, they will be overwritten!"
	exists=y
fi

# Obtain login creds by prompting user for username and password:
prompt-txt "Please enter username for DTUsecure (format: example@elektro.dtu.dk)" y user
read -s -p "Please enter password (will not be printed on screen): " pass

# Create hashed password:
pass=$(echo -n "${pass}" | iconv -t utf16le | openssl md4 | awk -F "= " '{print $2}')

if [ $exists == "y" ]; then
	# Get the location of the DTUsecure settings:
	sed -n '/network={/ n;/ssid="DTUsecure"/,/}/=' /etc/wpa_supplicant/wpa_supplicant.conf > /tmp/DTUsecure_settings.out

	startln=$(head -1 /tmp/DTUsecure_settings.out | awk '{printf "%d", $1 - 1}')
	endln=$(tail -1 /tmp/DTUsecure_settings.out)

	# Delete the current settings
	sed "${startln},${endln}d" /etc/wpa_supplicant/wpa_supplicant.conf > /tmp/wpa_supplicant.tmp
else
	# Copy wpa_supplicant to temp for appending:
	cp /etc/wpa_supplicant/wpa_supplicant.conf /tmp/wpa_supplicant.tmp
fi
# Append the new settings to the wpa_supplicant.conf file
echo "Writing settings to wpa_supplicant..."

echo -e "network={" >> /tmp/wpa_supplicant.tmp
echo -e "    ssid=\"DTUsecure\"" >> /tmp/wpa_supplicant.tmp
echo -e "    key_mgmt=WPA-EAP" >> /tmp/wpa_supplicant.tmp
echo -e "    pairwise=CCMP" >> /tmp/wpa_supplicant.tmp
echo -e "    group=CCMP TKIP" >> /tmp/wpa_supplicant.tmp
echo -e "    eap=PEAP" >> /tmp/wpa_supplicant.tmp
echo -e "    ca_cert=\"/etc/wpa_supplicant/DTUsecure.pem\"" >> /tmp/wpa_supplicant.tmp
echo -e "    identity=\"${user}\"" >> /tmp/wpa_supplicant.tmp
echo -e "    phase2=\"auth=MSCHAPV2\"" >> /tmp/wpa_supplicant.tmp
echo -e "    password=hash:${pass}" >> /tmp/wpa_supplicant.tmp
echo -e "    anonymous_identity=\"anonymous@dtu.dk\"" >> /tmp/wpa_supplicant.tmp
echo -e "}" >> /tmp/wpa_supplicant.tmp

# Ensure wireless region is set
#echo "Setting wireless region to DK..."
#iw reg set DK

# Move files and clean-up
mv /etc/wpa_supplicant/wpa_supplicant.conf /etc/wpa_supplicant/wpa_supplicant.conf.bak
mv /tmp/wpa_supplicant.tmp /etc/wpa_supplicant/wpa_supplicant.conf
rm -f /tmp/DTUsecure_settings.out

echo "Done!"
