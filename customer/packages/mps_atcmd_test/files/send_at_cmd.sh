#!/bin/sh

# Usage: ./send_at.sh "AT command"
# Socket path
SOCKET="/var/run/m2mb_ssd_serv"

# Check if input argument is provided
if [ -z "$1" ]; then
	echo "Usage: $0 \"AT command\""
	exit 1
fi

# AT command argument
AT_CMD="$1"

# Send command and print response
echo -e "${AT_CMD}\r\n" | socat - UNIX-CONNECT:$SOCKET

