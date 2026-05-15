#!/bin/bash

BRIDGE_NAME=br0

JP_LINK=docker0
DVL_DEVICE=wlo1

# wait for device names to be available first
while ! ip link show "$JP_LINK" &>/dev/null || ! ip link show "$DVL_DEVICE" &>/dev/null; do
    echo "Waiting for interfaces..."
    sleep 1
done

# if bridge exist delete it
if ip link show "$BRIDGE_NAME" &>/dev/null; then
	sudo ip link set $BRIDGE_NAME down
	sudo ip link delete $BRIDGE_NAME type bridge
fi
RED='\033[0;31m'
NC='\033[0m' # No Color
# warn user about SSH being disabled and ask for confirmation
echo -e "Warning: This script will create a network bridge between $JP_LINK and $DVL_DEVICE. \n${RED}This will likely disable SSH. ${NC} \nContinue? Y/[n]"
read tmp
if [[ "$tmp" != "Y" && "$tmp" != "y" ]]; then
    echo "Aborting."
    exit 0
fi
sudo ip link add name $BRIDGE_NAME type bridge
sudo ip link set dev $JP_LINK master $BRIDGE_NAME
sudo ip link set dev $DVL_DEVICE master $BRIDGE_NAME

sudo ip link set $BRIDGE_NAME up
sudo ip link set $JP_LINK up
sudo ip link set $DVL_DEVICE up
