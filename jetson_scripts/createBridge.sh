#!/bin/bash

BRIDGE_NAME=br0

JP_LINK=eno1
DVL_DEVICE=enx00e04c2a4978

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


sudo ip link add name $BRIDGE_NAME type bridge
sudo ip link set dev $JP_LINK master $BRIDGE_NAME
sudo ip link set dev $DVL_DEVICE master $BRIDGE_NAME

sudo ip link set $BRIDGE_NAME up
sudo ip link set $JP_LINK up
sudo ip link set $DVL_DEVICE up
