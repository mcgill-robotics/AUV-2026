#!/bin/bash

# most common pluggable devices
# tty: serial devices
# video: cameras
# hidraw: human device interface
# input: input devices (keyboards, mice, etc.)
# sd[a-z]: storage devices (e.g. USB drives, external SSD), skip numbers because those are usually partitions
# sg: similar to sd
# net: virtual network interfaces (e.g. USB Ethernet adapters)
# snd: USB audio interfaces
WATCH_REGEX='.*/\(tty\|video\|hidraw\|input\|sd[a-z]\|sg\|snd\).*'

# generic command to find all pluggable devices
find_pluggable_devices() {
    # search in /dev, only search up to two "directories" down
    find /dev -maxdepth 2 -regex "$WATCH_REGEX" | sort
}

echo "Please unplug the device you're trying to identify."
read -p "Press Enter when ready..."

# List current devices
before=$(find_pluggable_devices)
echo "Captured list before plugging in the device."

echo
echo "Now plug in the device."
read -p "Press Enter when the device is plugged in..."
# Sleep for a moment to allow the system to recognize the new device and create the corresponding /dev entry
sleep 1

# List pluggable devices again
after=$(find_pluggable_devices)
echo "Captured list after plugging in the device."

# Compare the two lists to find the difference
echo
echo "New device(s) detected:"
diff <(printf "%s\n" "${before[@]}" | sort) <(printf "%s\n" "${after[@]}" | sort) | grep '^>' | sed 's/^> //'