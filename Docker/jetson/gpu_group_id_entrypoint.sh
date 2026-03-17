#!/bin/bash

DUMMY_GID=9999

fixuid -q

# 1. Dynamically detect the host's GIDs directly from the mounted hardware
if [[ -e /dev/dri/renderD128 ]]; then
    HOST_RENDER_GID=$(stat -c "%g" /dev/dri/renderD128)
else
    HOST_RENDER_GID=104 # Fallback
fi

if [ -e /dev/nvhost-gpu ]; then
    HOST_VIDEO_GID=$(stat -c "%g" /dev/nvhost-gpu)
else
    HOST_VIDEO_GID=44 # Fallback
fi

# 2. Fix collisions (like 'messagebus' holding the ID)
for GID in $HOST_RENDER_GID $HOST_VIDEO_GID; do
    CONFLICT=$(getent group $GID | cut -d: -f1)
    if [[ -n $CONFLICT ]] && [[ $CONFLICT != "render" ]] && [[ $CONFLICT != "video" ]]; then
        sudo groupmod -g $DUMMY_GID "$CONFLICT"
	DUMMY_GID=$(( DUMMY_GID - 1 ))
    fi
done

# 3. Align the container's groups to match the hardware GIDs exactly
sudo groupmod -g $HOST_RENDER_GID render 2>/dev/null || sudo groupadd -g $HOST_RENDER_GID render
sudo groupmod -g $HOST_VIDEO_GID video 2>/dev/null || sudo groupadd -g $HOST_VIDEO_GID video

# 4. Add douglas to the freshly aligned groups
sudo usermod -aG render,video douglas

# 5. Execute the original container command (e.g., bash or ros2 launch)
exec "$@"


