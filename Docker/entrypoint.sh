#!/bin/bash

DUMMY_GID=9999

echo "Starting entrypoint script with user 'douglas' (UID: $(id -u douglas), GIDs: $(id -G douglas))"
echo "Home is now: $HOME"

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

echo "Detected host GIDs: render=$HOST_RENDER_GID, video=$HOST_VIDEO_GID"

# 2. Fix collisions (like 'messagebus' holding the ID)
for GID in $HOST_RENDER_GID $HOST_VIDEO_GID; do
    # Check if the GID is already taken by a group other than 'render' or 'video'
    CONFLICT=$(getent group $GID | cut -d: -f1)
    if [[ -n $CONFLICT ]] && [[ $CONFLICT != "render" ]] && [[ $CONFLICT != "video" ]]; then
        # if it is already taken, give the conflicting group a dummy GID to free up the desired GID for render/video
        echo "GID $GID is taken by group '$CONFLICT'. Reassigning '$CONFLICT' to GID $DUMMY_GID to free up $GID for render/video."
        sudo groupmod -g $DUMMY_GID "$CONFLICT"
    # Decrement the dummy GID for the next potential conflict
	DUMMY_GID=$(( DUMMY_GID - 1 ))
    fi
done

# 3. Align the container's groups to match the hardware GIDs exactly, either change the existing 'render' and 'video' groups or create them if they don't exist
sudo groupmod -g $HOST_RENDER_GID render 2>/dev/null || sudo groupadd -g $HOST_RENDER_GID render
sudo groupmod -g $HOST_VIDEO_GID video 2>/dev/null || sudo groupadd -g $HOST_VIDEO_GID video

# 4. Add douglas to the freshly aligned groups
sudo usermod -aG render,video douglas

# 5. Execute the original container command (bash)
exec "$@"


