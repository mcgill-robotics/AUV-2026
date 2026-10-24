# 1. Create a clean, temporary Xauthority file for the container
touch ./.docker.xauth

# 2. Extract the current X11 cookie and merge it into the temporary file
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f ./.docker.xauth nmerge -