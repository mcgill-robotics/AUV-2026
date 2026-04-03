
ROUTER_CONNECTION_NAME="JP-link"
# static IP assigned by router
ROUTER_JETSON_IP_ADDRESS=192.168.0.102

DVL_DEVICE_NAME=enx00e04c2a4978
# what the DVL thinks the jetson IP address is
DVL_JETSON_IP_ADDRESS=192.168.194.90
SUBNET_MASK_CIDR=24
DVL_CONNECTION_NAME=dvl-link

if nmcli connection show "$DVL_CONNECTION_NAME" > /dev/null 2>&1; then
	echo "Connection '$DVL_CONNECTION_NAME' already exists. Skipping creation."
else
	echo "Connection '$DVL_CONNECTION_NAME' not found. Creating and configuring"
	# add USB connection or type ethernet
	sudo nmcli connection add type ethernet con-name $DVL_CONNECTION_NAME ifname $DVL_DEVICE_NAME ip4 $DVL_JETSON_IP_ADDRESS/$SUBNET_MASK_CIDR
	
fi
# make USB connection manual which disables DHCP search
sudo nmcli connection modify $DVL_CONNECTION_NAME ipv4.method manual
# never use this connection as default gateway (prevent using it as internet source on boot
sudo nmcli connection modify $DVL_CONNECTION_NAME ipv4.never-default yes

# prevent router from acting as a gateway to the internet
sudo nmcli connection modify $ROUTER_CONNECTION_NAME ipv4.never-default yes
# router provides local network
sudo nmcli connection modify $ROUTER_CONNECTION_NAME ipv4.ignore-auto-routes yes
sudo nmcli connection modify $ROUTER_CONNECTION_NAME ipv4.ignore-auto-dns yes
# assign static IP from router connection
sudo nmcli connection modify $ROUTER_CONNECTION_NAME ipv4.addresses $ROUTER_JETSON_IP_ADDRESS/$SUBNET_MASK_CIDR



# activate connections
sudo nmcli connection up $DVL_CONNECTION_NAME
sudo nmcli connection up $ROUTER_CONNECTION_NAME
