roscore &
sleep 2 
sudo route add -net 224.0.0.0 netmask 224.0.0.0 wlo1
sleep 3
rosrun master_sync_fkie master_sync &
sleep 3 
rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1 &




