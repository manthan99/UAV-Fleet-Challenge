#sudo killall roscore &
#sleep 2 &

#sudo date 120900002019.00
sleep 1
sudo route add -net 224.0.0.0 netmask 224.0.0.0 wlan0 
sleep 1 
roscore &
sleep 4 
rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1 &
sleep 2 
rosrun master_sync_fkie master_sync &
sleep 1 
roslaunch swarm_search apm2.launch &
sleep 1
roslaunch swarm_search master2.launch &




