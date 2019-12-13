#sudo killall roscore &
#sleep 2 &

#sudo date 120900002019.00
#sleep 1
sudo route add -net 224.0.0.0 netmask 224.0.0.0 wlan0 
sleep 1 
roslaunch swarm_search apm2.launch &
sleep 4 
#rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1 &
#sleep 2 
#rosrun master_sync_fkie master_sync _sync_topics:=['/test','master/drone2/ground_msg','/drone2/chatter','/drone0/mavros/global_position/global','/drone1/mavros/global_position/global','/drone2/mavros/global_position/global','/drone3/mavros/global_position/global','/drone0/mavros/state','/drone1/mavros/state','/drone2/mavros/state','/drone3/mavros/state'] &
#sleep 1
roslaunch swarm_search master2.launch &
sleep 1
python ~/ardupilot_ws/src/UAV_Fleet_Challenge/Planning/waypoint_generator/src/path2.py &



