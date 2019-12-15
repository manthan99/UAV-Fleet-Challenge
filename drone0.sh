sudo systemctl restart multimastersetup 
sleep 4
roslaunch swarm_search apm0.launch &
sleep 4 
rosservice call /drone0/mavros/set_stream_rate 0 10 1




