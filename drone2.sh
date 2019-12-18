sudo systemctl restart multimastersetup 
sleep 4
roslaunch swarm_search apm2.launch &
sleep 4 
rosservice call /drone2/mavros/set_stream_rate 0 10 1
sleep 2
rosservice call /drone2/mavros/set_stream_rate 0 10 1




