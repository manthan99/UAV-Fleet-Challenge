sudo systemctl restart multimastersetup 
sleep 4
roslaunch swarm_search apm3.launch &
sleep 4 
rosservice call /drone3/mavros/set_stream_rate 0 10 1




