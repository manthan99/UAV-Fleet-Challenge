sudo systemctl restart multimastersetup 
sleep 4
roslaunch swarm_search apm1.launch &
sleep 4 
rosservice call /drone1/mavros/set_stream_rate 0 10 1
sleep 2
rosservice call /drone1/mavros/set_stream_rate 0 10 1




