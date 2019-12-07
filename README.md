# UAV_Fleet_Challenge

## Packages : 
### swarm_search : https://github.com/reyanshsolis/swarm_search

```
git clone https://github.com/Aryan-jaiswal/UAV_Fleet_Challenge/

git submodule update --init --recursive
```

## To run in simulator(SITL) : 
```
Add this to one of your workspace and then follow the following commands.

../Tools/autotest/sim_vehicle.py --map --console in ArduCopter directory of Ardupilot
roslaunch swarm_search apm.launch
python UAV_Fleet_Challenge/Planning/waypoint_generator/src/path.py 
python UAV_Fleet_Challenge/Planning/waypoint_generator/src/talker.py
python UAV_Fleet_Challenge/swarm_search/src/ground_station_ros.py
rosrun swarm_search master_drone1

Then simply turn into guided mode and tada.
```