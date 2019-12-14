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

### To change for individual drone(0/1/2/3):

```
In File Planning/waypoint_generator/path.py : 
Change in listener, all drone2 to drone*  change all will work
In File Planning/waypoint_generator/talker.py : 
change in talker drone2 to drone* change all will work

In file swarm_search/src/ground_station_ros.py : 
Change according to which all drones are being used, also the input square and drone inputs

In file swarm_search/src/<master_drone1.cpp : 
change all drone2 to drone*
```