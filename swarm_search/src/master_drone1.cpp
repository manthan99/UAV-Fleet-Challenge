#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h> 
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include <ros/ros.h>
#include <ros/duration.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Float64.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <unistd.h>
#include <time.h>
#include <cmath>
#include <math.h>

#include <geographic_msgs/GeoPoseStamped.h>

#include <swarm_search/sip_goal.h>
#include <swarm_search/point_list.h>

using namespace std;

//Set global variables
mavros_msgs::State current_state;

float takeoff_alt = 1.5;
double goal_tolerance = 0.002;
int cnt =0;
bool connected;
bool armed = 0;
bool reached_target;

bool takeoff_drone; // to tell when to takeoff 
bool land_drone; // to tell when to land

bool takeoff_done = 0;
bool land_done;

bool ROI_scan_flag = 0;

double delta_to_destination;
double prev_delta = 0.0;
int time_tolerance_to_destination = 100000;

sensor_msgs::NavSatFix current_pose; //current drone pose

geographic_msgs::GeoPoseStamped pose; //target pose - updated in function setDestination

geometry_msgs::Twist drone_status;

geometry_msgs::Twist flags;

swarm_search::point_list arr;

/*
flags.linear.x = scan_flag
flags.linear.y = between scanning and search
flags.linear.z = search_flag
flags.angular.x = targets_found
flags.angular.y = recovery1_flag
flags.angular.z = recovery2_flag
*/

float initial_takeoff_height = 0;
int first=0;
swarm_search::sip_goal master_goal;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
  bool connected = current_state.connected;
  bool armed = current_state.armed;
}


//get current position of drone
void pose_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_pose = *msg;
  if(first<15)
  {
    initial_takeoff_height = current_pose.altitude;
    first += 1;
}

  // ROS_INFO("Latitude %f Longitude: %f Altitude: %f", current_pose.latitude, current_pose.longitude, current_pose.altitude);
}

// void setDestination(float x, float y, float z)
void setDestination(const geographic_msgs::GeoPoseStamped::ConstPtr& msg2)
{
  pose = *msg2;
  double X,Y,Z;

  X = pose.pose.position.latitude;
  Y = pose.pose.position.longitude;
  Z = pose.pose.position.altitude;

  //DIAG POSE - DOUBLE VAL 3

  ROS_INFO("Destination set to Latitude: %f longitudetude: %f Altitude %f", X, Y, Z);
}

void callback_sip(const swarm_search::sip_goal::ConstPtr& msg)
{
  master_goal = *msg;
  cout<<"master goal takeoff flag is "<< master_goal.takeoff_flag.data << endl ;
}

void receive_cmd(const mavros_msgs::GlobalPositionTarget::ConstPtr& msg)
{

}

double haversine(double lat1, double lon1, double lat2, double lon2) 
{ 
    // distance between latitudes 
    // and longitudes 
    double dLat = (lat2 - lat1) * M_PI / 180.0; 
    double dLon = (lon2 - lon1) * M_PI / 180.0; 

    // convert to radians 
    lat1 = (lat1) * M_PI / 180.0; 
    lat2 = (lat2) * M_PI / 180.0; 

    // apply formulae 
    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2)*cos(lat1)*cos(lat2); 
    double rad = 6371; 
    double c = 2 * asin(sqrt(a)); 
    return rad * c; 
} 

int navigate(ros::NodeHandle nh, geographic_msgs::GeoPoseStamped pose1)
{
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(10.0);

  cout << pose1 << endl;

  ros::Publisher drone_status_pub = nh.advertise<geometry_msgs::Twist>("/master/drone1/drone_status", 10);

  ros::Publisher global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("/drone1/mavros/setpoint_position/global_to_local", 10);
  ros::Subscriber currentPos = nh.subscribe<sensor_msgs::NavSatFix>("/drone1/mavros/global_position/global", 10, pose_cb);

  drone_status.linear.x = 0;
  drone_status.linear.y = 0;
  drone_status.linear.z = 0;
  drone_status.angular.x = 0;
  drone_status.angular.y = 0;
  drone_status.angular.z = 0;
  // allow the subscribers to initialize
  ROS_INFO("INITIALISING...");
  for(int i=0; i<100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  int tolerance_time = 0;
  if(global_pos_pub)
  {    //to reach the destination, it returns 1 when it has reached the destination
   // for (int i=time_tolerance_to_destination; ros::ok() && i>0;--i)
	while(ros::ok())
    {
     // global_pos_pub.publish(pose1);
      delta_to_destination = haversine(pose1.pose.position.latitude,pose1.pose.position.longitude,current_pose.latitude,current_pose.longitude);
     //if(delta_to_destination > 0.005)
       //     global_pos_pub.publish(pose1);
		pose1.header.stamp = ros::Time::now();
		//if( cnt%2)
        //{
         // pose1.pose.position.latitude = pose1.pose.position.latitude + 0.000005;
          //pose1.pose.position.longitude = pose1.pose.position.longitude + 0.000005;
          //pose1.pose.position.altitude = pose1.pose.position.altitude;
        //}
        //else
        //{

//        pose1.pose.position.latitude = pose1.pose.position.latitude - 0.000005;
  //      pose1.pose.position.longitude = pose1.pose.position.longitude - 0.000005;
    //    pose1.pose.position.altitude = pose1.pose.position.altitude;
      //  }
        //cnt++;
        if(abs(delta_to_destination - prev_delta) < 0.000005)
        {
          tolerance_time++;
		ROS_INFO("tolerance time is : %d", tolerance_time);
        }

  		global_pos_pub.publish(pose1);
      ROS_INFO("Delta to Destination : %f Height Difference Remaining %f ", delta_to_destination, abs(pose1.pose.position.altitude-current_pose.altitude));
      // cout << "Delta to Destination : "<< delta_to_destination << endl;
      // cout << "Height Difference Remaining : " << abs(pose1.pose.position.altitude-current_pose.altitude) << endl;

      drone_status.linear.x = delta_to_destination;
      drone_status.linear.y = pose1.pose.position.altitude-current_pose.altitude;

      drone_status_pub.publish(drone_status);

      if( ((delta_to_destination < goal_tolerance) && abs(pose1.pose.position.altitude-current_pose.altitude)<0.5)||tolerance_time > 200) /////////////////////change this//////////
      {
        ROS_INFO("Reached at the target position");  
        ROS_INFO("tolerance time is : %d", tolerance_time);
        return 1;
      }
      prev_delta = delta_to_destination;
      ros::spinOnce();
      //ros::Duration(0.5).sleep();
	rate.sleep();

    }
 }
  //while (ros::ok())
  //{
    // pose1.header.stamp = ros::Time::now();
    // global_pos_pub.publish(pose1);

    //ros::spinOnce();
    //rate.sleep();
  //}

}

bool arm_drone(ros::NodeHandle nh)
{
  // arming
  ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("/drone1/mavros/cmd/arming");
  mavros_msgs::CommandBool srv_arm_i;
  srv_arm_i.request.value = true;
  if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success){
    ROS_INFO("ARM sent %d", srv_arm_i.response.success);
    cout << "*******************************************************************" << endl;
    return 1;
  }
  else
  {
    ROS_ERROR("Failed arming");
    return 0;
  }
}

bool takeoff(ros::NodeHandle nh, float takeoff_alt_local)
{
    //request takeoff
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/drone1/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = takeoff_alt_local;
    if(takeoff_cl.call(srv_takeoff)){
      ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
      sleep(10);
       return 1;
    }
    else{
      ROS_ERROR("Failed Takeoff");
      return 0;
    }

    sleep(10);
}

bool land(ros::NodeHandle nh)
{
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/drone1/mavros/cmd/land");
  mavros_msgs::CommandTOL srv_land;
  if (land_client.call(srv_land) && srv_land.response.success)
  {
    ROS_INFO("land sent %d", srv_land.response.success);
    sleep(10);
    return 1;
  }

  else
  {
    ROS_ERROR("Landing failed");
    ros::shutdown();                 /////////////////////////////////////////////////doubt here
    return 0;
  }
  sleep(10);
}


int check_ROI_confidence()
{
	// to be done
	return  1;
}


void roi_list(const swarm_search::point_list::ConstPtr& msg)
{
  arr = *msg;
  // cout << sizeof(arr)/sizeof(random_pt) <<endl;
  // cout << arr.points[0] <<endl;
}

int search_main(ros::NodeHandle nh)
{
  int k = 0;

  for(int i=0;arr.points[i].x!=0;i++)
  {
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "target_position";
    pose.pose.position.latitude = arr.points[i].x;
    pose.pose.position.longitude = arr.points[i].y;
    pose.pose.position.altitude = current_pose.altitude;
    //pose.pose.orientation.w = 1.0;
    k = navigate(nh,pose);
    sleep(5);
    int detected_points = 0;
    if(detected_points >=4)
    {
      return 1;
    }
  }

  //exit ke baad flag change hona chahiye code me

  ROS_INFO("Main search is now over");
  return 0;

  // ek callback hoga yaha se that will take ROI points and go there.

	// multi goal path planning se waypoints leke list me rakaho
	// ek ek kar ke navigate(pose) : pose update karte raho when reached prev target
	// sleep for some time so that sufficient frames can be taken
	// keep checking # of target detection, if = 4 : end search  
}

void scan_main(ros::NodeHandle nh)
{

  ros::Publisher flags_pub = nh.advertise<geometry_msgs::Twist>("/drone1/flags", 10);
  
	pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "target_position";
	pose.pose.position.latitude = master_goal.sip_start.x;
	pose.pose.position.longitude = master_goal.sip_start.y;
	//pose.pose.position.latitude = 22.319573;
        //pose.pose.position.longitude = 87.301437;
          pose.pose.position.altitude = initial_takeoff_height + 5;
  //pose.pose.orientation.w = 1.0;
  // pose.pose.position.altitude = 10;

  int x = navigate(nh, pose); // navigate function to be done, returns 1 when reached

	int ittr_count = 1;
	
	if(x == 1)
	{
		ROS_INFO(" SIP Reached ");
		ROS_INFO(" Requesting ROI Scanning to start ");
		//ROI_scan_flag = 1;
     
    //drone_status.angular.x = ROI_scan_flag;
		// when ROI_scan_flag ==1 : start ROI_detection code

    flags.linear.x = 1; 
    flags_pub.publish(flags); //scanning should start here

   	sleep(5);

		pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "target_position";
		pose.pose.position.latitude = master_goal.sip_end.x;
		pose.pose.position.longitude = master_goal.sip_end.y;
		//pose.pose.position.latitude = 22.319571;
                //pose.pose.position.longitude = 87.301337;
                pose.pose.position.altitude = current_pose.altitude;
    //pose.pose.orientation.w = 1.0;

    // pose.pose.position.altitude = 10;

   	int y = navigate(nh, pose);

 		if(y == 1)
 		{
  		ROS_INFO(" Itteration 1 Scanning Complete ");
  		//ROS_INFO(" Requesting ROI Scanning to stop and pass coordinated to Multi-Goal Path Planner ")
  		//ROI_scan_flag = 0;
      flags.linear.x = 0;//ROI_scan_flag;
      flags.linear.y = 1; // between scanning and search
      flags_pub.publish(flags);
  		// when ROI_scan_flag == 0 : stop ROI_detection code

      int z = check_ROI_confidence(); // to be done, it analyses features 
    	if (z == 1)
    	{	
        ROS_INFO(" Good ROI Confidence - Proceeding to Search and Verify Target ");
       sleep(2);
	 pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "target_position";
        pose.pose.position.latitude = master_goal.sip_end.x;
        pose.pose.position.longitude = master_goal.sip_end.y;
        pose.pose.position.altitude = initial_takeoff_height + 2;
        //pose.pose.orientation.w = 1.0;

        // start Multi-Goal Path Planning and wait for result, also start the detection code
       
        int w = navigate(nh,pose);
       	sleep(1);
	 	//land(nh);
        //return;
        if (w==1)
        {
          ROS_INFO(" Reached the Search Height ");
          flags.linear.y = 0; //between scanning and search over
          flags.linear.z = 1; // search starts
          flags_pub.publish(flags);
          sleep(1);
          int k = search_main(nh);    // k = 1 means all detection is completed and all four are found
          // k = 0 means not all have been found but all roi has been explored
        }

    	}
    	else
    	{
    		ROS_INFO(" Poor ROI Confidence - Requesting Recovery Mode 1 ");
    		flags.angular.y = 1;//recovery1_flag = 1;
    		flags_pub.publish(flags);
    	}
 		}
 		else
 		{
 			ROS_INFO(" FAILED TO COMPLETE SCAN_1 ");
 		}
 	}

 	else
 	{
 		ROS_INFO(" FAILED TO REACH SIP ");
 	}

  bool z = land(nh);
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone1_master_node");
  ros::NodeHandle nh;

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/drone1/mavros/state", 10, state_cb);
  //ros::Publisher global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("/drone1/mavros/setpoint_position/global", 10);
  ros::Publisher global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("/drone1/mavros/setpoint_position/global", 10);
  ros::Subscriber currentPos = nh.subscribe<sensor_msgs::NavSatFix>("/drone1/mavros/global_position/global", 10, pose_cb);
 
  ros::Subscriber nextGoal = nh.subscribe<geographic_msgs::GeoPoseStamped>("/master/drone1/next_goal/pose", 10, setDestination);
  ros::Subscriber receive_master_cmd = nh.subscribe<mavros_msgs::GlobalPositionTarget>("/master/drone1/received_cmd", 10, receive_cmd);
  ros::Subscriber groundStation_value = nh.subscribe<swarm_search::sip_goal>("/master/drone1/ground_msg",10,callback_sip);
  ros::Subscriber ROI_points = nh.subscribe<swarm_search::point_list>("/drone1/ROI_flow",10,roi_list);
  // allow the subscribers to initialize

  flags.linear.x = 0;
  flags.linear.y = 0;
  flags.linear.z = 0;

  flags.angular.x = 0;
  flags.angular.y = 0;
  flags.angular.z = 0;

  ROS_INFO("INITIALISING...");
  for(int i=0; i<100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  // change mode to GUIDED
  while(current_state.mode != "GUIDED")
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  // // wait for FCU connection
  while(ros::ok() && current_state.mode == "GUIDED")
	{
  // // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  while(!takeoff_done && ros::ok())
  {
    while(!armed  && ros::ok() && current_state.mode == "GUIDED")
    {
      armed = arm_drone(nh);
		cout << "stuck here" << endl;
		ros::spinOnce();
    }
   
    while( master_goal.takeoff_flag.data != true  && ros::ok() && current_state.mode == "GUIDED")
    {
      cout <<"**********hjvjvj*************" <<master_goal.takeoff_flag.data << endl;
      armed = arm_drone(nh);
      cout << master_goal.takeoff_flag.data << endl;
      ROS_INFO("Waiting for the permission to take off");
      ros::spinOnce();
      ros::Duration(0.01).sleep();
      //wait for permission to take off
    }
  
  ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/drone1/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = 1.5;
  if(takeoff_cl.call(srv_takeoff)){
    ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
    takeoff_done = 1;

   }else{
    ROS_ERROR("Failed Takeoff");
    return -1;
  }

  sleep(10);
   // takeoff_done = takeoff(nh, takeoff_alt);
  }
  ros::spinOnce();
  scan_main(nh);
 }}

