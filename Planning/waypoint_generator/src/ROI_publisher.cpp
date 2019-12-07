#include <cmath>
#include <fstream>
#include "ros/ros.h"
#include "geodetic_conv.hpp"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"
#include "waypoint_generator/point_list.h"
#include "geometry_msgs/PoseStamped.h"

#define PI 3.14159

using namespace std;
using namespace ros;

// nav_msgs::Path path;
int curr_waypoint = 0;
bool waypoint_reached = true;
double height = 0;
double FOV = 2*PI/3;
double resolution_y = 1280;
double resolution_x = 720;

geometry_msgs::PoseStamped current_pos;


double lat, lon, alt;
void globalCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
 	lat=msg->latitude;
 	lon=msg->longitude;
 	alt=msg->altitude;
}

void orientCallback(const std_msgs::Float64::ConstPtr& msg)
{
	heading_angle = *msg;
}

// input is list of frame points 
// converting that to distance in m
// finding cosine in direction of N and E 
// using N and E to find 
waypoint_generator::point_list frame_points_gps;
void framePointsToGPS (const waypoint_generator::point_list::ConstPtr& frame_points)
{

	geodetic_converter::GeodeticConverter conv;
	conv.initialiseReference(lat,lon,alt);
	double l = 2*height*tan(FOV/2); //metres
	double length_per_pixel = l/resolution_y;
	double lat_temp, lon_temp, alt_temp;
	double distance_N, distance_E;
	geometry_msgs::Point temp_point;
	for (int i=0; i<frame_points->points.size(); i++)
	{
		distance = *length_per_pixelsqrt(pow(frame_points->points[i].x-resolution_x/2, 2)+(frame_points->points[i].y-resolution_y/2, 2));
		distance_E = ;
		conv.enu2Geodetic(distance_E, distance_N, 0, &lat_temp, &lon_temp, &alt_temp);
		temp_point.x = lat_temp;
		temp_point.y = lon_temp;
		frame_points_gps.points.push_back(temp_point);
	}	
}

// waypoint_generator::point_list::ConstPtr& frame_points = NULL;
void framePointCallback(const waypoint_generator::point_list::ConstPtr& msg)
{
	framePointsToGPS(msg);
	// can be accessed as msg->points[i]
}

void localposcallback(const geometry_msgs::PoseStamped::ConstPtr& position)
{
	current_pos = *position;
	height = current_pos.pose.position.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ROI_publisher");
	ros::NodeHandle n;
	ros::Subscriber gps_sub = n.subscribe("/drone1/mavros/global_position/global", 10, globalCallback);
	ros::Subscriber frame_point_sub = n.subscribe("/some_topic", 1000, framePointCallback);
	ros::Subscriber current_position = n.subscribe("/drone1/mavros/local_position/pose",10, localposcallback);
  	// ros::Subscriber imu = n.subscribe("/imu", 1000, orientCallback);

	ros::Rate loop_rate(10);

	std::map < pair<double, double>, vector<pair<double, double> > > ROI_list;
	double scale = 0.00001;
	for (int i=0; i<frame_points_gps.points.size(); i++)
	{
		double lat = frame_points_gps.points[i].x;
		double lon = frame_points_gps.points[i].y; 
		double lat_scaled = (int)(lat/scale) * scale;
		double lon_scaled = (int)(lon/scale) * scale;
		ROI_list[{lat_scaled, lon_scaled}].push_back({lat, lon});
	}

	waypoint_generator::point_list final_target_points_gps;
	for (auto i=ROI_list.begin(); i!=ROI_list.end(); i++)
	{
		geometry_msgs::Point temp;
		double lat_final=0, lon_final=0;
		for (int j=0; j<i->second.size(); j++)
		{
			lat_final += i->second[j].first;
			lon_final += i->second[j].second;
		}
		lat_final /= i->second.size();
		lon_final /= i->second.size();
		temp.y = lon_final;
		temp.x = lat_final;
		final_target_points_gps.points.push_back(temp);
	}

	// depending on the reached signal publish next waypoint
    // ros::Rate loop_rate(20);
    while( ros::ok )
    {	
		
		ros::spinOnce();
		loop_rate.sleep();
    }


	return 0;
}





// input: pixel_points, drone_gps, theta, height, image_resolution(x,y) 