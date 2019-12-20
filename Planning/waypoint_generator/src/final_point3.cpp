#include <cmath>
#include "ros/ros.h"
#include <math.h>
#include "geodetic_conv.hpp"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"
#include <geometry_msgs/Twist.h>
#include "waypoint_generator/point_list.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float64.h>
#include <swarm_search/local_flags.h>


// #define M_PI 3.14159

using namespace std;
using namespace ros;

double heading_angle = 0.0;


// nav_msgs::Path path;
bool waypoint_reached = true;
double height = 5;
double FOV = 72.4*M_PI/180.0;  			// define in radians 
double resolution_y = 480;		// image cols
double resolution_x = 640;		// image rows

swarm_search::local_flags flags;
std::map < pair<double, double>, vector<pair<double, double> > > ROI_list;
ros::Publisher close_waypoint_pub;

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

void statusCallback(const swarm_search::local_flags::ConstPtr& flag_val)
{
	flags = *flag_val;
}

geometry_msgs::PoseStamped current_pos;
void localposcallback(const geometry_msgs::PoseStamped::ConstPtr& position)
{
	current_pos = *position;
	height = current_pos.pose.position.z;
	height = 5;
	// cout<<"Height received: "<<height<<endl;
}

double prev_lat = 0.0, prev_long = 0.0, distance_threshold = 0.005;
waypoint_generator::point_list frame_points_gps;
void closeFramePointCallback(const waypoint_generator::point_list::ConstPtr& frame_points)
{
	if (flags.search_flag.data == true ) // will enter if flag is 1
	{
		double lat = frame_points->points[0].x;
		double lon = frame_points->points[0].y;
		heading_angle = frame_points->points[0].z;

		geodetic_converter::GeodeticConverter conv;
		conv.initialiseReference(lat,lon,0);
		// cout<<"length_per_pixel: "<<length_per_pixel<<endl;

		double alpha = 0;
		double lat_temp, lon_temp, alt_temp;
		double distance_N, distance_E, distance;
		double length_per_pixel = 2*height*tan(FOV/2)/resolution_y; //FOV or focal length
		
		geometry_msgs::Point temp_point;
		for (int i=1; i<frame_points->points.size(); i++)
		{
			distance = length_per_pixel*sqrt(pow(frame_points->points[i].x-resolution_x/2, 2)+pow(frame_points->points[i].y-resolution_y/2, 2));
			// cout<<"distance: "<<distance<<endl;
			if((frame_points->points[i].y-resolution_y/2)!= 0)
				alpha = atan((frame_points->points[i].x-resolution_x/2)/(frame_points->points[i].y-resolution_y/2));
			else if((frame_points->points[i].x-resolution_x/2)>0)
				alpha = M_PI/2;
			else if((frame_points->points[i].x-resolution_x/2)<0 )
				alpha = -M_PI/2;
			else alpha = 0; 
			if (frame_points->points[i].y-resolution_y/2 < 0)
				distance *= (-1);
			distance_E = distance*sin(M_PI-alpha+heading_angle);
			distance_N = distance*cos(M_PI-alpha+heading_angle);    
			cout<<"frame_points->points[i].x: "<<frame_points->points[i].x<<" frame_points->points[i].y: "<<frame_points->points[i].y<<
			"α: "<<alpha<<"distance_N: "<<distance_N<<" distance_E: "<<distance_E<<endl; 

			conv.enu2Geodetic(distance_E, distance_N, 0, &lat_temp, &lon_temp, &alt_temp);
			temp_point.x = lat_temp;
			temp_point.y = lon_temp;
			// cout << std::fixed << std::setprecision(8)<< "Lat: "<<lat_temp<<" Lon: "<<lon_temp<<endl;
		
		}	

		double scale = 0.00001;
		for (int i=0; i<frame_points_gps.points.size(); i++)
		{
			double lat = frame_points_gps.points[i].x;
			double lon = frame_points_gps.points[i].y; 
			double lat_scaled = (int)(lat/scale) * scale;
			double lon_scaled = (int)(lon/scale) * scale;
			ROI_list[{lat_scaled, lon_scaled}].push_back({lat, lon});
		}
		cout<<"Moving to next frame\n"<<endl;
	}
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "final_points1");
	ros::NodeHandle n;
	
	ros::Subscriber current_position = n.subscribe("/drone3/mavros/local_position/pose",10, localposcallback);
	ros::Subscriber closed_frame_point_sub = n.subscribe("/drone3/close_coordinates", 1000, closeFramePointCallback);
  	ros::Subscriber status_sub = n.subscribe("/drone3/flags", 1000, statusCallback);
	
  	close_waypoint_pub = n.advertise<geometry_msgs::Point>("/drone3/listener1", 1000);

    ros::Rate loop_rate(20);
    while( ros::ok )
    {	
		if (flags.recovery1_flag.data == true)
		{
			cout<<"Averaging started"<<endl;
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
				// cout<<"Final points: "<< std::fixed << std::setprecision(8)<<temp.x<<" "<<temp.y<<endl;
			}
			close_waypoint_pub.publish(final_target_points_gps);
		}
		ros::spinOnce();
		loop_rate.sleep();
    }

	return 0;
}

