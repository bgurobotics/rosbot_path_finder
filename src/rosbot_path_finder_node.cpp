#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include <opencv2/opencv.hpp>
#include <sstream>

#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <iostream>
#include "opencv2/core/core.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Define Constants
#define minTempDiff 10
#define obstacle 65535
#define free 0
#define xMapSize 100
#define yMapSize 100

using namespace cv;
using namespace std;

// Define Global variables
Mat true_map, obstacleMap;
std_msgs::Bool inPosition, solved;
geometry_msgs::Point start, target;					// now_point = my current position; always updating
geometry_msgs::Point next_point, old_point, now_point;						// old_point = 

// Define Functions
void get_map(const sensor_msgs::ImageConstPtr &map);
void myPosition(const geometry_msgs::Pose &pos);
void AreWeThere(const std_msgs::Bool &dis);
//list findPath();// uses global variables : "true_map" (temperature map), "now_point" (current position), "target" (navigetion target)

cv::Mat initiate_map();		//delete
cv::Mat build_temp_map();
bool isReachable ();

int main(int argc, char **argv)
{
	// First thing first
	
	//true_map = initiate_map(true_map);								// leave only frame as black=obsitle, everything else make it free=white
	next_point.x = 1;												// next X point to nav to
	next_point.y = 1;												// next Y point to nav to
	old_point.x = 1;												// old X point nav from
	old_point.y = 1;												// old Y point nav from
	inPosition.data = false;
	solved.data = false;
	target.x = 60;
	target.y = 60;
	
	// ros definitions
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	image_transport::ImageTransport it_(n);
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Pose>("/rosbot/obsticle", 1000);		// send new (X,Y) position
	//image_sub_ = it_.subscribe("/camera/image_raw", 1,   &ImageConverter::imageCb, this);
	image_transport::Subscriber sub1 = it_.subscribe("/rosbot/image_raw", 1, get_map);			// get new obsticels map
	ros::Subscriber sub2 = n.subscribe("/rosbot/dis", 1000, AreWeThere);					// get distance from target
	ros::Subscriber sub3 = n.subscribe("/rosbot/position", 1000, myPosition);					// get my position
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		// -------------------------------------------------------------
		// Navigation code
		true_map = cv::Mat(yMapSize+2, xMapSize+2, CV_16UC1, obstacle);		// set all map as black=obticale
		if (isReachable ())					
		{
			double min, max;
			true_map = build_temp_map();		// build the temperature map, it takes the global mat "obstacleMap"
			Mat temp_map = build_temp_map();
			cv::minMaxLoc(abs(temp_map-true_map), &min, &max);				// calc the diff between the temperature map from each iteretion
			true_map = temp_map;												// true_map = temperature map + obstacles + frame
			while (max > minTempDiff)		// if temp diff between itiretions is bigger that (minTempDiff). do another iteration
			{
				temp_map = build_temp_map();	
				cv::minMaxLoc(abs(temp_map-true_map), &min, &max);
				true_map = temp_map;
			}
				// now we have the tempetature map after some iterations
		
			imshow("Map", obstacleMap );
		
			if(inPosition.data == true)												// did we get to nedt point
			{
				double r = true_map.at<ushort>(next_point.x+1, next_point.y);		// right value
				double l = true_map.at<ushort>(next_point.x-1, next_point.y);		// left value
				double u = true_map.at<ushort>(next_point.x, next_point.y-1);		// up value
				double d = true_map.at<ushort>(next_point.x, next_point.y+1);		// down value
				if (r<l && r<u && r<d)
					next_point.x = r;						// go right
				else if (l<r && l<u && l<d)
					next_point.x = l;						// go left
				else if (u<l && u<r && u<d)
					next_point.y = u;						// go up
				else //if (d<l && d<r && d<u)
					next_point.y = d; 						// go down

				chatter_pub.publish(next_point);						// publish the new point to go to		
				old_point = now_point;									// update old point
			}
		}

		else
			ROS_INFO("The taeget isnt reachable !!! , please define other target");
		// build path


		
		// end of navigetion code
		// -------------------------------------------------------------

		ros::spinOnce();
		loop_rate.sleep();
	}
  return 0;
}

bool isReachable (){
	// check if the target is reachable
	Mat temp_map = true_map;
	temp_map.at<ushort>(start.y, start.x) = 9;
	bool hasChanged = true;
	
	while (!hasChanged)
	{
		hasChanged = false;
		for (int row = 1; row < temp_map.rows; row++)
			for (int col = 1; col < temp_map.cols; col++)
			{
				if (temp_map.at<ushort>(row+1, col)==9)
				{
					if (temp_map.at<ushort>(row+1, col) == 0)
					{
						temp_map.at<ushort>(row+1, col) = 9;
						hasChanged = true;
					}
					if (temp_map.at<ushort>(row-1, col) == 0)
					{
						temp_map.at<ushort>(row-1, col) = 9;
						hasChanged = true;
					}
					if (temp_map.at<ushort>(row, col+1) == 0)
					{
						temp_map.at<ushort>(row, col+1) = 9;
						hasChanged = true;
					}
					if (temp_map.at<ushort>(row, col-1) == 0)
					{
						temp_map.at<ushort>(row, col-1) = 9;
						hasChanged = true;
					}
				}
			}
	}
	if (temp_map.at<ushrot>(target.y, target.x) == 9)
		return true;
	return false;
}

Mat initiate_map(Mat map)
{
	for (int row = 1; row<=map.rows; row++)
		for (int col = 1; col<=map.cols; col++)
			map.at<ushort>(row, col) = 0;
	return map;
}

Mat build_temp_map()
{
	// This function builds the temperature map. change only the free points
	Mat temp_map = obstacleMap;
	for (int row = 1; row<=temp_map.rows; row++)
		for (int col = 1; col<=temp_map.cols; col++)
			if (temp_map.at<ushort>(row,col) != obstacle)			// if this point is free, calc the average
				temp_map.at<ushort>(row, col) = (temp_map.at<ushort>(row+1, col) + temp_map.at<ushort>(row-1, col) + temp_map.at<ushort>(row, col+1) + temp_map.at<ushort>(row, col-1))/4;
	return temp_map;												// return the temperature map
}

void get_map(const sensor_msgs::ImageConstPtr &map)
{
	cv_bridge::CvImagePtr cv_ptr;
	// ROS callback
	// This function gets obstacle map and place Frame around it -> The new map calls "ObstacleMap"
	//	cv::Rect roi( cv::Point( originX, originY ), cv::Size( width, height ));
	Mat temp_map;																	// define temporary map
	cv_ptr = cv_bridge::toCvCopy(map, sensor_msgs::image_encodings::MONO16);		
	obstacleMap = cv_ptr->image.clone();											// convert ro image to openCV image (Mat)
	//temp_map.copyTo(obstacleMap(cv::Rect(1, 1, xMapSize, xMapSize)));			// copy the map recived to obstacle map + frame
	//cv_ptr = cv_bridge::toCvShare(map, "CV_16UC1")-> obstacleMap);
}

void AreWeThere(const std_msgs::Bool &dis)
{
	// ROS callback
	// This function return 1 if we on position, or 0 if we dont
	inPosition.data = dis.data;
}

void myPosition(const geometry_msgs::Pose &pos)
{
	// ROS callback
	//true_map.at<ushort>(me.x, me.y) = free;
	now_point.x = pos.position.x;
	now_point.y = pos.position.y;
	true_map.at<ushort>(now_point.x, now_point.y) = obstacle;
}
