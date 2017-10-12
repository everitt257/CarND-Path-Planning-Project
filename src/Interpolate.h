/*
 * Interpolate.h
 *
 *  Created on: Aug 7, 2017
 *      Author: xuandong
 */

#ifndef INTERPOLATE_H_
#define INTERPOLATE_H_
#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include "spline.h"
#include "JMT.h"
#include "helper.h"
using namespace std;

class Interpolate {
public:
	// Save all waypoints data, taken from main.cpp
	Interpolate();
	// Setup nearest waypoints data, given the main car's x and y
	void Update(double x, double y);
	// return x,y value pair based on car's s and d,
	vector<double> myXY(double s, double d);
	// make path consist of x and y
	vector< vector<double> > generate_pathXY(JMT & s_traj, JMT & d_traj, int n);
	// way points used to generate spline
	vector<double> nearest_waypoints_x;
	vector<double> nearest_waypoints_y;
	vector<double> nearest_waypoints_s;
	vector<double> nearest_waypoints_dx;
	vector<double> nearest_waypoints_dy;
	vector<double> nearest_s;
	// spline used to generate x,y
	tk::spline splineX;
	tk::spline splineY;
	tk::spline splineDx;
	tk::spline splineDy;
private:
	// taken from main.cpp
	double distance(double x1, double y1, double x2, double y2);
	// taken from main.cpp
	int ClosestWaypoint(double x, double y, vector<double> & map_waypoints_x, vector<double> & map_waypoints_y);
	// taken from main.cpp
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;
};

#endif /* INTERPOLATE_H_ */
