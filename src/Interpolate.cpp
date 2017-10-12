/*
 * Interpolate.cpp
 *
 *  Created on: Aug 7, 2017
 *      Author: xuandong
 */

#include "Interpolate.h"

using namespace std;

Interpolate::Interpolate() {

	  // Waypoint map to read from
	  string map_file_ = "../data/highway_map.csv";
	  // The max s value before wrapping around the track back to 0
	  double max_s = 6945.554;

	  ifstream in_map_(map_file_.c_str(), ifstream::in);

	  string line;
	  while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		//cout << "map_waypoints_s: " << s << endl;
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	  }

}

void Interpolate::Update(double x, double y){
	vector<double> s_list;
	vector<double> x_list;
	vector<double> y_list;
	vector<double> dx_list;
	vector<double> dy_list;

	int start_index = ClosestWaypoint(x, y, map_waypoints_x, map_waypoints_y) - 6;
	//cout << "s(closest): " << map_waypoints_s[start_index + 6] << endl;
    for(int i=0; i<24; i++){
      int myindex;
      double s_map;

      if(start_index < 0){
        myindex = map_waypoints_x.size() + start_index-1;
        s_map = map_waypoints_s[myindex];
        s_map = s_map - map_waypoints_s[map_waypoints_s.size()-1];
      }
      else{
        myindex = start_index % map_waypoints_x.size();
        s_map = map_waypoints_s[myindex];
      }
			//cout << "s_map: " << s_map << endl;
      s_list.push_back(s_map);
      x_list.push_back(map_waypoints_x[myindex]);
      y_list.push_back(map_waypoints_y[myindex]);
      dx_list.push_back(map_waypoints_dx[myindex]);
      dy_list.push_back(map_waypoints_dy[myindex]);

      start_index++;
    }
    nearest_waypoints_s = s_list;
    nearest_waypoints_x = x_list;
    nearest_waypoints_y = y_list;
    nearest_waypoints_dx = dx_list;
    nearest_waypoints_dy = dy_list;

    this->splineX.set_points(nearest_waypoints_s, nearest_waypoints_x);
    this->splineY.set_points(nearest_waypoints_s, nearest_waypoints_y);
    this->splineDx.set_points(nearest_waypoints_s, nearest_waypoints_dx);
    this->splineDy.set_points(nearest_waypoints_s, nearest_waypoints_dy);
}

vector<double> Interpolate::myXY(double s, double d){
	double x = splineX(s) + splineDx(s)*d;
	double y = splineY(s) + splineDy(s)*d;
	return {x,y};
}

vector< vector<double> > Interpolate::generate_pathXY(JMT & s_traj, JMT & d_traj, int n){
	// TODO: generate x, y vectors with s_traj and d_traj
	vector<double> x;
	vector<double> y;
	for(int i=0; i<n; i++){
		double s = s_traj(TIME_INCREMENT*i);
		double d = d_traj(TIME_INCREMENT*i);
		x.push_back(myXY(s,d)[0]);
		y.push_back(myXY(s,d)[1]);
	}

	return {x,y};
}

double Interpolate::distance(double x1, double y1, double x2, double y2){
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int Interpolate::ClosestWaypoint(double x, double y, vector<double> & maps_x, vector<double> & maps_y){
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			  closestLen = dist;
			  closestWaypoint = i;
		}

	}

	return closestWaypoint;
}
