/*
 * Car.h
 *
 *  Created on: Aug 7, 2017
 *      Author: xuandong
 */

#ifndef CAR_H_
#define CAR_H_
#include <vector>
#include <math.h>
#include <iostream>
#include "helper.h"

using namespace std;

class Car {
public:
	// Car();
	Car(int id, double s, double d, double v);
	// Set lane type base on car's d
	void SetLaneType();
	// Get desired lane in number based on sequent state
	double get_desired_lane(FSM_state nextState);
	// Get desired lane in lane type based on sequent state
	Lane get_desired_laneType(FSM_state nextState);
	// Update last starting point as new starting point
	void UpdateLast(vector<double> s_last, vector<double> d_last);
	// predict car's location
	// vector<double> CarAt(double t);
	// Car's information
	int id;
	double s;
	double d;
	double v;
	Lane current_lane;
	Lane left_lane;
	Lane right_lane;
	vector<double> Last_s;
	vector<double> Last_d;

private:
	// return car's desired d base on lane's type
	double get_desired_d(Lane nextLane);
};

#endif /* CAR_H_ */
