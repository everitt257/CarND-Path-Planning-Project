/*
 * Planner.h
 *
 *  Created on: Aug 10, 2017
 *      Author: xuandong
 */

#ifndef PLANNER_H_
#define PLANNER_H_
#include "Car.h"
#include "FSM.h"
#include "JMT.h"
#include <math.h>
#include <vector>
#include "costfunction.h"
using namespace std;

class Planner {
public:
	Planner(FSM & state, Car & maincar, vector<Car> & other_cars);
	vector<JMT> GenerateTraj();
	vector<JMT> GeneratePotentialTraj(vector<double> potential_s, vector<double> potential_d);
	vector< vector<double> > potential_goals_s;
	vector< vector<double> > potential_goals_d;
	vector<double> future_s;
	vector<double> future_d;
	vector<double> s_start;
	vector<double> d_start;
private:
	int CarAhead(FSM_state & next_state, Car & maincar, vector<Car> & other_cars);
	int CarBelow(FSM_state & next_state, Car & maincar, vector<Car> & other_cars);
};

#endif /* PLANNER_H_ */
