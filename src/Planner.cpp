/*
 * Planner.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: xuandong
 */

#include "Planner.h"

Planner::Planner(FSM & state, Car & maincar, vector<Car> & other_cars) {
	// TODO Auto-generated constructor stub
	// If some vehicle ahead had been detected, slow down to follow the nearest vehicle

	// By default it should be current state's subsequent states
	// Only testing KL state for now
	cout << "-----------------------" << endl;

	s_start = maincar.Last_s;
	d_start = maincar.Last_d;
	// FSM_state next_state = FSM_state::KL;
	vector<FSM_state> next_states = state.getSuccess_state();
	for(int i = 0; i < next_states.size(); i++){
		// Default behavior
		FSM_state next_state = next_states[i];
		state_out(next_state);
		int nearest_car = CarAhead(next_state, maincar, other_cars);
		int nearest_car2 = CarBelow(next_state, maincar, other_cars);
		vector<double> lastS =  maincar.Last_s;
		vector<double> future_s1 = {lastS[0] + lastS[1]*DURATION, 20, 0};
		vector<double> future_d1 = {maincar.get_desired_lane(next_state), 0, 0};
		double possible_s_front;
		double possible_speed;
		// If Car found ahead
		if(nearest_car > 0) {
			if(next_state == FSM_state::KL){
					possible_s_front = other_cars[nearest_car].s + other_cars[nearest_car].v*DURATION - 2*BUFFER;
					possible_speed = other_cars[nearest_car].v;
					if(other_cars[nearest_car].s - s_start[0] < 20){
						future_s1 = {possible_s_front, possible_speed, 0};
						future_d1 = {other_cars[nearest_car].d, 0, 0};
					}
			}
			else{
				if(maincar.v > other_cars[nearest_car].v && nearest_car2 > 0 && other_cars[nearest_car2].s + 2*BUFFER < maincar.s){
					possible_s_front = other_cars[nearest_car].s + other_cars[nearest_car].v*DURATION + 2*BUFFER;
					possible_speed = other_cars[nearest_car].v;
				}
				else{
					possible_s_front = other_cars[nearest_car].s + other_cars[nearest_car].v*DURATION - 2*BUFFER;
					possible_speed = other_cars[nearest_car].v;
				}
					if(fabs(other_cars[nearest_car].s - s_start[0]) < 20){
						future_s1 = {possible_s_front, possible_speed, 0};
						future_d1 = {other_cars[nearest_car].d, 0, 0};
					}
			}
		}
		cout << "potential d_target: " << future_d1[0] << ", ";
		cout << "potential s_target " << future_s1[0] << endl;
		potential_goals_s.push_back(future_s1);
		potential_goals_d.push_back(future_d1);
	}
	cout << endl;
	cout << "Begin cost evaluating: " << endl;

	// cost evaluating
	double minimum = 2000;
	int optimal_traj_id = -99;
	FSM_state optimal_next_state = FSM_state::KL;
	cost_function myfunctions;
	for(int i=0; i<potential_goals_s.size(); i++){
		state_out(next_states[i]);
		vector< cost_function::functionPair >::iterator funPair = myfunctions.functionList.begin();
		vector< cost_function::functionPair >::iterator endFunPair = myfunctions.functionList.end();
		// generate
		vector<JMT> traj_sd = GeneratePotentialTraj(potential_goals_s[i], potential_goals_d[i]);
		double traj_cost = 0;
		for(;funPair != endFunPair; funPair++){
			double single_cost = funPair->funp(traj_sd, other_cars);
			cout << ", cost = " << single_cost << endl;
			double single_weight = funPair->weight;
			traj_cost += single_cost*single_weight;
		}
		if(traj_cost < minimum){
			optimal_traj_id = i;
			minimum = traj_cost;
			optimal_next_state = next_states[i];
		}
	}

	future_s = potential_goals_s[optimal_traj_id];
	future_d = potential_goals_d[optimal_traj_id];
	state.Current = optimal_next_state;
	cout << "My choosen state: "; state_out(optimal_next_state);
	// Update FSM
	//state.Current = FSM_state::KL;
	cout << "###################" << endl;
}

vector<JMT> Planner::GenerateTraj(){
	JMT s_traj(s_start, future_s, DURATION);
	JMT d_traj(d_start, future_d, DURATION);
	return {s_traj, d_traj};
}

vector<JMT> Planner::GeneratePotentialTraj(vector<double> potential_s, vector<double> potential_d){
	JMT s_traj(s_start, potential_s, DURATION);
	JMT d_traj(d_start, potential_d, DURATION);
	return {s_traj, d_traj};
}

int Planner::CarAhead(FSM_state & next_state, Car & maincar, vector<Car> & other_cars){
	Lane lanetoCompare = maincar.get_desired_laneType(next_state);
	// default return negative id if none close to the range
	int nearest_id = -999;
	double shortest_d = MAXIMUM_D;
	for(int i=0; i<other_cars.size();i++){
		if(other_cars[i].current_lane == lanetoCompare){
			if(fabs(other_cars[i].s - maincar.s) < DETECTION_DISTANCE && (other_cars[i].s - maincar.s) >= 0){
				if(fabs(other_cars[i].s - maincar.s) < shortest_d){
					shortest_d = fabs(other_cars[i].s - maincar.s);
					nearest_id = other_cars[i].id;
				}
			}

		}
	}
	cout << "CarAhead id: " << nearest_id << ", distance: " << shortest_d << endl;
	return nearest_id;
}

int Planner::CarBelow(FSM_state & next_state, Car & maincar, vector<Car> & other_cars){
	Lane lanetoCompare = maincar.get_desired_laneType(next_state);
	// default return negative id if none close to the range
	int nearest_id = -999;
	double shortest_d = MAXIMUM_D;
	for(int i=0; i<other_cars.size();i++){
		if(other_cars[i].current_lane == lanetoCompare){
			if(fabs(other_cars[i].s - maincar.s) < DETECTION_DISTANCE && (maincar.s - other_cars[i].s) > 0){
				if(fabs(other_cars[i].s - maincar.s) < shortest_d){
					shortest_d = fabs(other_cars[i].s - maincar.s);
					nearest_id = other_cars[i].id;
				}
			}

		}
	}
	cout << "Carbelow id: " << nearest_id << ", distance: " << shortest_d << endl;
	return nearest_id;
}
