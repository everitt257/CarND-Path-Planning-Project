/*
 * costfunction.h
 *
 *  Created on: Aug 10, 2017
 *      Author: xuandong
 */

#ifndef COSTFUNCTION_H_
#define COSTFUNCTION_H_
#include "Car.h"
#include "JMT.h"
#include "helper.h"
#include <cmath>
#include <iostream>

using namespace std;

typedef double (*fp)(vector<JMT> & trajs,vector<Car> & othercars);

inline double safety_cost(vector<JMT> & trajs,vector<Car> & othercars){
	cout << "safety: ";
	double nearest = 5000;
	for(unsigned i=0; i<othercars.size(); i++){
		for(double t=0; t < DURATION; t+=0.02){
			double othercar_s = othercars[i].s + othercars[i].v*t;
			double othercar_d = othercars[i].d;
			double my_s = trajs[0](t);
			double my_d = trajs[1](t);
			double s_difference = my_s - othercar_s;
			double d_difference = my_d - othercar_d;
			double d = sqrt(s_difference*s_difference+d_difference*d_difference);
			if(d < nearest)
				nearest = d;
		}
	}

	return logistic(2*2.5/nearest);
};

inline double efficiency_cost(vector<JMT> & trajs,vector<Car> & othercars){
	cout << "efficiency_cost: ";
	double sum_speed = 0;
	for(double t=0; t < DURATION; t+=0.02){
		double car_v = trajs[0].velocity(t);
		sum_speed += car_v;
	}
	double ave_speed = sum_speed/NSTEP;
	double target_speed = trajs[0].velocity(DURATION);//trajs[0].velocity(DURATION);
	return logistic(fabs(target_speed - 20) / 20);
};

inline double exceed_v_cost(vector<JMT> & trajs,vector<Car> & othercars){
	cout << "exceed_v_cost: ";
	double maxium_v = 0;
	for(double t=0; t < DURATION; t+=0.02){
		double car_v = trajs[0].velocity(t);
		if(car_v > maxium_v){
			maxium_v = car_v;
		}
	}
	if(maxium_v > 20.5)
		return 1;
	else
		return 0;
}
inline double exceed_a_cost(vector<JMT> & trajs,vector<Car> & othercars){
	cout << "exceed_a_cost: ";
	double sum_a = 0;
	for(double t=0; t < DURATION; t+=0.02){
		double car_a = trajs[0].accel(t);
		sum_a += car_a;
	}
	double ave_a = sum_a/NSTEP;
	if(ave_a > 10)
		return 1;
	else
		return 0;
};

inline double exceed_a_dot_cost(vector<JMT> & trajs,vector<Car> & othercars){
	cout << "exceed_a_dot_cost: ";
	double sum_a_dot = 0;
	for(double t=0; t < DURATION; t+=0.02){
		double car_a_dot = trajs[0].jerk(t);
		sum_a_dot += car_a_dot;
	}
	double ave_a_dot = sum_a_dot/NSTEP;
	if(ave_a_dot > 10)
		return 1;
	else
		return 0;
};

class cost_function {
public:

	struct functionPair{
		fp funp;
		double weight;
	};
	cost_function();
	functionPair safety;
	functionPair efficiency;
	functionPair exceed_v;
	functionPair exceed_a;
	functionPair exceed_a_dot;
	vector<functionPair> functionList;

};

#endif /* COSTFUNCTION_H_ */
