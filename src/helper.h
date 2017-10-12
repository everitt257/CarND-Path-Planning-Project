/*
 * helper.h
 *
 *  Created on: Aug 7, 2017
 *      Author: xuandong
 */

#ifndef HELPER_H_
#define HELPER_H_
#include <vector>
#include <cmath>
#include <string>
#include <iostream>
using namespace std;

// helper types
enum class Lane{
	LEFT, MID, RIGHT, UNKNOWN, NONE
};

enum class FSM_state{
	KL, LC, RC, PLC, PRC
};

// constants
const double TIME_INCREMENT = 0.02;
const double DETECTION_DISTANCE = 80;
const double MAXIMUM_D = 2000;
const double BUFFER = 2.5;
const double DURATION = 2;
const int NSTEP = DURATION/TIME_INCREMENT;

// helper functions
inline double logistic(double t){
	return 2.0/(1+exp(-t)) - 1;
};
// helper to debugg
inline void state_out(FSM_state mystate){
	if(mystate == FSM_state::KL)
		cout << "KL" << endl;
	else if(mystate == FSM_state::LC)
		cout << "LC" << endl;
	else if(mystate == FSM_state::RC)
		cout << "RC" << endl;
};


#endif /* HELPER_H_ */
