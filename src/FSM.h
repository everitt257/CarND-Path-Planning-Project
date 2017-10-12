/*
 * FSM.h
 *
 *  Created on: Aug 10, 2017
 *      Author: xuandong
 */

#ifndef FSM_H_
#define FSM_H_
#include "helper.h"

class FSM {
public:
	FSM();
	vector<FSM_state> getSuccess_state();
	FSM_state Current;
};

#endif /* FSM_H_ */
