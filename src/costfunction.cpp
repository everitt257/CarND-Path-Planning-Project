/*
 * costfunction.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: xuandong
 */

#include "costfunction.h"


cost_function::cost_function(){
	// link
	safety.funp = &safety_cost;
	efficiency.funp = &efficiency_cost;
	exceed_v.funp = &exceed_v_cost;
	exceed_a.funp = &exceed_a_cost;
	exceed_a_dot.funp = &exceed_a_dot_cost;

	// weight assign
	safety.weight = 1.5;
	efficiency.weight = 1;
	exceed_v.weight = 1;
	exceed_a.weight = 1;
	exceed_a_dot.weight = 1;

	this->functionList = {safety, efficiency, exceed_v, exceed_a, exceed_a_dot};
}
