/*
 * FSM.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: xuandong
 */

#include "FSM.h"

FSM::FSM() {
	// TODO Auto-generated constructor stub
	Current = FSM_state::KL;
}

vector<FSM_state> FSM::getSuccess_state(){
	if(Current == FSM_state::KL){
		return {FSM_state::KL, FSM_state::LC, FSM_state::RC};
	}
	else if(Current == FSM_state::LC){
		return {FSM_state::KL};
	}
	else{
		return {FSM_state::KL};
	}
}
