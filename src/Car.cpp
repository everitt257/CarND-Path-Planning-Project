/*
 * Car.cpp
 *
 *  Created on: Aug 7, 2017
 *      Author: xuandong
 */

#include "Car.h"

//Car::Car() {
//	// TODO Auto-generated constructor stub
//
//}

Car::Car(int id, double s, double d, double v){
	this->id = id;
	this->s = s;
	this->d = d;
	this->v = v;
	this->SetLaneType();
}

void Car::SetLaneType(){
	// left
	if(d>0 && d<=4){
		this->current_lane = Lane::LEFT;
		this->left_lane = Lane::NONE;
		this->right_lane = Lane::MID;
	}
	// mid
	else if(d>4 && d<=8){
		this->current_lane = Lane::MID;
		this->left_lane = Lane::LEFT;
		this->right_lane = Lane::RIGHT;
	}
	// right
	else if(d>8 && d<=12){
		this->current_lane = Lane::RIGHT;
		this->left_lane = Lane::MID;
		this->right_lane = Lane::NONE;
	}
	else{
		this->current_lane = Lane::UNKNOWN;
		this->left_lane = Lane::UNKNOWN;
		this->right_lane = Lane::UNKNOWN;
	}
}

double Car::get_desired_d(Lane nextLane){
	if(nextLane == Lane::MID)
		return 6;
	else if(nextLane == Lane::LEFT)
		return 2;
	else if(nextLane == Lane::RIGHT)
		return 10;
	else
		return 6;
}

double Car::get_desired_lane(FSM_state nextState){
	if(nextState == FSM_state::KL){
		return get_desired_d(this->current_lane);
	}
	else if(nextState == FSM_state::LC){
		return get_desired_d(this->left_lane);
	}
	else if(nextState == FSM_state::RC){
		return get_desired_d(this->right_lane);
	}
	else
		return 6;
}

Lane Car::get_desired_laneType(FSM_state nextState){
	if(nextState == FSM_state::KL){
		return current_lane;
	}
	else if(nextState == FSM_state::LC){
		return left_lane;
	}
	else if(nextState == FSM_state::RC){
		return right_lane;
	}
	else
		return Lane::UNKNOWN;
}

void Car::UpdateLast(vector<double> s_last, vector<double> d_last){
	this->Last_s = s_last;
	this->Last_d = d_last;
}

// vector<double> Car::CarAt(double t){
// 	return{s + v*t, d};
// }
