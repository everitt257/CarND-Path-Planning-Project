/*
 * JMT.h
 *
 *  Created on: Aug 7, 2017
 *      Author: xuandong
 */

#ifndef JMT_H_
#define JMT_H_

#include <iostream>
#include "helper.h"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

struct JMT{
	JMT(vector<double> start, vector<double> end, double T);
	vector<double> coeff;
	double operator()(double t);
	double velocity(double t);
	double accel(double t);
	double jerk(double t);
};

inline JMT::JMT(vector<double> start, vector<double> end, double T){
    MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;

	MatrixXd B = MatrixXd(3,1);
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];

	MatrixXd Ai = A.inverse();

	MatrixXd C = Ai*B;

	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}

    coeff = result;
};

inline double JMT::operator ()(double t){
	double result = 0;
	for(unsigned i=0;i<coeff.size();i++)
		result += pow(t,i)*coeff[i];
	return result;
};

inline double JMT::velocity(double t){
	double result = 0;
	for(unsigned i=1;i<coeff.size();i++)
		result += pow(t,i-1)*coeff[i]*i;
	return result;
};

inline double JMT::accel(double t){
	double result = 0;
	for(unsigned i=2;i<coeff.size();i++)
		result += pow(t,i-2)*coeff[i]*i*(i-1);
	return result;
};

inline double JMT::jerk(double t){
	double result = 0;
	for(unsigned i=3;i<coeff.size();i++)
		result += pow(t,i-3)*coeff[i]*i*(i-1)*(i-2);
	return result;
}

#endif /* JMT_H_ */
