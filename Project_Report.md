## Path Planning Project Report
### Model Description
#### General Description
The model make use of finite state machine, JMT trajectory generation, cost functions evaluation and third order cubic spline curves for interpolating. The car is able to drive it self in a sparse traffic environment without breaking the rules most of times. Note that I made little change to `CMakeLists.txt`.

#### Finite state machine
I made three states available for the car to be in. There are the `KeepLane` state, where the car aims to drive as fast as possible before seeing another car in front. If it sees another car in front, it would follow that car's speed but kept a distance away from it.

The two other states are the `ChangeLeft` and `ChangeRight`. These are the two states where I make left lane change or right lane change. It's has two actions at most. Either it follows a car and stay behind it, or it surpasses that car but ending up with that car's speed.

However I do see some defects with using three states. The car occasionally collide with other car. But I have no idea how to implement the `PrepareChangeLeft` nor the `PrepareChangeRight` case. I mean I tried implementing it several times but it just ends up performing worse than before.
#### Trajectory Generation
The trajectory were generated with `JMT` curves that minimize jerks. This is pretty much the same as in class except I made a little class for it.
```
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
```
I also want to evaluate acceleration and jerks so I added these functions inside my `JMT.h`

#### Interpolate
After trajectory for S and D were generated with `JMT`, I tried generating a smooth curve with the `spline.h` library. Basically I took the nearest 24 waypoints to generate spline for `X(s)`,`Y(s)`,`Dx(s)`,`Dy(s)`.

```
vector<double> Interpolate::myXY(double s, double d){
	double x = splineX(s) + splineDx(s)*d;
	double y = splineY(s) + splineDy(s)*d;
	return {x,y};
}
```

#### Behavior Planner
This is probably the most confusing part. So what I did was to generate trajectories based on the consequent states from current state and the nearest vehicle ahead of my vehicle. I saved possible future goal for `S` and `D` and evaluated the `JMT` trajectories with `costfunctions` I mimicked from the `python trajectory exercise`. So far the `safety` cost function doesn't work so good.
```
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
```

#### Link to video
Link to video: https://youtu.be/O-Nm5T2OmAE

### Conclusion
This is a very challenging project for me. I struggled really hard to even get the vehicle to drive smoothly at constant speed. Then I found `Behavior Planning` quite difficult to implement. Hopefully this project pass so I can move on.
