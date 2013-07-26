#include "transporter/EPOS.h"

#ifndef TRANSPORTER_DRIVER
#define TRANSPORTER_DRIVER

class Transporter
{
	EPOS leftMotor, rightMotor;
	int leftmotor_fd,rightmotor_fd;
	int prev_Lpos, prev_Rpos;
public:
	Transporter ();
	~Transporter ();

	std::string LEFT_MOTOR;
	std::string RIGHT_MOTOR;
	int AXLE_LEN; //in mm
	int WHEEL_DIA;
	int GEAR_RATIO;
	int STEPS_PER_REV;
	double wheel_circum_correction;
	double wheel_base_correction;
	double odom_angular_scale_correction;

	int CurrentRegulatorPGain;
	int CurrentRegulatorIGain;
	int MotorMaxContinuousCurrent;
	int PositionPGain;
	int PositionIGain;
	int PositionDGain;
	int PositionProfileAcceleration;
	int PositionProfileDeceleration;
	int PositionProfileVelocity;
	int MaxProfileVelocity;
	int VelocityPGain;
	int VelocityIGain;
	int MaxFollowError;
	int PositionProfileWindow;
	int PositionProfileWindowTime;
	int CurrentModeSetting;
	int ThermalTimeConstantWinding;

	bool start();
	void shutDown();
	void setParams(EPOS* m);
	void moveVelocity(double vx,double w); // [m/s,rad/s]
	void moveDistance(double x); // [m]
	void moveAngle(double theta); // [degrees]
	void getDisplacement(double* distance, double* angle); // [m, rad]

}; //class Transporter

#endif
