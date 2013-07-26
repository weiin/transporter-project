/*
 * An EPOS interface to control both motors of a differential drive robot
 *
 */

//#define DEBUG
#include "transporter/transporter_driver.h"
#include <math.h>

Transporter::Transporter ()
{
} //Transporter constructor

Transporter::~Transporter ()
{
	shutDown();
} //Transporter destructor

bool Transporter::start()
{
	leftmotor_fd=leftMotor.openPort(LEFT_MOTOR.c_str());
	rightmotor_fd=rightMotor.openPort(RIGHT_MOTOR.c_str());
	usleep(10000);
	
	if(!leftMotor.FaultReset()) return false;
	if(!rightMotor.FaultReset()) return false;
	usleep(10000);
	leftMotor.MoveToEnableOperation();
	rightMotor.MoveToEnableOperation();
	usleep(10000);
	setParams(&leftMotor);
	setParams(&rightMotor);
	printf("Motors ready to move\n");
	return true;
}
void Transporter::shutDown()
{
	printf("\t-----Shutting down robot...-----\n");
	moveVelocity(0,0);
	if(leftMotor.closePort())
	{
		printf("Left motor %i has been shutdown properly\n",leftmotor_fd);
	}
	else printf("Left motor %i was not shutdown\n",leftmotor_fd);
	if(rightMotor.closePort())
	{
		printf("Right motor %i has been shutdown properly\n",rightmotor_fd);
	}
	else printf("Right motor %i was not shutdown\n",rightmotor_fd);
}

void Transporter::setParams(EPOS* motor)
{
	motor->SetCurrentRegulatorIGain(CurrentRegulatorIGain);        
	motor->SetCurrentRegulatorPGain(CurrentRegulatorPGain);
	motor->SetMotorMaxContCurrent(MotorMaxContinuousCurrent);
	motor->SetMotorMaxPeakCurrent((unsigned short)(MotorMaxContinuousCurrent * 2));
	motor->SetPositionDGain(PositionDGain);
	motor->SetPositionIGain(PositionIGain);
	motor->SetPositionPGain(PositionPGain);
	motor->SetPositionProfileAcceleration(PositionProfileAcceleration);
	motor->SetPositionProfileDeceleration(PositionProfileDeceleration);
	motor->SetPositionProfileVelocity(PositionProfileVelocity);
	motor->SetMaxProfileVelocity(MaxProfileVelocity);
	motor->SetVelocityIGain(VelocityIGain);
	motor->SetVelocityPGain(VelocityPGain);
	motor->SetMaxFollowError(MaxFollowError);
	motor->SetPositionWindow(PositionProfileWindow);
	motor->SetPositionWindowTime(PositionProfileWindowTime);
	motor->SetCurrentModeSetting(CurrentModeSetting);
	motor->SetThermalTimeConstantWinding(ThermalTimeConstantWinding);

	motor->GetPosition();
	motor->GetVelocity();
}

void Transporter::moveVelocity(double linear, double angular)
{
	int v,w;
	v=(int)( (linear*1000)/(M_PI*WHEEL_DIA*wheel_circum_correction) * 60 * GEAR_RATIO );
	w=(int)( angular*0.5*AXLE_LEN*wheel_base_correction/(M_PI*WHEEL_DIA*wheel_circum_correction) * 60 * GEAR_RATIO );

#ifdef DEBUG
	printf("Move at vx=%f, w=%f -> n=%i, %i \n", linear,angular,v,w);
#endif

	leftMotor.SetOperationMode(EPOS::PROFILE_VELOCITY);
	rightMotor.SetOperationMode(EPOS::PROFILE_VELOCITY);
	leftMotor.SetMotionProfileType(EPOS::TRAPEZOIDAL);
	rightMotor.SetMotionProfileType(EPOS::TRAPEZOIDAL);

	leftMotor.SetTargetVelocity(v-w);
	rightMotor.SetTargetVelocity(-v-w);

	//set bit 8 of controlword to 0 to start motion
	unsigned short cwd1,cwd2;
	leftMotor.GetControlWord();
	cwd1 = (unsigned short)(leftMotor.ControlWord & 0xfeff);
	rightMotor.GetControlWord();
	cwd2 = (unsigned short)(rightMotor.ControlWord & 0xfeff);
	leftMotor.SetControlWord(cwd1);
	rightMotor.SetControlWord(cwd2);

/*	//set bit 8 of controlword to 1 to stop axle
	rightMotor.SetControlWord(cwd2 | 0x0100);
	leftMotor.SetControlWord(cwd1 | 0x0100);
*/
} // moveVelocity(double linear, double angular)

void Transporter::moveDistance(double dist)
{
	int d;
	//convert dist in mm to motor encoder steps
	d=(int)( dist/(M_PI*WHEEL_DIA*wheel_circum_correction) * STEPS_PER_REV * GEAR_RATIO );

#ifdef DEBUG
	printf("Move dist=%f mm -> n=%i \n", dist,d);
#endif

	leftMotor.SetOperationMode(EPOS::PROFILE_POSITION);
	rightMotor.SetOperationMode(EPOS::PROFILE_POSITION);
	leftMotor.SetMotionProfileType(EPOS::TRAPEZOIDAL);
	rightMotor.SetMotionProfileType(EPOS::TRAPEZOIDAL);

	leftMotor.SetTargetPosition(d);
	rightMotor.SetTargetPosition(-d);

	//set bits 4,5,6 to '1' to assume target pos, set immediately, target pos is relative value
	//set bit 8 of controlword to 0 to start motion
	unsigned short cwd1,cwd2;
	leftMotor.GetControlWord();
	cwd1 = (unsigned short)( (leftMotor.ControlWord|0x0070) & 0xfeff );
	rightMotor.GetControlWord();
	cwd2 = (unsigned short)( (rightMotor.ControlWord|0x0070) & 0xfeff );
	leftMotor.SetControlWord(cwd1);
	rightMotor.SetControlWord(cwd2);

	unsigned short stat1, stat2;
	do // keep moving until target reached (bit 10 of status)
	{
		leftMotor.GetStatusWord();
		stat1 = (leftMotor.StatusWord & 0x0400)>>10;
		rightMotor.GetStatusWord();
		stat2 = (rightMotor.StatusWord & 0x0400)>>10;
	} while (stat1==0 || stat2 == 0);

} // void Transporter::moveDistance(double dist)

void Transporter::moveAngle(double angle)
{
	int th;
	//convert angle in degrees to motor encoder steps
	th=(int)( angle/180.0 * 0.5*AXLE_LEN*wheel_base_correction / (WHEEL_DIA*wheel_circum_correction) * STEPS_PER_REV * GEAR_RATIO );

#ifdef DEBUG
	printf("Move angle=%f degrees -> n=%i \n", angle,th);
#endif

	leftMotor.SetOperationMode(EPOS::PROFILE_POSITION);
	rightMotor.SetOperationMode(EPOS::PROFILE_POSITION);
	leftMotor.SetMotionProfileType(EPOS::TRAPEZOIDAL);
	rightMotor.SetMotionProfileType(EPOS::TRAPEZOIDAL);

	 // wheels have to turn negative dir for robot to turn positive angle
	leftMotor.SetTargetPosition(-th);
	rightMotor.SetTargetPosition(-th);

	//set bits 4,5,6 to '1' to assume target pos, set immediately, target pos is relative value
	//set bit 8 of controlword to 0 to start motion
	unsigned short cwd1,cwd2;
	leftMotor.GetControlWord();
	cwd1 = (unsigned short)( (leftMotor.ControlWord|0x0070) & 0xfeff );
	rightMotor.GetControlWord();
	cwd2 = (unsigned short)( (rightMotor.ControlWord|0x0070) & 0xfeff );
	leftMotor.SetControlWord(cwd1);
	rightMotor.SetControlWord(cwd2);

	unsigned short stat1, stat2;
	do // keep moving until target reached (bit 10 of status)
	{
		leftMotor.GetStatusWord();
		stat1 = (leftMotor.StatusWord & 0x0400)>>10;
		rightMotor.GetStatusWord();
		stat2 = (rightMotor.StatusWord & 0x0400)>>10;
	} while (stat1==0 || stat2 == 0);

} // void Transporter::moveAngle(double angle)

void Transporter::getDisplacement(double* distance, double* angle)
{
	int cur_Lpos, cur_Rpos;
	double left_mm, right_mm;

	prev_Lpos = leftMotor.ActualPosition;
	prev_Rpos = rightMotor.ActualPosition;

	leftMotor.GetPosition();
	rightMotor.GetPosition();
	cur_Lpos = leftMotor.ActualPosition;
	cur_Rpos = rightMotor.ActualPosition;

	// distance travelled by each wheel
	left_mm = (double)(cur_Lpos-prev_Lpos) / (STEPS_PER_REV*GEAR_RATIO) * M_PI*WHEEL_DIA*wheel_circum_correction;
	right_mm = (double)(cur_Rpos-prev_Rpos) / (STEPS_PER_REV*GEAR_RATIO) * M_PI*WHEEL_DIA*wheel_circum_correction;

//note: the following calculations are approximations of the robot motion
	// displacement of robot
	*distance = (( left_mm - right_mm )/2.0) / 1000.0; //in m
		//note: using left_encoder-right_encoder because they are opposite signs.
	// rotation of robot
	*angle = -( left_mm + right_mm )/AXLE_LEN * odom_angular_scale_correction; //in rad
		//note: taking negative because defined left turn (rotate around z) as positive angle (right hand rule).

#ifdef DEBUG
	printf("left wheel travelled [%f]mm, right wheel travelled [%f]mm \n",left_mm,right_mm);
	printf("distance travelled by robot: [%f]mm, angle [%f]rad \n",*distance,*angle);
#endif

}
