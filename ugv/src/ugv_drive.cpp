/** Definition of ugv_drive class
 *
 * Author: Jacob Willis
 * Date Started: 30-May-2019
 */

#include "ugv_drive.h"
#include "pwm.h"
#include "printf.h"


#define UGV_FREQ 50

#define UGV_STRAIGHT_PWM 1100 // PWM command that makes the wheels straight
#define UGV_LEFT_PWM 1650	  // PWM command that turns the wheels left
#define UGV_RIGHT_PWM 650	  // PWM command that turns the wheels right
#define UGV_MAX_STEERING_ANGLE 35 // degrees, the max angle to the left/right the wheels can steer
#define UGV_STEER_MAX_PWM UGV_LEFT_PWM
#define UGV_STEER_MIN_PWM UGV_RIGHT_PWM

#define UGV_DRIVE_STOP 1500
#define UGV_DRIVE_MAX_PWM 2000
#define UGV_DRIVE_MIN_PWM 1000


/* saturateSteeringAngle
 * Given an angle value make sure it is within [-UGV_MAX_STEERING_ANGLE, UGV_MAX_STEERING_ANGLE].
 */
static double saturateSteeringAngle(double angle)
{
	if (angle > UGV_MAX_STEERING_ANGLE)
	{
		return UGV_MAX_STEERING_ANGLE;
	}
	else if (angle < -UGV_MAX_STEERING_ANGLE)
	{
		return -UGV_MAX_STEERING_ANGLE;
	}
	else
	{
		return angle;
	}
}

void UGV_DRIVE::init(uint32_t drive_pin, uint32_t servo_pin)
{
	drive_pwm.init(&pwm_config[drive_pin], UGV_FREQ, UGV_DRIVE_MAX_PWM, UGV_DRIVE_MIN_PWM);
	drive_pwm.writeUs(UGV_DRIVE_STOP);

	servo_pwm.init(&pwm_config[servo_pin], UGV_FREQ, UGV_STEER_MAX_PWM, UGV_STEER_MIN_PWM);
	servo_pwm.writeUs(UGV_STRAIGHT_PWM);
}


/* setDriveSpeed
 * Given a value between [-1, 1], set the drive motor to a cooresponding speed
 */
void UGV_DRIVE::setDriveSpeed(double delta_t)
{
	uint32_t throttle = UGV_DRIVE_STOP + (int)(delta_t*(UGV_DRIVE_MAX_PWM - UGV_DRIVE_STOP));
	drive_pwm.writeUs(throttle);
	//printf("wrote %d to drive\n\r", throttle);
}

/* setSteeringAngle
 * given an angle, set the wheels to that angle. 
 * A positive angle turns the wheels to the right.
 */
void UGV_DRIVE::setSteeringAngle(double angle)
{
	angle = saturateSteeringAngle(angle);
	double delta_a = angle/UGV_MAX_STEERING_ANGLE;
	uint32_t setpoint = UGV_STRAIGHT_PWM + (int) (delta_a*(UGV_LEFT_PWM - UGV_STRAIGHT_PWM));
	servo_pwm.writeUs(setpoint);
	//printf("wrote %d to steer\n\r", setpoint);
}

/* kill
 * set drive and steering servos to centerpoints
 * turn off pwm to entirely disable both
 */
void UGV_DRIVE::kill()
{
	setDriveSpeed(0.0);
	setSteeringAngle(0.0);
	servo_pwm.disable();
	drive_pwm.disable();
}
