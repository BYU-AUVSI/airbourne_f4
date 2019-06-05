/** Definition of ugv_drive class
 *
 * Author: Jacob Willis
 * Date Started: 30-May-2019
 */

#ifndef UGV_DRIVE_H
#define UGV_DRIVE_H

#include "pwm.h"

class UGV_DRIVE 
{
	public:

		void init(uint32_t drive_pin, uint32_t servo_pin);
		
		/* setSteeringAngle
		 * given an angle, set the wheels to that angle. 
		 * A positive angle turns the wheels to the right.
		 */
		void setSteeringAngle(double angle);
		
		/* setDriveSpeed
		 * Given a value between [-1, 1], set the drive motor to a cooresponding speed
		 */
		void setDriveSpeed(double delta_t);

		/* kill
		 * set drive and steering servos to centerpoints
		 * turn off pwm to entirely disable both
		 */
		void kill();

	private:
		PWM_OUT drive_pwm;
		PWM_OUT servo_pwm;

};

#endif /* UGV_DRIVE_H */
