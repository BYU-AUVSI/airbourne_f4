/** Definition of ugv_localization class
 *
 * Author: Jacob Willis
 * Date Started: 30-May-2019
 */


#ifndef UGV_LOCALIZ_H
#define UGV_LOCALIZ_H

#include "ublox.h"

class UGV_LOCALIZATION 
{
	public:

		void init(UBLOX gnss);
		
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
		UBLOX _ubx
		PWM_OUT servo_pwm;

};

#endif /* UGV_LOCALIZ_H */
