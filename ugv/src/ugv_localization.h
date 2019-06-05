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
		

	private:
		UBLOX _ubx
		PWM_OUT servo_pwm;

};

#endif /* UGV_LOCALIZ_H */
