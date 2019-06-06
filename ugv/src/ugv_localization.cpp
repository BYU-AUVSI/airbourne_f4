/** Definition of ugv_localization class
 *
 * Author: Jacob Willis
 * Date Started: 30-May-2019
 */

#include <stdbool.h>
#include <math.h>
#include "ugv_localization.h"
#include "led.h"
#include "ublox.h"


void UGV_LOCALIZATION::init(UBLOX* gps_){

  this->gps = gps_;

}

bool UGV_LOCALIZATION::pull_gps(){
	if (this->gps->new_data())
	{
		struct UBLOX::GNSSPVT data;
		data = this->gps->read();
		printf("got data\n\r");
		lla[0] = (double)data.lat;
		lla[1] = (double)data.lon;
		lla[2] = (double)data.height;
		vel[0] = (double)data.vel_n;
		vel[1] = (double)data.vel_e;
		vel[2] = (double)data.vel_d;
		heading = atan2(vel[0], vel[1])*RAD_TO_DEG;
		t_ms = data.time;
		return true;
	}
	return false;
}

void UGV_LOCALIZATION::print_gps(){
    printf("t: %d\tlla: %6.6f, %6.6f, %4.2f\tvel: %3.3f, %3.3f, %3.3f\n\r",
       t_ms, lla[0], lla[1], lla[2], vel[0], vel[1], vel[2]);
}
