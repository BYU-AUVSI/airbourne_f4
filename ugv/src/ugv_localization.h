/** Definition of ugv_localization class
 *
 * Author: Jacob Willis
 * Date Started: 30-May-2019
 */


#ifndef UGV_LOCALIZ_H
#define UGV_LOCALIZ_H

#include "serial.h"
#include "system.h"
#include "uart.h"
#include "vcp.h"
#include "ublox.h"
#include "revo_f4.h"
#include "printf.h"

#define RAD_TO_DEG 180/3.14

class UGV_LOCALIZATION
{
	public:

		void init(UBLOX* gps_);

		bool pull_gps();
		void print_gps();

		UBLOX* gps;
		double lla[3] = {};
		double vel[3] = {};
		double heading;
		uint32_t t_ms;
};

#endif /* UGV_LOCALIZ_H */
