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
#include "led.h"

class UGV_LOCALIZATION
{
	public:

		void init(int uart_num);

		void pull_gps(float *lat, float *lon);

	private:
		VCP vcp;
		UART uart;
		UBLOX gps;
		LED led1;
		double lla[3] = {};
	  float vel[3] = {};
	  uint8_t fix_type = 0;
	  uint32_t t_ms;
};

#endif /* UGV_LOCALIZ_H */
