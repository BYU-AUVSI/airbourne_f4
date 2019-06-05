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

class UGV_LOCALIZATION
{
	public:

		void init(int uart_num);

		bool pull_gps();
		void print_gps();

	private:
		UART uart;
		UBLOX gps;
		double lla[3] = {};
		double vel[3] = {};
		uint32_t t_ms;
};

#endif /* UGV_LOCALIZ_H */
