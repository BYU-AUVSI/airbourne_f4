/** Definition of ugv_localization class
 *
 * Author: Jacob Willis
 * Date Started: 30-May-2019
 */

#include <stdbool.h>
#include "ugv_localization.h"

void UGV_LOCALIZATION::init(int uart_num){

  UART uart;
  uart.init(&uart_config[UART3], 115200);

  gps.init(&uart);
}

bool UGV_LOCALIZATION::pull_gps(){
	if (gps.new_data())
	{
		struct UBLOX::GNSSPVT data;
		data = gps.read();
		printf("got data\n\r");
		lla[0] = (double)data.lat;
		lla[1] = (double)data.lon;
		lla[2] = (double)data.height;
		vel[0] = (double)data.vel_n;
		vel[1] = (double)data.vel_e;
		vel[2] = (double)data.vel_d;
		t_ms = data.time;
		return true;
	}
	return false;
}

void UGV_LOCALIZATION::print_gps(){
    printf("t: %d\tlla: %6.6f, %6.6f, %4.2f\tvel: %3.3f, %3.3f, %3.3f\n\r",
       t_ms, lla[0], lla[1], lla[2], vel[0], vel[1], vel[2]);
}
