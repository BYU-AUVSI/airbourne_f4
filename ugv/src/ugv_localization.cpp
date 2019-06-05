/** Definition of ugv_localization class
 *
 * Author: Jacob Willis
 * Date Started: 30-May-2019
 */

#include "ugv_localization.h"

void UGV_LOCALIZATION::init(int uart_num){
  systemInit();

  vcp.init();
  serPtr = &vcp;

  uart.init(&uart_config[UART3], 115200);

  init_printf(NULL, _putc);

  gps.init(&uart);

  led1.init(LED1_GPIO, LED1_PIN);

}

void UGV_LOCALIZATION::pull_gps(float *lat, float *lon){
  while (1)
  {
    if (gps.new_data())
    {
      struct UBLOX::GNSSPVT data;
    data = gps.read();
    printf("t: %d\tlla: %6.6f, %6.6f, %4.2f\tvel: %3.3f, %3.3f, %3.3f\n\r",
       data.time, (float)data.lat, (float)data.lon, (float)data.height,
       (double)data.vel_n, (double)data.vel_e, (double)data.vel_d);
      led1.toggle();
      *lat = (float)data.lat;
      *lon = (float)data.lon;
      return;
    }
  }
}
