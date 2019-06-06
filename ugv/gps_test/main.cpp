/*
 * Copyright (c) 2017, James Jackson
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "serial.h"
#include "system.h"
#include "uart.h"
#include "vcp.h"
#include "ublox.h"
#include "revo_f4.h"
#include "printf.h"
#include "led.h"

#include "ugv_localization.h"
#include "ugv_drive.h"

UART* uartPtr;

#define UGV_DRIVE_PIN 0 // PWM port 1 for the drive motor
#define UGV_STEER_PIN 1 // PWM port 2 for the steering servo

#define FLEXIPORT_UART UART3
#define MAINPORT_UART UART1

#define LON_TOLERANCE .0001
#define LAT_TOLERANCE .0001

#define KP_THETA = 1

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    uartPtr->put_byte(c);
}


void rx_callback(uint8_t byte)
{
  uartPtr->put_byte(byte);
}

int main()
{
	systemInit();

	UART uart;
	uart.init(&uart_config[MAINPORT_UART], 9600);
	uartPtr = &uart;
	uart.register_rx_callback(rx_callback);  // Uncomment to test callback version

	LED led1;
	led1.init(LED1_GPIO, LED1_PIN);
	LED led2;
	led2.init(LED2_GPIO, LED2_PIN);

	init_printf(NULL, _putc);
	delay(2000);
	printf("Printf initialized!\n\r");
	UGV_LOCALIZATION ugv_localization;
	ugv_localization.init(FLEXIPORT_UART);
  UGV_DRIVE ugv_drive;
  ugv_drive.init(UGV_DRIVE_PIN, UGV_STEER_PIN);

	delay(1000);
  ugv_drive.setDriveSpeed(1);


	do {
    if(ugv_localization.pull_gps()) {
      ugv_localization.print_gps();
      led1.toggle();
    }
    while()
    float delta_lat = COORD_DRIVE_TARGET_N - ugv_localization.lla[0];
    float delta_lon = COORD_DRIVE_TARGET_E - ugv_localization.lla[1];
    theta_r = atan2(delta_lat, delta_lon)*RAD_TO_DEG;
    theta_err = theta_r - ugv_localization.heading;
    ugv_drive.setSteeringAngle(theta_err);

  	} while(delta_lat < LAT_TOLERANCE && delta_lon < LON_TOLERANCE)
    ugv_drive.setDriveSpeed(0);

	}
}
