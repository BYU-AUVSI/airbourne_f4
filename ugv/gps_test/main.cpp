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

#include <math.h>

#include "serial.h"
#include "system.h"
#include "uart.h"
#include "vcp.h"
#include "ublox.h"
#include "revo_f4.h"
#include "printf.h"
#include "led.h"

#include "ugv_localization.h"
#include "ugv_coodinates.h"
#include "ugv_drive.h"

UART* uartPtr;
Serial *serPtr = NULL;

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

	UART comm_uart;
	comm_uart.init(&uart_config[MAINPORT_UART], 9600);
	uartPtr = &comm_uart;
	comm_uart.register_rx_callback(rx_callback);  // Uncomment to test callback version
	init_printf(NULL, _putc);
	delay(2000);
	printf("Printf initialized!\n\r");


	LED led1;
	led1.init(LED1_GPIO, LED1_PIN);
	LED led2;
	led2.init(LED2_GPIO, LED2_PIN);

	UART gps_uart;
	gps_uart.init(&uart_config[FLEXIPORT_UART], 115200);
	UBLOX gps_;
	gps_.init(&gps_uart);
	UGV_LOCALIZATION ugv_localization;
	ugv_localization.init(&gps_);

	UGV_DRIVE ugv_drive;
	ugv_drive.init(UGV_DRIVE_PIN, UGV_STEER_PIN);

	printf("Initialization completed\n\r");
	ugv_drive.setDriveSpeed(.3);

	float delta_lat;
	float delta_lon;

	do {
		if(ugv_localization.pull_gps()) {
			ugv_localization.print_gps();
			printf("attempting to reach %6.6f lat, %6.6f lon\n\r", COORD_DRIVE_TARGET_N, COORD_DRIVE_TARGET_E);
			led1.toggle();
		}

		float delta_lat = COORD_DRIVE_TARGET_N - ugv_localization.lla[0];
		float delta_lon = COORD_DRIVE_TARGET_E - ugv_localization.lla[1];
		float theta_r = atan2(delta_lat, delta_lon)*RAD_TO_DEG;
		float theta_err = theta_r - ugv_localization.heading;
		ugv_drive.setSteeringAngle(theta_err);

		delay(50);
		led2.toggle();
	} while(delta_lat < LAT_TOLERANCE && delta_lon < LON_TOLERANCE);

	ugv_drive.kill();

	led2.on();
	led1.off();


	while(true) {
		led2.toggle();
		led1.toggle();
		printf("Killed\n\r");
		delay(250);
	}

}

