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

#include "system.h"
#include "led.h"
#include "ugv_drive.h"

#include "revo_f4.h"

#include "printf.h"
#include "uart.h"

#include <Eigen/Core>


#define UGV_DRIVE_PIN 0 // PWM port 1 for the drive motor
#define UGV_STEER_PIN 1 // PWM port 2 for the steering servo

UART* uartPtr = NULL;

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
	uart.init(&uart_config[UART1], 9600);
	uartPtr = &uart;
    uart.register_rx_callback(rx_callback);  // Uncomment to test callback version

	init_printf(NULL, _putc);
	printf("Printf initialized!\n\r");

	LED info;
	info.init(LED2_GPIO, LED2_PIN);
	info.on();

	UGV_DRIVE drive;
	drive.init(UGV_DRIVE_PIN, UGV_STEER_PIN);

	delay(5000);

	// straight ahead
	printf("straight ahead\n\r");
	drive.setSteeringAngle(0.0);
	drive.setDriveSpeed(0.4);
	delay(2000);

	info.toggle();
	// stop
	printf("stop\n\r");
	drive.setDriveSpeed(0.0);
	delay(1000);

	info.toggle();
	// turn wheels left
	printf("turn wheels left\n\r");
	drive.setSteeringAngle(20);
	delay(1000);

	info.toggle();
	// drive 1 sec
	printf("drive 1 sec\n\r");
	drive.setDriveSpeed(.4);
	delay(1000);
	
	info.toggle();
	// stop
	printf("killing\n\r");
	drive.kill();
	delay(1000);


	// done
	while(1)
	{
		info.toggle();
		printf("stopped\n\r");
		delay(500);
	}
}
