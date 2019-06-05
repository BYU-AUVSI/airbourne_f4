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

UART* uartPtr;

#define FLEXIPORT_UART UART3
#define MAINPORT_UART UART1

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
	delay(1000);

	while(true) {
		if(ugv_localization.pull_gps()) {
			ugv_localization.print_gps();
			led1.toggle();
		}
	}
}
