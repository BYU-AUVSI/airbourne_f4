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
#include "pwm.h"
#include "led.h"

#include "revo_f4.h"

#include "printf.h"
#include "vcp.h"



#define UGV_NUM_PWM 2
#define UGV_FREQ 50
#define UGV_STEER_MAX_PWM 2200
#define UGV_STEER_MIN_PWM 477
#define UGV_PWM_SWEEP 5

#define UGV_STRAIGHT_PWM 1100 // PWM command that makes the wheels straight
#define UGV_LEFT_PWM 1650	  // PWM command that turns the wheels left
#define UGV_RIGHT_PWM 650	  // PWM command that turns the wheels right

#define UGV_MOTOR_STOP 1500

VCP* uartPtr = NULL;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    uartPtr->put_byte(c);
}

void ugv_writeUs(uint32_t val, PWM_OUT *esc_out)
{
	for (int i = 0; i < UGV_NUM_PWM; ++i)
	{
		esc_out[i].writeUs(val);
		delay(5);
	}	
}

void sweepUp(uint32_t min_, uint32_t max_, uint32_t step_, PWM_OUT *esc_out)
{
	static uint32_t throttle = min_;

	for (int i = 0; i < UGV_NUM_PWM; ++i)
	{
		esc_out[i].writeUs(throttle);
		delay(5);
	}
	throttle += step_;
	if (throttle > max_)
	{
		throttle = min_;
	}
}

void sweepDown(uint32_t min_, uint32_t max_, uint32_t step_, PWM_OUT *esc_out)
{
	static uint32_t throttle = max_;

	for (int i = 0; i < UGV_NUM_PWM; ++i)
	{
		esc_out[i].writeUs(throttle);
		delay(5);
	}
	throttle -= step_;
	if (throttle < min_)
	{
		throttle = max_;
	}
}

void sweepUpDown(uint32_t min_, uint32_t max_, uint32_t step_, PWM_OUT *esc_out)
{
	static uint32_t throttle = min_;
	uint32_t delay_ms = 750;
	while (throttle < max_)
	{
		for (int i = 0; i < UGV_NUM_PWM; ++i)
		{
			esc_out[i].writeUs(throttle);
			delay(5);
		}
		throttle += step_;
	}
	delay(delay_ms);
	while (throttle > min_)
	{
		for (int i = 0; i < UGV_NUM_PWM; ++i)
		{
			esc_out[i].writeUs(throttle);
			delay(5);
		}
		throttle -= step_;
	}
	delay(delay_ms);
}

int main()
{
	systemInit();

	VCP vcp;
	vcp.init();
	uartPtr = &vcp;

	init_printf(NULL, _putc);

	LED info;
	info.init(LED2_GPIO, LED2_PIN);

	PWM_OUT esc_out[UGV_NUM_PWM];
	for (int i = 0; i < UGV_NUM_PWM; ++i)
	{
		esc_out[i].init(&pwm_config[i], UGV_FREQ, UGV_STEER_MAX_PWM, UGV_STEER_MIN_PWM);
	}

	while(1)
	{
		 sweepUpDown(UGV_RIGHT_PWM, UGV_LEFT_PWM, UGV_PWM_SWEEP, esc_out);
		ugv_writeUs(1500, esc_out);
		printf("test print\n\r");
		delay(7500);
		for (int i = 1800; i < 2000; i+=10)
		{
			ugv_writeUs(i, esc_out);
			printf("writing us value: %d\n\r", i);
			delay(7500);
			info.toggle();
		}
	}
}
