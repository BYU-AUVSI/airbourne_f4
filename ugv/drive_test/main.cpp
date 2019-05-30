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
#include "uart.h"



#define UGV_NUM_PWM 2
#define UGV_FREQ 50
#define UGV_DRIVE_PIN 0 // PWM port 1 for the drive motor
#define UGV_STEER_PIN 1 // PWM port 2 for the steering servo

#define UGV_STRAIGHT_PWM 1100 // PWM command that makes the wheels straight
#define UGV_LEFT_PWM 1650	  // PWM command that turns the wheels left
#define UGV_RIGHT_PWM 650	  // PWM command that turns the wheels right
#define UGV_MAX_STEERING_ANGLE 35 // degrees, the max angle to the left/right the wheels can steer
#define UGV_STEER_MAX_PWM UGV_LEFT_PWM
#define UGV_STEER_MIN_PWM UGV_RIGHT_PWM

#define UGV_DRIVE_STOP 1500
#define UGV_DRIVE_MAX_PWM 2000
#define UGV_DRIVE_MIN_PWM 1000

UART* uartPtr = NULL;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    uartPtr->put_byte(c);
}


/* setDriveSpeed
 * Given a value between [-1, 1], set the drive motor to a cooresponding speed
 */
void setDriveSpeed(double delta_t, PWM_OUT drive_pwm)
{
	uint32_t throttle = UGV_DRIVE_STOP + (int)(delta_t*(UGV_DRIVE_MAX_PWM - UGV_DRIVE_STOP));
	drive_pwm.writeUs(throttle);
	printf("wrote %d to drive\n\r", throttle);
}

/* saturateSteeringAngle
 * Given an angle value make sure it is within [-UGV_MAX_STEERING_ANGLE, UGV_MAX_STEERING_ANGLE].
 */
double saturateSteeringAngle(double angle)
{
	if (angle > UGV_MAX_STEERING_ANGLE)
	{
		return UGV_MAX_STEERING_ANGLE;
	}
	else if (angle < -UGV_MAX_STEERING_ANGLE)
	{
		return -UGV_MAX_STEERING_ANGLE;
	}
	else
	{
		return angle;
	}
}


/* setSteeringAngle
 * given an angle, set the wheels to that angle. 
 * A positive angle turns the wheels to the right.
 */
void setSteeringAngle(double angle, PWM_OUT servo_pwm)
{
	angle = saturateSteeringAngle(angle);
	double delta_a = angle/UGV_MAX_STEERING_ANGLE;
	uint32_t setpoint = UGV_STRAIGHT_PWM + (int) (delta_a*(UGV_LEFT_PWM - UGV_STRAIGHT_PWM));
	servo_pwm.writeUs(setpoint);
	printf("wrote %d to steer\n\r", setpoint);
}

/* killDrive
 * set drive and steering servos to centerpoints
 * turn off pwm to entirely disable both
 */
void killDrive(PWM_OUT servo_pwm, PWM_OUT drive_pwm)
{
	setDriveSpeed(0.0, drive_pwm);
	setSteeringAngle(0.0, steer_pwm);
	steer_pwm.disable();
	drive_pwm.disable();
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

	PWM_OUT drive_pwm_out;
	drive_pwm_out.init(&pwm_config[UGV_DRIVE_PIN], UGV_FREQ, UGV_DRIVE_MAX_PWM, UGV_DRIVE_MIN_PWM);
	drive_pwm_out.writeUs(UGV_DRIVE_STOP);

	PWM_OUT steer_pwm_out;
	steer_pwm_out.init(&pwm_config[UGV_STEER_PIN], UGV_FREQ, UGV_STEER_MAX_PWM, UGV_STEER_MIN_PWM);
	steer_pwm_out.writeUs(UGV_STRAIGHT_PWM);

	delay(5000);

	// straight ahead
	printf("straight ahead\n\r");
	setSteeringAngle(0.0, steer_pwm_out);
	setDriveSpeed(0.4, drive_pwm_out);
	delay(2000);

	info.toggle();
	// stop
	printf("stop\n\r");
	setDriveSpeed(0.0, drive_pwm_out);
	delay(1000);

	info.toggle();
	// turn wheels left
	printf("turn wheels left\n\r");
	setSteeringAngle(20, steer_pwm_out);
	delay(1000);

	info.toggle();
	// drive 1 sec
	printf("drive 1 sec\n\r");
	setDriveSpeed(.4, drive_pwm_out);
	delay(1000);
	
	info.toggle();
	// stop
	printf("killing\n\r");
	killDrive(steer_pwm_out, drive_pwm_out);
	delay(1000);


	// done
	while(1)
	{
		info.toggle();
		printf("stopped\n\r");
		delay(500);
	}
}
