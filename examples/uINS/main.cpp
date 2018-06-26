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
#include "uart.h"
#include "uINS.h"
#include "vcp.h"
#include "revo_f4.h"
#include "printf.h"

VCP* uartPtr = NULL;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    uartPtr->put_byte(c);
}


int main()
{
    systemInit();
    
    VCP vcp;
    vcp.init();
    uartPtr = &vcp;
    init_printf(NULL, _putc);
    
    UART uart;
    uart.init(&uart_config[UART1], 3000000);
    
    uINS uins;
    uins.init(&uart);
    
    float ned[3];
    float uvw[3];
    float q[4];
    float acc[3];
    float pqr[3];
    float mag[3];
    float baro;
    uint32_t time;
    
    uint32_t last_print_ms = millis();
    while(1)
    {
        if (millis() - last_print_ms > 30)
        {
            if (uins.present())
            {
                uins.read_INS(ned, uvw, q, &time);
                uins.read_IMU(pqr, acc, &time);
                uins.read_other_sensors(mag, &baro, &time);
            }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
            printf("ned: %.3f, %.3f, %.3f", ned[0], ned[1], ned[2]);
            printf(" | uvw: %.3f, %.3f, %.3f", uvw[0], uvw[1], uvw[2]);
            printf(" | q: %.3f, %.3f, %.3f, %.3f\n", q[0], q[1], q[2], q[3]);
            printf("acc: %.3f, %.3f, %.3f", acc[0], acc[1], acc[2]);
            printf(" | pqr: %.3f, %.3f, %.3f", pqr[0], pqr[1], pqr[2]);
            printf(" | mag: %.3f, %.3f, %.3f", mag[0], mag[1], mag[2]);
            printf("baro: %.3f, time: %dms\n\n", baro, time);
            last_print_ms = millis();
#pragma GCC diagnostic pop
        }
    }
}
