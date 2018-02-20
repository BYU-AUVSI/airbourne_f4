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

#ifndef RC_H
#define RC_H

#include "system.h"

#include "rc.h"
#include "gpio.h"
#include "uart.h"

class RC_SBUS
{

private:

  enum
  {
    START_BYTE = 0x0F,
    END_BYTE = 0x00
  };

  typedef enum
  {
    SBUS_SIGNAL_OK,
    SBUS_SIGNAL_LOST,
    SBUS_SIGNAL_FAILSAFE
  } failsafe_state_t;

  failsafe_state_t failsafe_status_;

  GPIO* inv_pin_;
  UART* uart_;
  uint32_t raw_[18];
  uint32_t last_pulse_ms_ = 0;
  uint8_t buffer_[25];
  uint8_t buffer_pos_ = 0;

  void decode_buffer();

public:
  void init(GPIO *inv_pin, UART *uart);
  void read_cb(uint8_t byte);
  uint32_t read(uint8_t channel);
  bool lost();

};

#endif // RC_H
