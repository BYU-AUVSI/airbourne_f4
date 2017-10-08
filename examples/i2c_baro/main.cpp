#include "system.h"
#include "drv_i2c.h"
#include "ms5611.h"
#include "drv_led.h"
#include "vcp.h"
#include "printf.h"

VCP* vcpPtr = NULL;

static void _putc(void *p, char c)
{
  (void)p; // avoid compiler warning about unused variable
  vcpPtr->put_byte(c);
}

int main() {

  systemInit();

  VCP vcp;
  vcpPtr = &vcp;
  init_printf(NULL, _putc);

  LED warn(LED1_GPIO, LED1_PIN);
  LED info(LED2_GPIO, LED2_PIN);

  delay(500);

  info.on();
  I2C i2c1(I2C2);
  MS5611 baro(&i2c1);

  if (!baro.init())
  {
    while(1)
    {
      warn.toggle();
      delay(100);
    }
  }

  float pressure, temperature;
  while(1)
  {
    info.toggle();
    baro.update();
    baro.read(&pressure, &temperature);
    printf("%d Pa, %d.%d K\n",
           (int32_t)(pressure),
           (int32_t)(temperature),
           (int32_t)(temperature*100)%100);

    delay(10);
  }
}