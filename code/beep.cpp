#include "beep.hpp"

#include "init.hpp"
#include "zf_common_headfile.hpp"

void beep_short(void)
{
    beep_gpio.set_level(1);
    system_delay_ms(100);
    beep_gpio.set_level(0);
}

void beep_on(void)
{
    beep_gpio.set_level(1);
}

void beep_off(void)
{
    beep_gpio.set_level(0);
}
