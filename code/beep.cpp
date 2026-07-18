#include "beep.hpp"

#include "init.hpp"
#include "zf_common_headfile.hpp"

static int beep_enabled = 1;

void beep_set_enabled(int enabled)
{
    beep_enabled = enabled ? 1 : 0;
    if (!beep_enabled)
        beep_gpio.set_level(0);
}

int beep_is_enabled(void)
{
    return beep_enabled;
}

void beep_short(void)
{
    if (!beep_enabled)
        return;

    beep_gpio.set_level(1);
    system_delay_ms(100);
    beep_gpio.set_level(0);
}

void beep_times(int count)
{
    if (!beep_enabled)
        return;

    for (int i = 0; i < count; i++)
    {
        beep_short();
        if (i + 1 < count)
            system_delay_ms(100);
    }
}

void beep_on(void)
{
    if (!beep_enabled)
        return;

    beep_gpio.set_level(1);
}

void beep_off(void)
{
    beep_gpio.set_level(0);
}
