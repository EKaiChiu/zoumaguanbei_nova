#ifndef BEEP_HPP
#define BEEP_HPP

void beep_set_enabled(int enabled);
int beep_is_enabled(void);
void beep_short(void);
void beep_times(int count);
void beep_on(void);
void beep_off(void);

#endif
