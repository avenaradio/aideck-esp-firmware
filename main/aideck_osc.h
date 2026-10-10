#ifndef AIDECK_OSC_H
#define AIDECK_OSC_H

#include <stdbool.h>

bool osc_send_battery_empty(void);
bool osc_send_landing(void);
bool osc_send_target(float values[]);
void osc_send_floats(const char *topic, const char *format, float *values);
void osc_init(void);

#endif /* AIDECK_OSC_H */
