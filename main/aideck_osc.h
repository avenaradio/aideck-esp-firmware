#ifndef AIDECK_OSC_H
#define AIDECK_OSC_H

#include <stdbool.h>

bool osc_send_target(float values[]);
void osc_init(void);

#endif /* AIDECK_OSC_H */
