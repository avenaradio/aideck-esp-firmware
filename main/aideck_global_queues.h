#ifndef AIDECK_GLOBAL_QUEUES_H
#define AIDECK_GLOBAL_QUEUES_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct{
    float x;
    float y;
    float z;
    float batteryP;
} Parameters_t;
BaseType_t set_parameters(const Parameters_t *parameters);
BaseType_t get_parameters(Parameters_t *parameters);

typedef struct{
    float x;
    float y;
    float z;
} GoToPosition_t;
BaseType_t set_goto_position(const GoToPosition_t *goto_fix_position);
BaseType_t get_goto_position(GoToPosition_t *goto_fix_position);

void global_parameters_init(void);

#endif /* AIDECK_GLOBAL_QUEUES_H */
