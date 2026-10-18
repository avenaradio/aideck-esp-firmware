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
    char landing;
    char avoiding_object;
    char start_failed;
    char battery_empty;
    char position_lost;
} GoToPosition_and_errors_t;
BaseType_t set_goto_position(const GoToPosition_and_errors_t *goto_fix_position);
BaseType_t get_goto_position(GoToPosition_and_errors_t *goto_fix_position);

BaseType_t set_goto_ack(const GoToPosition_and_errors_t *goto_ack);
BaseType_t get_goto_ack(GoToPosition_and_errors_t *goto_ack);

typedef struct{
    float x_max_positive;
    float x_max_negative;
    float y_max_positive;
    float y_max_negative;
    float z_max_positive;
    float z_max_negative;
} Constants_t;
extern Constants_t global_constants;

typedef struct{
    char topic[50];
    char format[4];
    float af;
    float bf;
    float cf;
    char string[50];
} Osc_send_t;
BaseType_t set_osc_send(const Osc_send_t *osc_send_floats);
BaseType_t get_osc_send(Osc_send_t *osc_send_floats);

void global_queues_init(void);

#endif /* AIDECK_GLOBAL_QUEUES_H */
