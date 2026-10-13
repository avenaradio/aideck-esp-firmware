#include "aideck_osc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/FreeRTOSConfig.h"
#include "aideck_global_queues.h"
#include "esp_log.h"

#define TAG "AIDECK_QUEUES"

QueueHandle_t parameters_queue = NULL;
QueueHandle_t goto_fix_position_queue = NULL;
QueueHandle_t goto_ack_queue = NULL;
QueueHandle_t osc_send_queue = NULL;

Constants_t global_constants = {0};

void global_parameters_init(void){
    parameters_queue = xQueueCreate(1, sizeof(Parameters_t));
    goto_fix_position_queue = xQueueCreate(50, sizeof(GoToPosition_and_errors_t));
    goto_ack_queue = xQueueCreate(5, sizeof(GoToPosition_and_errors_t));
    osc_send_queue = xQueueCreate(25, sizeof(Osc_send_t));
}

BaseType_t set_parameters(const Parameters_t *parameters){
    if(parameters == NULL || parameters_queue == NULL){
        return pdFALSE;
    }
    // Queue length is 1, so this replaces the previous value.
    return xQueueOverwrite(parameters_queue, parameters);
}

BaseType_t get_parameters(Parameters_t *parameters){
    if (parameters == NULL || parameters_queue == NULL){
        return pdFALSE;
    }
    // Peek keeps the latest value in the queue.
    return xQueuePeek(
        parameters_queue,
        parameters,
        2
    );
}

BaseType_t set_goto_position(const GoToPosition_and_errors_t *goto_fix_position){
    if(goto_fix_position == NULL || goto_fix_position_queue == NULL){
        return pdFALSE;
    }
    return xQueueSend(goto_fix_position_queue, goto_fix_position, 100);
}

BaseType_t get_goto_position(GoToPosition_and_errors_t *goto_fix_position){
    if (goto_fix_position == NULL || goto_fix_position_queue == NULL){
        return pdFALSE;
    }
    return xQueueReceive(
        goto_fix_position_queue,
        goto_fix_position,
        5
    );
}

BaseType_t set_goto_ack(const GoToPosition_and_errors_t *goto_ack){
    if(goto_ack == NULL || goto_ack_queue == NULL){
        return pdFALSE;
    }
    return xQueueSend(goto_ack_queue, goto_ack, 100);
}

BaseType_t get_goto_ack(GoToPosition_and_errors_t *goto_ack){
    if (goto_ack == NULL || goto_ack_queue == NULL){
        return pdFALSE;
    }
    return xQueueReceive(
        goto_ack_queue,
        goto_ack,
        5
    );
}

BaseType_t set_osc_send(const Osc_send_t *osc_send_floats){
    if(osc_send_floats == NULL || osc_send_queue == NULL){
        return pdFALSE;
    }
    return xQueueSend(osc_send_queue, osc_send_floats, 5);
}

BaseType_t get_osc_send(Osc_send_t *osc_send_floats){
    if (osc_send_floats == NULL || osc_send_queue == NULL){
        return pdFALSE;
    }
    return xQueueReceive(
        osc_send_queue,
        osc_send_floats,
        5
    );
}