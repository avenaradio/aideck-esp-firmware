#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/FreeRTOSConfig.h"
#include "aideck_global_queues.h"

QueueHandle_t parameters_queue = NULL;
QueueHandle_t goto_fix_position_queue = NULL;
QueueHandle_t log_queue = NULL;

Constants_t global_constants = {0};

void global_parameters_init(void){
    parameters_queue = xQueueCreate(1, sizeof(Parameters_t));
    goto_fix_position_queue = xQueueCreate(100, sizeof(GoToPosition_t));
    log_queue = xQueueCreate(50, sizeof(log_message_t)); // 100 was too big, killed everything
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
        100
    );
}

BaseType_t set_goto_position(const GoToPosition_t *goto_fix_position){
    if(goto_fix_position == NULL || goto_fix_position_queue == NULL){
        return pdFALSE;
    }
    return xQueueSend(goto_fix_position_queue, goto_fix_position, 100);
}

BaseType_t get_goto_position(GoToPosition_t *goto_fix_position){
    if (goto_fix_position == NULL || goto_fix_position_queue == NULL){
        return pdFALSE;
    }
    return xQueueReceive(
        goto_fix_position_queue,
        goto_fix_position,
        100
    );
}

BaseType_t set_log(const log_message_t *log_message){
    if(log_message == NULL || log_queue == NULL){
        return pdFALSE;
    }
    return xQueueSend(log_queue, log_message, 10);
}

BaseType_t get_log(log_message_t *log_message){
    if (log_message == NULL || log_queue == NULL){
        return pdFALSE;
    }
    return xQueueReceive(
        log_queue,
        log_message,
        10
    );
}