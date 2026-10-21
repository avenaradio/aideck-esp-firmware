#include "aideck_osc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/FreeRTOSConfig.h"
#include "aideck_global_queues.h"
#include "esp_log.h"

#define TAG "AIDECK_QUEUES"

QueueHandle_t parameters_queue = NULL;
QueueHandle_t ginstruction_queue = NULL;
QueueHandle_t instruction_ack_queue = NULL;
QueueHandle_t osc_send_queue = NULL;

Constants_t global_constants = {0};

void global_parameters_init(void){
    parameters_queue = xQueueCreate(1, sizeof(Parameters_t));
    ginstruction_queue = xQueueCreate(50, sizeof(Instructions_t));
    instruction_ack_queue = xQueueCreate(5, sizeof(Instructions_t));
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

BaseType_t set_instruction(const Instructions_t *instruction){
    if(instruction == NULL || ginstruction_queue == NULL){
        return pdFALSE;
    }
    return xQueueSend(ginstruction_queue, instruction, 100);
}

BaseType_t get_instruction(Instructions_t *instruction){
    if (instruction == NULL || ginstruction_queue == NULL){
        return pdFALSE;
    }
    return xQueueReceive(
        ginstruction_queue,
        instruction,
        5
    );
}

BaseType_t set_instruction_ack(const Instructions_t *instruction){
    if(instruction == NULL || instruction_ack_queue == NULL){
        return pdFALSE;
    }
    return xQueueSend(instruction_ack_queue, instruction, 100);
}

BaseType_t get_instruction_ack(Instructions_t *instruction){
    if (instruction == NULL || instruction_ack_queue == NULL){
        return pdFALSE;
    }
    return xQueueReceive(
        instruction_ack_queue,
        instruction,
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