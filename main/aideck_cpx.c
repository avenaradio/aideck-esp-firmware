#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_transport.h"
#include "com.h"
#include "aideck_global_queues.h"
#include "aideck_osc.h"

#define TAG "AIDECK_CPX"

typedef enum {
    CPX_IF_INIT = 0,
    CPX_IF_GOTO_FIXED_COORDINATES = 1,
    CPX_IF_NEW_PARAMETERS = 2,
    CPX_IF_GOTO_ACK = 3,
    CPX_IF_CONSTANTS = 4,
    CPX_IF_BATTERY_EMPTY = 5,
    CPX_IF_LANDING = 6,
    CPX_IF_AVOIDING_OBJECT = 7,
    CPX_IF_START_FAILED = 8,
    CPX_IF_POSITION_LOST = 9,
} CPXInternalFunction_t;

static esp_routable_packet_t txp_to_app;
static esp_routable_packet_t rxp;

bool constants_received_ok = false;
bool battery_is_empty = false;

// Function prototypes
static void aideck_receive_cpx_task(void *pvParameters);
void write_parameters_to_queue(float x, float y, float z, float batteryP);
static void aideck_send_cpx_task(void *pvParameters);
void send_goto_position_to_stm(float x, float y, float z);
void send_empty_IF(CPXInternalFunction_t internal_function);
void write_float_to_uint8_array(float value, uint8_t array[], uint16_t position);
float read_uin8_array_to_float(const uint8_t array[], uint16_t position);

void aideck_cpx_init() {
    xTaskCreate(aideck_receive_cpx_task, "AiDeck Receive CPX Task", 5000, NULL, 1, NULL);
    xTaskCreate(aideck_send_cpx_task, "AiDeck Send CPX Task", 5000, NULL, 1, NULL);
    ESP_LOGI(TAG, "aideck_cpx_init done");
}

//------------------------------------- SEND ----------------------------------------------------//
static void aideck_send_cpx_task(void *pvParameters) {
    GoToPosition_and_errors_t goto_position = {0};
    while (1) {
        if (get_goto_position(&goto_position) == pdPASS) {
            //ESP_LOGI(TAG, "Got goto_fix_position_get: x=%.4f, y=%.4f, z=%.4f", goto_fix_position.x, goto_fix_position.y, goto_fix_position.z);
            if (goto_position.landing) {
                send_empty_IF(CPX_IF_LANDING);
            } else {
                send_goto_position_to_stm(goto_position.x, goto_position.y, goto_position.z);
            }
        }
        vTaskDelay(10);
    }
}


void send_goto_position_to_stm(float x, float y, float z){
    int parameters_count = 3;
    ESP_LOGI(TAG, "Sending goto_fix_position to STM x=%.4f, y=%.4f, z=%.4f", x, y, z);
    cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_APP, &txp_to_app.route); // Add route to txp_to_app
    txp_to_app.data[0] = (uint8_t)CPX_IF_GOTO_FIXED_COORDINATES;
    write_float_to_uint8_array(x, txp_to_app.data, 1);
    write_float_to_uint8_array(y, txp_to_app.data, 4+1);
    write_float_to_uint8_array(z, txp_to_app.data, 8+1);
    txp_to_app.dataLength = 1 + parameters_count*4;
    espAppSendToRouterBlocking(&txp_to_app); // Send message
}

void send_empty_IF(CPXInternalFunction_t internal_function){
    cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_APP, &txp_to_app.route); // Add route to txp_to_app
    txp_to_app.data[0] = (uint8_t)internal_function;
    txp_to_app.dataLength = 1;
    espAppSendToRouterBlocking(&txp_to_app); // Send message
}

//------------------------------------- RECEIVE ----------------------------------------------------//
static void aideck_receive_cpx_task(void *pvParameters) {
    while (1) {
        GoToPosition_and_errors_t ack = {0};
        com_receive_aideck_app_blocking(&rxp);
        CPXInternalFunction_t intFunction = (CPXInternalFunction_t)rxp.data[0];
        switch (intFunction) {
            case CPX_IF_NEW_PARAMETERS:
                write_parameters_to_queue(
                    read_uin8_array_to_float(rxp.data, 1),
                    read_uin8_array_to_float(rxp.data, 1+4),
                    read_uin8_array_to_float(rxp.data, 1+8),
                    read_uin8_array_to_float(rxp.data, 1+12));
                break;
            case CPX_IF_GOTO_ACK: {
                ack.x = read_uin8_array_to_float(rxp.data, 1);
                ack.y = read_uin8_array_to_float(rxp.data, 1+4);
                ack.z = read_uin8_array_to_float(rxp.data, 1+8);
                set_goto_ack(&ack);
                break;
            }
            case CPX_IF_CONSTANTS: {
                if (!constants_received_ok){
                    Constants_t received_constants = {
                        .x_max_positive = read_uin8_array_to_float(rxp.data,1),
                        .x_max_negative = read_uin8_array_to_float(rxp.data,4+1),
                        .y_max_positive = read_uin8_array_to_float(rxp.data,8+1),
                        .y_max_negative = read_uin8_array_to_float(rxp.data,12+1),
                        .z_max_positive = read_uin8_array_to_float(rxp.data,16+1),
                        .z_max_negative = read_uin8_array_to_float(rxp.data,20+1)
                    };
                    ESP_LOGI(TAG, "Got global_consts: x:%.2f %.2f, y: %.2f %.2f, z: %.2f %.2f",
                        received_constants.x_max_positive,
                        received_constants.x_max_negative,
                        received_constants.y_max_positive,
                        received_constants.y_max_negative,
                        received_constants.z_max_positive,
                        received_constants.z_max_negative
                    );
                    global_constants = received_constants;
                    constants_received_ok = true;
                }
                send_empty_IF(CPX_IF_INIT);
                break;
            }
            case CPX_IF_BATTERY_EMPTY:
                if (!battery_is_empty) {
                    battery_is_empty = true;
                    ack.battery_empty = 1;
                    set_goto_ack(&ack);
                }
                break;
            case CPX_IF_LANDING: {
                ack.landing = 1;
                set_goto_ack(&ack);
                break;
            }
            case CPX_IF_AVOIDING_OBJECT: {
                ack.avoiding_object = 1;
                set_goto_ack(&ack);
                break;
            }
            case CPX_IF_START_FAILED: {
                ack.start_failed = 1;
                set_goto_ack(&ack);
                break;
            }
            case CPX_IF_POSITION_LOST: {
                ack.position_lost = 1;
                set_goto_ack(&ack);
                break;
            }
            default:
                break;
        }
    }
}

void write_parameters_to_queue(float x, float y, float z, float batteryP){
    //ESP_LOGI(TAG, "Got parameters: x=%.4f, y=%.4f, z=%.4f, batteryP=%.4f", x, y, z, batteryP);
    Parameters_t received_parameters = {
        .x = x,
        .y = y,
        .z = z,
        .batteryP = batteryP
    };
    set_parameters(&received_parameters);
}

//------------------------------------- HELPER FUNCTIONS ----------------------------------------------------//

void write_float_to_uint8_array(float value, uint8_t array[], uint16_t position){
    if (sizeof(float) != 4){
        return;
    }
    memcpy(&array[position], &value, sizeof(float));
}

float read_uin8_array_to_float(const uint8_t array[],uint16_t position){
    float value;
    if (sizeof(float) != 4){
        return 0.0f;
    }
    memcpy(&value, &array[position], sizeof(value));
    return value;
}
