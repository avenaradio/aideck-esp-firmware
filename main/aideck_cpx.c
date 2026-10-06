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
    CPX_IF_CONSTANTS = 4
} CPXInternalFunction_t;

bool constants_received_ok = false;

// Function prototypes
static void aideck_receive_cpx_task(void *pvParameters);
void write_parameters_to_queue(float x, float y, float z, float batteryP);
static void aideck_send_cpx_task(void *pvParameters);
void send_goto_position_to_stm(float x, float y, float z);
void send_init_done();
void write_float_to_uint8_array(float value, uint8_t array[], uint16_t position);
float read_uin8_array_to_float(const uint8_t array[], uint16_t position);

static esp_routable_packet_t rxp;

void aideck_cpx_init() {
    xTaskCreate(aideck_receive_cpx_task, "AiDeck Receive CPX Task", 5000, NULL, 1, NULL);
    xTaskCreate(aideck_send_cpx_task, "AiDeck Send CPX Task", 5000, NULL, 1, NULL);

    ESP_LOGI(TAG, "aideck_cpx_init done");
}

//------------------------------------- RECEIVE ----------------------------------------------------//
static void aideck_receive_cpx_task(void *pvParameters) {
    while (1) {
        com_receive_aideck_app_blocking(&rxp);
        CPXInternalFunction_t intFunction = (CPXInternalFunction_t)rxp.data[0];
        float target_position[3] = {0};
        switch (intFunction) {
            case CPX_IF_NEW_PARAMETERS:
                write_parameters_to_queue(
                    read_uin8_array_to_float(rxp.data, 1),
                    read_uin8_array_to_float(rxp.data, 1+4),
                    read_uin8_array_to_float(rxp.data, 1+8),
                    read_uin8_array_to_float(rxp.data, 1+12));
                break;
            case CPX_IF_GOTO_ACK:
                    target_position[0] = read_uin8_array_to_float(rxp.data, 1);
                    target_position[1] = read_uin8_array_to_float(rxp.data, 1+4);
                    target_position[2] = read_uin8_array_to_float(rxp.data, 1+8);
                osc_send_target(target_position);
                break;
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
                send_init_done();
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


//------------------------------------- SEND ----------------------------------------------------//
static void aideck_send_cpx_task(void *pvParameters) {
    GoToPosition_t goto_fix_position = {0};
    while (1) {
        if (get_goto_position(&goto_fix_position) == pdPASS) {
            //ESP_LOGI(TAG, "Got goto_fix_position_get: x=%.4f, y=%.4f, z=%.4f", goto_fix_position.x, goto_fix_position.y, goto_fix_position.z);
            send_goto_position_to_stm(goto_fix_position.x, goto_fix_position.y, goto_fix_position.z);
        }
        vTaskDelay(10);
    }
}

static esp_routable_packet_t txp_to_app;
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

void send_init_done(){
    cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_APP, &txp_to_app.route); // Add route to txp_to_app
    txp_to_app.data[0] = (uint8_t)CPX_IF_INIT;
    txp_to_app.dataLength = 1;
    espAppSendToRouterBlocking(&txp_to_app); // Send message
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
