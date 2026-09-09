#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_transport.h"
#include "com.h"
#include "aideck_parameters.h"

typedef enum {
    CPX_IF_INIT = 0,
    CPX_IF_GOTO_FIXED_COORDINATES = 1,
    CPX_IF_NEW_PARAMETERS = 2,
} CPXInternalFunction_t;

// Function prototypes
static void aideck_receive_cpx_task(void *pvParameters);
void saveReceivedParameters(float x, float y, float z, float batteryP);
static void aideck_send_cpx_task(void *pvParameters);
void sendGotoFixedPositionToStm(float x, float y, float z);
void writeFloatToUint8Array(float value, uint8_t array[], uint16_t position);
float readUint8ArrayToFloat(const uint8_t array[], uint16_t position);

static esp_routable_packet_t rxp;

void aideck_cpx_init() {
    xTaskCreate(aideck_receive_cpx_task, "AiDeck Receive CPX Task", 5000, NULL, 1, NULL);
    xTaskCreate(aideck_send_cpx_task, "AiDeck Send CPX Task", 5000, NULL, 1, NULL);

    ESP_LOGI("AIDECK_CPX", "Initialized");
}

//------------------------------------- RECEIVE ----------------------------------------------------//
static void aideck_receive_cpx_task(void *pvParameters) {
    while (1) {
        com_receive_aideck_app_blocking(&rxp);
        CPXInternalFunction_t intFunction = (CPXInternalFunction_t)rxp.data[0];
        switch (intFunction) {
            case CPX_IF_NEW_PARAMETERS:
                saveReceivedParameters(
                    readUint8ArrayToFloat(rxp.data, 1),
                    readUint8ArrayToFloat(rxp.data, 1+4),
                    readUint8ArrayToFloat(rxp.data, 1+8),
                    readUint8ArrayToFloat(rxp.data, 1+12));
                break;
            default:
                break;
        }
    }
}

void saveReceivedParameters(float x, float y, float z, float batteryP){
    //ESP_LOGI("AIDECK_CPX", "Got parameters: x=%.4f, y=%.4f, z=%.4f, batteryP=%.4f", x, y, z, batteryP);
    Parameters_t received_parameters = {
        .x = x,
        .y = y,
        .z = z,
        .batteryP = batteryP
    };
    parameters_set(&received_parameters);
}


//------------------------------------- SEND ----------------------------------------------------//
static void aideck_send_cpx_task(void *pvParameters) {
    GoToFixPosition_t goto_fix_position = {0};
    while (1) {
        goto_fix_position_get(&goto_fix_position);
        sendGotoFixedPositionToStm(
            goto_fix_position.x,
            goto_fix_position.y,
            goto_fix_position.z
        );
        vTaskDelay(100);
    }
}

static esp_routable_packet_t txp_to_app;
// Send string to app
// TODO add first character to tell which fx should be called
void sendGotoFixedPositionToStm(float x, float y, float z){
    int parameters_count = 3;
    ESP_LOGI("AIDECK_CPX", "Sending goto_fix_position to STM x=%.4f, y=%.4f, z=%.4f\n", x, y, z);

    cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_APP, &txp_to_app.route); // Add route to txp_to_app
    txp_to_app.data[0] = (uint8_t)CPX_IF_GOTO_FIXED_COORDINATES;
    writeFloatToUint8Array(x, txp_to_app.data, 1);
    writeFloatToUint8Array(y, txp_to_app.data, 4+1);
    writeFloatToUint8Array(z, txp_to_app.data, 8+1);
    txp_to_app.dataLength = 1 + parameters_count*4;
    espAppSendToRouterBlocking(&txp_to_app); // Send message
}

//------------------------------------- HELPER FUNCTIONS ----------------------------------------------------//

/**
 * Converts a float into an int32_t by multiplying it by 10,000,
 * then writes the 4 bytes into a uint8_t array in MSB-first order.
 *
 * @param value     Float value to convert
 * @param array     Array to write into
 * @param position  Starting position in the array
 */
void writeFloatToUint8Array(float value, uint8_t array[], uint16_t position){
    int32_t converted_value = (int32_t)(value * 10000.0f);
    uint32_t bytes = (uint32_t)converted_value;

    array[position + 0] = (uint8_t)((bytes >> 24) & 0xFF);
    array[position + 1] = (uint8_t)((bytes >> 16) & 0xFF);
    array[position + 2] = (uint8_t)((bytes >> 8) & 0xFF);
    array[position + 3] = (uint8_t)(bytes & 0xFF);
}

/**
 * Reads 4 MSB-first bytes from a uint8_t array,
 * converts them to an int32_t, and divides by 10,000
 * to recover the original float value.
 *
 * @param array     Array to read from
 * @param position  Starting position in the array
 * @return          Reconstructed float value
 */
float readUint8ArrayToFloat(const uint8_t array[], uint16_t position){
    uint32_t bytes =
        ((uint32_t)array[position + 0] << 24) |
        ((uint32_t)array[position + 1] << 16) |
        ((uint32_t)array[position + 2] << 8)  |
        ((uint32_t)array[position + 3]);

    int32_t converted_value = (int32_t)bytes;

    return (float)converted_value / 10000.0f;
}