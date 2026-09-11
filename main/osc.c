#include "osc.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_osc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "aideck_offset_math.h"

#define OSC_ADDRESS "255.255.255.255"
#define OSC_PORT 9000
#define OSC_BUFFER_SIZE 1024

#define TAG "osc"

static esp_osc_client_t client;

static void sender(void *parameter);
static bool callback(const char *topic, const char *format, esp_osc_value_t *values);
static void receiver(void *parameter);

void osc_start(void){
    esp_osc_init(&client, OSC_BUFFER_SIZE, OSC_PORT);
    xTaskCreatePinnedToCore(sender, "sender", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(receiver, "receiver", 4096, NULL, 10, NULL, 1);
}

static void sender(void *parameter){
    (void)parameter;

    esp_osc_target_t target = esp_osc_target(OSC_ADDRESS, OSC_PORT);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        esp_osc_send(&client, &target, "test", "ihfdsb", 42, (int64_t)84, 3.14f, 6.28, "foo", 3, "bar");
    }
}

static bool callback(const char *topic, const char *format, esp_osc_value_t *values){
    //ESP_LOGI(TAG, "got message: %s (%s)", topic, format);

    // for (size_t i = 0; i < strlen(format); i++) {
    //     switch (format[i]) {
    //     case 'i':
    //         ESP_LOGI(TAG, "==> i: %d", values[i].i);
    //         break;

    //     case 'h':
    //         ESP_LOGI(TAG, "==> h: %lld", values[i].h);
    //         break;

    //     case 'f':
    //         ESP_LOGI(TAG, "==> f: %f", values[i].f);
    //         break;

    //     case 'd':
    //         ESP_LOGI(TAG, "==> d: %f", values[i].d);
    //         break;

    //     case 's':
    //         ESP_LOGI(TAG, "==> s: %s", values[i].s);
    //         break;

    //     case 'b':
    //         ESP_LOGI(TAG, "==> b: %.*s (%d)",
    //                  values[i].bl,
    //                  values[i].b,
    //                  values[i].bl);
    //         break;
    //     }
    // }
    if (strcmp(topic, "/adm/obj/16/azim") == 0) {
        azim(values[0].f);
    }
    else if (strcmp(topic, "/adm/obj/16/elev") == 0) {
        elev(values[0].f);
    }
    else if (strcmp(topic, "/adm/obj/16/dist") == 0) {
        dist(values[0].f);
    }
    else if (strcmp(topic, "/adm/obj/16/aed") == 0) {
        aed(values[0].f, values[1].f, values[2].f);
    }
    else if (strcmp(topic, "/adm/obj/16/x") == 0) {
        x(values[0].f);
    }
    else if (strcmp(topic, "/adm/obj/16/y") == 0) {
        y(values[0].f);
    }
    else if (strcmp(topic, "/adm/obj/16/z") == 0) {
        z(values[0].f);
    }
    else if (strcmp(topic, "/adm/obj/16/xy") == 0) {
        xy(values[0].f, values[1].f);
    }
    else if (strcmp(topic, "/adm/obj/16/xyz") == 0) {
        // just for testing, need to convert into adm space!!!!
        ESP_LOGI(TAG, "got message: %s (%s): %f %f %f", topic, format, values[0].f, values[1].f, values[2].f);
        xyz(values[0].f, values[1].f, values[2].f);
    }
    // else if (strcmp(topic, "/adm/obj/16/w") == 0) {
    // }
    // else if (strcmp(topic, "/adm/obj/16/gain") == 0) {
    // }
    // else if (strcmp(topic, "/adm/obj/16/dref") == 0) {
    // }
    // else if (strcmp(topic, "/adm/obj/16/dmax") == 0) {
    // }
    // else if (strcmp(topic, "/adm/obj/16/mute") == 0) {
    // }
    // else if (strcmp(topic, "/adm/obj/16/name") == 0) {
    // }
    // else if (strcmp(topic, "/adm/lis/xyz") == 0) {
    // }
    // else if (strcmp(topic, "/adm/lis/ypr") == 0) {
    // }
    // else if (strcmp(topic, "/adm/env/change") == 0) {
    // }
    return true;
}

static void receiver(void *parameter){
    (void)parameter;

    while(1){
        esp_osc_receive(&client, callback);
    }
}
