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


#define OSC_IN_PORT 4001
#define OSC_OUT_PORT 4002
#define OSC_OUT_ADDRESS "255.255.255.255" // 192.168.178.255 // 255.255.255.255
#define OSC_BUFFER_SIZE 1024

#define TAG "osc"

static esp_osc_client_t client;
static esp_osc_target_t target;

//static void sender(void *parameter);
static bool callback(const char *topic, const char *format, esp_osc_value_t *values);
static void receiver(void *parameter);

void osc_start(void){
    target = esp_osc_target(OSC_OUT_ADDRESS, OSC_OUT_PORT);
    esp_osc_init(&client, OSC_BUFFER_SIZE, OSC_IN_PORT);
    // xTaskCreatePinnedToCore(sender, "sender", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(receiver, "receiver", 4096, NULL, 10, NULL, 1);
}

// static void sender(void *parameter){
//     (void)parameter;
//     for (;;) {
//         vTaskDelay(pdMS_TO_TICKS(1000));

//         esp_osc_send(&client, &target, "test", "ihfdsb", 42, (int64_t)84, 3.14f, 6.28, "foo", 3, "bar");
//     }
// }

bool osc_send_floats(const char *topic, const char *format, float *values){
    if (topic == NULL || format == NULL || values == NULL) {
        return false;
    }
    size_t length = strlen(format);
    switch (length) {
        case 1:
            return esp_osc_send(&client, &target, topic, format, values[0]);
        case 2:
            return esp_osc_send(&client, &target, topic, format, values[0], values[1]);
        case 3:
            return esp_osc_send(&client, &target, topic, format, values[0], values[1], values[2]);
        default:
            return false;
    }
}

static bool callback(const char *topic, const char *format, esp_osc_value_t *values){
    ESP_LOGI(TAG, "got message: %s (%s)", topic, format);
    AdmPolar_t polar = {0};
    AdmCartesian_t cart = {0};

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
        if (strcmp(format, "f") == 0) {
            azim(values[0].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            float values[1] = {polar.azimuth};
            osc_send_floats(topic, "f", values);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/elev") == 0) {
        if (strcmp(format, "f") == 0) {
            elev(values[0].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            float values[1] = {polar.elevation};
            osc_send_floats(topic, "f", values);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/dist") == 0) {
        if (strcmp(format, "f") == 0) {
            dist(values[0].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            float values[1] = {polar.distance};
            osc_send_floats(topic, "f", values);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/aed") == 0) {
        if (strcmp(format, "fff") == 0) {
            //ESP_LOGI(TAG, "got message: %s (%s): %f %f %f", topic, format, values[0].f, values[1].f, values[2].f);
            aed(values[0].f, values[1].f, values[2].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            float values[3] = {polar.azimuth, polar.elevation, polar.distance};
            bool result = osc_send_floats(topic, "fff", values);
            ESP_LOGI(TAG, "OSC send topic=%s format=%s result=%s", topic, format, result ? "success" : "failed");
        }
    }
    else if (strcmp(topic, "/adm/obj/16/x") == 0) {
        if (strcmp(format, "f") == 0) {
            x(values[0].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            float values[1] = {cart.x};
            osc_send_floats(topic, "f", values);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/y") == 0) {
        if (strcmp(format, "f") == 0) {
            y(values[0].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            float values[1] = {cart.y};
            osc_send_floats(topic, "f", values);
        }
        y(values[0].f);
    }
    else if (strcmp(topic, "/adm/obj/16/z") == 0) {
        if (strcmp(format, "f") == 0) {
            z(values[0].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            float values[1] = {cart.z};
            osc_send_floats(topic, "f", values);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/xy") == 0) {
        if (strcmp(format, "ff") == 0) {
            xy(values[0].f, values[1].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            float values[2] = {cart.x, cart.y};
            osc_send_floats(topic, "ff", values);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/xyz") == 0) {
        if (strcmp(format, "fff") == 0) {
            //ESP_LOGI(TAG, "got message: %s (%s): %f %f %f", topic, format, values[0].f, values[1].f, values[2].f);
            xyz(values[0].f, values[1].f, values[2].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            float values[3] = {cart.x, cart.y, cart.z};
            bool result = osc_send_floats(topic, "fff", values);
            ESP_LOGI(TAG, "OSC send topic=%s format=%s result=%s", topic, format, result ? "success" : "failed");
        }
    }
    else if (strcmp(topic, "/adm/obj/16/dmax") == 0) {
        if (strcmp(format, "f") == 0) {
            //ESP_LOGI(TAG, "got message: %s (%s): %f", topic, format, values[0].f);
            dmax(values[0].f);
        } else {
            float values[1] = {get_dmax()};
            bool result = osc_send_floats(topic, "f", values);
            ESP_LOGI(TAG, "OSC send topic=%s format=%s result=%s", topic, format, result ? "success" : "failed");
        }
    }
    return true;
}

static void receiver(void *parameter){
    (void)parameter;

    while(1){
        esp_osc_receive(&client, callback);
    }
}
