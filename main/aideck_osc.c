#include "aideck_osc.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_osc.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "aideck_offset_math.h"

#include "aideck_global_queues.h"
#include "xtensa/config/specreg.h"


#define OSC_IN_PORT 9001
#define OSC_OUT_PORT 9001
#define OSC_OUT_ADDRESS "192.168.178.255" // 192.168.178.255 // 255.255.255.255
#define OSC_BUFFER_SIZE 1024

#define TAG "AIDECK_OSC"

static esp_osc_client_t client;
static esp_osc_target_t target;

int log_enabled = 1;

void osc_send_floats(const char *topic, const char *format, float *values);
static bool osc_callback(const char *topic, const char *format, esp_osc_value_t *values);
static void read_receive_queue_task(void *parameter);
// static void log_sender(void *parameter);

void osc_init(void){
    target = esp_osc_target(OSC_OUT_ADDRESS, OSC_OUT_PORT);
    esp_osc_init(&client, OSC_BUFFER_SIZE, OSC_IN_PORT);
    // xTaskCreate(log_sender, "log_sender", 2048, NULL, 5, NULL);
    xTaskCreatePinnedToCore(read_receive_queue_task, "receiver", 4096, NULL, 10, NULL, 1);
    esp_osc_send(&client, &target, "/drone/output/ready-to-fly", "");
}

//TODO use esp_osc_send_v instead?
void osc_send_floats(const char *topic, const char *format, float *values){
    if (topic == NULL || format == NULL || values == NULL) {
        ESP_LOGW(TAG, "osc_send_floats: OSC send failed, a parameter is NULL.");
        return;
    }
    size_t length = strlen(format);
    bool ok = false;
    switch (length) {
        case 1:
            ok = esp_osc_send(&client, &target, topic, format, values[0]);
            break;
        case 2:
            ok = esp_osc_send(&client, &target, topic, format, values[0], values[1]);
            break;
        case 3:
            ok = esp_osc_send(&client, &target, topic, format, values[0], values[1], values[2]);
            break;
        default:
            ok = esp_osc_send(&client, &target, topic, format);
            break;
    }
    if (!ok) {
        ESP_LOGW(TAG, "OSC send failed: %s (%s)", topic, format);
    }
}

bool osc_send_target(float values[]){
    return esp_osc_send(&client, &target, "/drone/output/internal-target-pos-in-m", "fff", values[0], values[1], values[2]);
}

static bool osc_callback(const char *topic, const char *format, esp_osc_value_t *values){
    ESP_LOGI(TAG, "Got message: %s (%s)", topic, format);
    // ESP_LOGI(TAG, "Constants are: x_max: %.3f %.3f, y_max: %.3f %.3f", global_constants.x_max_positive, global_constants.x_max_negative, global_constants.y_max_positive, global_constants.y_max_negative);
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
    
    esp_osc_send(&client, &target, "/drone/output/osc_received" , "s", topic);

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
            //bool result = 
            osc_send_floats(topic, "fff", values);
            // ESP_LOGI(TAG, "OSC send topic=%s format=%s result=%s", topic, format, result ? "success" : "failed");
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
            //bool result = 
            osc_send_floats(topic, "fff", values);
            // ESP_LOGI(TAG, "OSC send topic=%s format=%s result=%s", topic, format, result ? "success" : "failed");
        }
    }
    else if (strcmp(topic, "/adm/obj/16/dmax") == 0) {
        if (strcmp(format, "f") == 0) {
            //ESP_LOGI(TAG, "got message: %s (%s): %f", topic, format, values[0].f);
            dmax(values[0].f);
        } else {
            float values[1] = {get_dmax()};
            //bool result = 
            osc_send_floats(topic, "f", values);
            // ESP_LOGI(TAG, "OSC send topic=%s format=%s result=%s", topic, format, result ? "success" : "failed");
        }
    }
    // DRONE SPECIFIC OSC
    else if (strcmp(topic, "/drone/control/current-position") == 0) {
        get_coords_from_parameters(&polar, &cart);
        float values[3] = {cart.x, cart.y, cart.z};
        osc_send_floats(topic, "fff", values);
    }
    else if (strcmp(topic, "/drone/control/battery-status") == 0) {
        Parameters_t params = {0};
        get_parameters(&params);
        float values[1] = {params.batteryP};
        osc_send_floats(topic, "f", values);
        ESP_LOGI(TAG, "Current params: xyz: %.2f %.2f %.2f, battery: %.2f", params.x, params.y, params.y, params.batteryP);
    }
    else if (strcmp(topic, "/drone/control/land") == 0) {
        GoToPosition_t land_now ={0.0f, 0.0f, -1.0f}; // Control will land if z is <= 0
        set_goto_position(&land_now);
    }
    return true;
}

static void read_receive_queue_task(void *parameter)
{
    (void)parameter;

    for (;;) {
        bool ok = esp_osc_receive(&client, osc_callback);

        if (!ok) {
            ESP_LOGW(TAG, "receiver: OSC send failed.");
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

