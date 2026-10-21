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
#define OSC_OUT_ADDRESS "192.168.1.255" // 192.168.178.255  192.168.1.255  149.222.206.227
#define OSC_BUFFER_SIZE 1024
// #define BROADCAST_IP "149.222.206.227"
// #define BROADCAST_PORT 9000

#define TAG "AIDECK_OSC"

static esp_osc_client_t client;
static esp_osc_target_t target;
// static esp_osc_target_t broadcast_target;

bool battery_empty = false;

static bool osc_callback(const char *topic, const char *format, esp_osc_value_t *values);
static void read_receive_queue_task(void *parameter);
static void ack_sender_task(void *parameter);

void osc_init(void){
    target = esp_osc_target(OSC_OUT_ADDRESS, OSC_OUT_PORT);
    // broadcast_target = esp_osc_target(BROADCAST_IP, BROADCAST_PORT);
    esp_osc_init(&client, OSC_BUFFER_SIZE, OSC_IN_PORT);
    vTaskDelay(500);
    xTaskCreate(read_receive_queue_task, "receiver", 4096, NULL, 10, NULL);
    xTaskCreate(ack_sender_task, "ack_sender", 4096, NULL, 5, NULL);
}

static void ack_sender_task(void *parameter){
    Instructions_t ack = {0};
    Osc_send_t osc_send_message = {0};
    // esp_osc_send(&client, &broadcast_target, "/drone/output/register-broadcast", "");
    vTaskDelay(5);
    esp_osc_send(&client, &target, "/drone/output/ready", "");
    while (1) {
        if (get_instruction_ack(&ack) == pdPASS) {
            if (ack.battery_empty) {
                esp_osc_send(&client, &target, "/drone/output/battery-empty", "");
            } else if (ack.landing) {
                esp_osc_send(&client, &target, "/drone/output/landing", "");
            } else if (ack.avoiding_object) {
                esp_osc_send(&client, &target, "/drone/output/avoiding-object", "");
            } else if (ack.start_failed) {
                esp_osc_send(&client, &target, "/drone/output/start-failed", "");
            } else if (ack.position_lost) {
                esp_osc_send(&client, &target, "/drone/output/position_lost", "");
            } else {
                esp_osc_send(&client, &target, "/drone/output/internal-target-pos-in-m", "fff", ack.x, ack.y, ack.z);
            }
        }
        vTaskDelay(5);
        if (get_osc_send(&osc_send_message) == pdPASS) {
            if (strcmp(osc_send_message.format, "s") == 0) {
                esp_osc_send(&client, &target, osc_send_message.topic, osc_send_message.format, osc_send_message.string);
            } else if (strcmp(osc_send_message.format, "f") == 0) {
                esp_osc_send(&client, &target, osc_send_message.topic, osc_send_message.format, osc_send_message.af);
            } else if (strcmp(osc_send_message.format, "ff") == 0) {
                esp_osc_send(&client, &target, osc_send_message.topic, osc_send_message.format, osc_send_message.af, osc_send_message.bf);
            } else if (strcmp(osc_send_message.format, "fff") == 0) {
                esp_osc_send(&client, &target, osc_send_message.topic, osc_send_message.format, osc_send_message.af, osc_send_message.bf, osc_send_message.cf);
            }
        }
        vTaskDelay(5);
    }
}

// Receive

static bool osc_callback(const char *topic, const char *format, esp_osc_value_t *values){
    if (strncmp(topic, "/drone/output", strlen("/drone/output")) == 0) {
        // Skip if starts with "/drone/output"
        return true;
    }
    // if (strncmp(topic, "/adm/obj/16/gain", strlen("/adm/obj/16/gain")) == 0) {
    //     return true;
    // }
    // if (strncmp(topic, "/adm/obj/16/mute", strlen("/adm/obj/16/mute")) == 0) {
    //     return true;
    // }
    ESP_LOGI(TAG, "Got message: %s (%s)", topic, format);
    Osc_send_t osc_send_message = {0};
    Osc_send_t osc_instant_response = {0};
    strcpy(osc_instant_response.topic, "/drone/output/osc_received");
    strcpy(osc_instant_response.format, "s");
    strcpy(osc_instant_response.string, topic);
    set_osc_send(&osc_instant_response);
    // esp_osc_send(&client, &target, "/drone/output/osc_received" , "s", topic);
    // ESP_LOGI(TAG, "Constants are: x_max: %.3f %.3f, y_max: %.3f %.3f", global_constants.x_max_positive, global_constants.x_max_negative, global_constants.y_max_positive, global_constants.y_max_negative);
    AdmPolar_t polar = {0};
    AdmCartesian_t cart = {0};

    if (strcmp(topic, "/drone/control/current-position") == 0) {
        Parameters_t params = {0};
        get_parameters(&params);
        ESP_LOGI(TAG, "Current params: xyz: %.2f %.2f %.2f, battery: %.2f", params.x, params.y, params.y, params.batteryP);
        get_coords_from_parameters(&polar, &cart);
        // Send osc
        strcpy(osc_send_message.topic, "/drone/output/current-position");
        strcpy(osc_send_message.format, "fff");
        osc_send_message.af = cart.x;
        osc_send_message.bf = cart.y;
        osc_send_message.cf = cart.z;
        set_osc_send(&osc_send_message);
    }
    if (strcmp(topic, "/drone/control/battery-status") == 0) {
        Parameters_t params = {0};
        get_parameters(&params);
        ESP_LOGI(TAG, "Current params: xyz: %.2f %.2f %.2f, battery: %.2f", params.x, params.y, params.y, params.batteryP);
        // Send osc
        strcpy(osc_send_message.topic, "/drone/output/battery-status");
        strcpy(osc_send_message.format, "f");
        osc_send_message.af = params.batteryP;
        set_osc_send(&osc_send_message);
    }
    
    // If battery low, stop here, dont trigger movements
    if (battery_empty) {
        return true;
    }

    if (strcmp(topic, "/drone/control/land") == 0) {
        Instructions_t land_now = {0};
        land_now.landing = 1;
        set_instruction(&land_now);
    }
    if (strcmp(topic, "/drone/control/disable_avoidance") == 0) {
        Instructions_t dis_av = {0};
        dis_av.disable_avoidance = 1;
        set_instruction(&dis_av);
    }

    // ADM-OSC implementation
    if (strcmp(topic, "/adm/obj/16/azim") == 0) {
        if (strcmp(format, "f") == 0) {
            azim(values[0].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            // Send osc
            strcpy(osc_send_message.topic, topic);
            strcpy(osc_send_message.format, "f");
            osc_send_message.af = polar.azimuth;
            set_osc_send(&osc_send_message);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/elev") == 0) {
        if (strcmp(format, "f") == 0) {
            elev(values[0].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            // Send osc
            strcpy(osc_send_message.topic, topic);
            strcpy(osc_send_message.format, "f");
            osc_send_message.af = polar.elevation;
            set_osc_send(&osc_send_message);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/dist") == 0) {
        if (strcmp(format, "f") == 0) {
            dist(values[0].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            // Send osc
            strcpy(osc_send_message.topic, topic);
            strcpy(osc_send_message.format, "f");
            osc_send_message.af = polar.distance;
            set_osc_send(&osc_send_message);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/aed") == 0) {
        if (strcmp(format, "fff") == 0) {
            //ESP_LOGI(TAG, "got message: %s (%s): %f %f %f", topic, format, values[0].f, values[1].f, values[2].f);
            aed(values[0].f, values[1].f, values[2].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            // Send osc
            strcpy(osc_send_message.topic, topic);
            strcpy(osc_send_message.format, "fff");
            osc_send_message.af = polar.azimuth;
            osc_send_message.bf = polar.elevation;
            osc_send_message.cf = polar.distance;
            set_osc_send(&osc_send_message);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/x") == 0) {
        if (strcmp(format, "f") == 0) {
            x(values[0].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            // Send osc
            strcpy(osc_send_message.topic, topic);
            strcpy(osc_send_message.format, "f");
            osc_send_message.af = cart.x;
            set_osc_send(&osc_send_message);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/y") == 0) {
        if (strcmp(format, "f") == 0) {
            y(values[0].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            // Send osc
            strcpy(osc_send_message.topic, topic);
            strcpy(osc_send_message.format, "f");
            osc_send_message.af = cart.y;
            set_osc_send(&osc_send_message);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/z") == 0) {
        if (strcmp(format, "f") == 0) {
            z(values[0].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            // Send osc
            strcpy(osc_send_message.topic, topic);
            strcpy(osc_send_message.format, "f");
            osc_send_message.af = cart.z;
            set_osc_send(&osc_send_message);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/xy") == 0) {
        if (strcmp(format, "ff") == 0) {
            xy(values[0].f, values[1].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            // Send osc
            strcpy(osc_send_message.topic, topic);
            strcpy(osc_send_message.format, "ff");
            osc_send_message.af = cart.x;
            osc_send_message.bf = cart.y;
            set_osc_send(&osc_send_message);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/xyz") == 0) {
        if (strcmp(format, "fff") == 0) {
            //ESP_LOGI(TAG, "got message: %s (%s): %f %f %f", topic, format, values[0].f, values[1].f, values[2].f);
            xyz(values[0].f, values[1].f, values[2].f);
        } else {
            get_coords_from_parameters(&polar, &cart);
            // Send osc
            strcpy(osc_send_message.topic, topic);
            strcpy(osc_send_message.format, "fff");
            osc_send_message.af = cart.x;
            osc_send_message.bf = cart.y;
            osc_send_message.cf = cart.z;
            set_osc_send(&osc_send_message);
        }
    }
    else if (strcmp(topic, "/adm/obj/16/dmax") == 0) {
        if (strcmp(format, "f") == 0) {
            //ESP_LOGI(TAG, "got message: %s (%s): %f", topic, format, values[0].f);
            dmax(values[0].f);
        } else {
            // Send osc
            strcpy(osc_send_message.topic, topic);
            strcpy(osc_send_message.format, "f");
            osc_send_message.af = get_dmax();
            set_osc_send(&osc_send_message);
        }
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

