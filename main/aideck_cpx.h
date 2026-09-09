#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_transport.h"

void aideck_cpx_init();

extern esp_routable_packet_t txp_to_app;
void sendGotoFixedPositionToStm(float x, float y, float z, float duration);