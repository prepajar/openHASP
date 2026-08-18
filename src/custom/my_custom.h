#ifndef HASP_CUSTOM_H
#define HASP_CUSTOM_H
#if defined(HASP_USE_CUSTOM)
#include "hasplib.h"
 
void custom_setup();
void custom_loop();
void custom_every_second();
void custom_every_5seconds();
bool custom_pin_in_use(uint8_t pin);
void custom_get_sensors(JsonDocument& doc);
void custom_topic_payload(const char* topic, const char* payload, uint8_t source);
void custom_state_subtopic(const char* subtopic, const char* payload);
 
#endif // HASP_USE_CUSTOM
#endif // HASP_CUSTOM_H
 
