//#pragma once
#ifndef FMSD
#define FMSD
#include <Arduino.h>

//#define DEBUG_SD
#define DEBUG_SONIC
//#define DEBUG_FLOW
//#define DEBUG_MUX
//#define DEBUG_NET
//#define DEBUG_MORE
#define USE_FLOW_INSTEAD // in case of sonic inaccuracy

// sensor number
#define FRESHWATER 1
#define NUTRIENTS 2

#define LOG Serial.printf("t[%06lli]l[%04i]f[%s] ", esp_timer_get_time() / 1000000, __LINE__, __FUNCTION__);Serial   //SerialAndTelnet or file

extern TaskHandle_t loop1; // fmsTask handle
extern portMUX_TYPE mux; // = portMUX_INITIALIZER_UNLOCKED; // for interrupts and xTasks


#endif