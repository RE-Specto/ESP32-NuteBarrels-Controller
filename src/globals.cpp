#include "globals.h"

TaskHandle_t loop1; // fmsTask handle
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // for interrupts and xTasks
