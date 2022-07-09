//#pragma once
#ifndef MODEM_SMS
#define MODEM_SMS
#include <Arduino.h>

void SendSMS(const char *message, byte item = 0xFF);
void modemInit();



#endif