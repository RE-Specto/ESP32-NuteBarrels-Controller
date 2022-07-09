//#pragma once
#ifndef WEB_SERVER
#define WEB_SERVER
#include <Arduino.h>
#include <WiFi.h>
//needed for library
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h> //https://github.com/tzapu/WiFiManager

//#include <Hash.h>
#include <AsyncTCP.h>
#include <ArduinoOTA.h>
#include "ESPmDNS.h"
//https://github.com/jandrassy/TelnetStream

class ServerClass
{
private:
public:
    //AsyncWebServer server;
    void begin();
    WiFiUDP UDP;
};

extern ServerClass WebServer;
#endif