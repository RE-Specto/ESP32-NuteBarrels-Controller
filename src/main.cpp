/*
Copyright 2019-2022 Evgeni Vi
more info and license - soon
*/
#include "main.h"
#include "Expanders.h"
#include "SystemState.h"
#include "Barrels.h"
#include "FlowSensor.h"
#include "PresureSensor.h"
#include "WebServer.h"
#include "Filesystem.h"
#include "Modem.h"
#include "NTPClient.h"
#include "FMSD.h"
#include "globals.h"

/* initial sequnence */
void setup()
{
    // initialize UART0 - serial output
    Serial.begin(115200);

    // initialize UART2 - serial MUX
    Serial2.begin(9600, SERIAL_8N1);

    // initialize expander ports - I2C wire
    Expanders.Init();

    // initialize SD Card SPI bus + SPIFFS Storage
    Filesys.begin();

    // load structs from SD card
    Filesys.LoadStructs();

    // set pins for pressure sonsors
    Pressure.begin();

    // attach interrupts for flow sensors
    Flow.begin();

    // attach interrupts for start-stop buttons 
    State.begin();

    // initialize uart2 SIM800L modem at MUX port 7?
    modemInit();

    // Create tasks
    xTaskCreatePinnedToCore(fmsTask, "loop1", 10000, (void *)1, 1, &loop1, ARDUINO_RUNNING_CORE);

    // setup NTP
    // test ntp:
    // LOG.println(TimeClient.getNtpTime());
  
    //start Web Server
    WebServer.begin();
    ArduinoOTA.setHostname("Barrels");
    ArduinoOTA.begin();
    //mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    MDNS.addService("http", "tcp", 80); // add mDNS http port

    // test all ultrasonic sensors
    // Barrels.TestSensors();
    // or only the first one
    // Barrels.SonicMeasure(0, 1);

    Expanders.setRGBLED(LED_OFF);
    //SendSMS("System Started");
}

void loop()
{
    ArduinoOTA.handle();
    State.isChanged();
    vTaskDelay(1);
    //vTaskDelay(portMAX_DELAY); // wait as much as posible ...
}