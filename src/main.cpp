#include <Arduino.h>
#include <MCP23017.h>
//#include <ESPAsyncWebServer.h>
//#include <SD.h>
//#include "SPIFFS.h"
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

// declare I/O Expander ports and constants


// Task 0 - Error reporting task


// SD Card and SPIFFS


// load JSON Object from Storage to Struct


// flow Sensors Pin Declarations
// and Interrupt routines


// Pressure Sensors Pin Declarations
// and Analog reads
// task pressure sensors - stops pumps on overpressure


// Ultrasonic sensors
// declare UART2 


// WebServer

//https://github.com/me-no-dev/ESPAsyncWebServer

//RTC module


// Task 1 - refilling and mixing task


// Task 2 - storing task


// Task 3 - draining task



/* initial sequnence */
void setup() {

  //initiate expander ports - I2C wire



  // initiate SD Card - SPI bus + SPIFFS



  // initiate UART0 - serial output
  Serial.begin(115200);

  // initialize UART2 - serial MUX
  Serial2.begin(9600, SERIAL_8N1);

  // initialize uart2 SIM800L modem at MUX port 7


  // setup Real Time Clock


  // Connect to Wi-Fi

  //start Web Server + WebSocket

  // attach interrupts for flow sensors


  // read last status "object" from sdcard

  // Create tasks

}
  
void loop() {
  // disable loop watchdog - working with tasks only?
}