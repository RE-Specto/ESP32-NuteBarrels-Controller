/*
to implement:
declassify
implement webserver
start/stop interrupts
    start sets canMix global boolean
        task or loop reads the bool - changes RGB immediately, but start mixing task only when no tasks running
    stop sets shouldStop global boolean
        task or loop reads the bool - changes RGB immediately, triggers stop and save
rgb led status (read from system status via error-reporting task?)
apply test code to check every part of the system:
    relays manual - on off
    flow sensor data - counter + flow/second
    pressure sensor data - live
    ultrasonic data - live
    rgb led
    start-stop
    sim800L - send sms
work on system hardware 


rewrite:
expander - ok
Task 0 - Error reporting - rewrite
storage - ok
barrels - rewrite
flow sensor - half-ok
pressure sensor - leave as class? make one flow-pressure?
ultrasonic - move to barrels?
implement webserver
deprecate RTC? replace with ntp + timeAlarms?
MixingTask - declassify
storing task - implement
draining task -declassify
add to setup everything
*/
#include <Arduino.h>
#include <Wire.h>
#include <MCP23017.h>
#include <SD.h>
#include "SPIFFS.h"
//#include <RtcDS3231.h>
#include <WiFi.h>

//needed for library
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>         //https://github.com/tzapu/WiFiManager

//#include <Hash.h>
#include <AsyncTCP.h>
#include <ArduinoOTA.h>
//https://github.com/jandrassy/TelnetStream
#include "TimeAlarms.h" 

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif


//hex codes for "error_state" bit-field
#define NOWATER_ERROR	    0x01    //1-1-water line no pressure
#define PS1TOOHIGH_ERROR    0x08    //4-8-pump1 pressure too high - storing solenoids malfunction
#define PS1TOOLOW_ERROR     0x10    //5-16-pump1 pressure too low - pump failure or no more liquid
#define PS2TOOHIGH_ERROR    0x20    //6-8-pump1 pressure too high - storing solenoids malfunction
#define PS2TOOLOW_ERROR     0x40    //7-16-pump1 pressure too low - pump failure or no more liquid
#define FS1_NOFLOW_ERROR    0x80    //8-128-fs1 no flow
#define FS2_NOFLOW_ERROR    0x100   //9-256-fs2 no flow
#define NOMAINS_ERROR       0x400   //B-1024-no mains power
#define BARRELS_ERROR       0x800   //C-2048-barrels error

//hex codes for "state" bit field
#define FILLING_STATE   0X10    // F = filling task on
#define MIXING_STATE    0x08    // M = mixing task on
#define STORING_STATE   0x04    // S = storing task on
#define DRAINIG_STATE   0x02    // E = draining task on
#define STOPPED_STATE   0x01    // X = stopped status on

// RGB_LED bitmap 00000BGR
#define LED_OFF     0x00 //000
#define LED_RED     0x01 //001
#define LED_GREEN   0x02 //010
#define LED_YELLOW  0X03 //011
#define LED_BLUE    0x04 //100
#define LED_MAGENTA 0X05 //101
#define LED_CYAN    0X06 //110
#define LED_WHITE   0X07 //111

#define OUT_PORT Serial //SerialAndTelnet or file
#define SYNC_INTERVAL 600 // NTP sync - in seconds
#define NTP_PACKET_SIZE 48
// NTP time is in the first 48 bytes of message
#define NTP_HOSTNAME "pool.ntp.org"
#define NTP_PORT 123
#define NTP_TIMEOUT 5000
#define LOCAL_UDP_PORT 8888  // local port to listen for UDP packets

// load Object from Storage to Struct
#define NUM_OF_BARRELS 6

 // in liters how much is gonna get stuck in the barrel 
 // because you can never drain to dry
 // !! reimplement next version to be adjustable
#define BARREL_LOW_LIMIT 50

//Flow sensor Pin declarations
#define FLOW_1_PIN 25
#define FLOW_2_PIN 26
//#define FLOW_3_PIN 27

//Pressure sensor Pin declarations
#define PRESSUR_1_PIN 36
#define PRESSUR_2_PIN 34
//#define PRESSUR_3_PIN 35


// Globals
File file;
WiFiUDP UDP;
AsyncWebServer server(80);
DNSServer dns;
bool isTimeSync = false;


















// declare I/O Expander ports and constants


// MCP23017 with pin settings
MCP23017 expander1(0); // Base Address + 0: 0x20
MCP23017 expander2(1); // Base Address + 1: 0x21

void setMUX(uint8_t address);
void setRGBLED(uint8_t address);

void expanderInit(){

    // initialize TwoWire communication
    Wire.begin();

    for (size_t i = 0; i < 16; i++)
    {
    // set GPA0-7 GPB0-7 to be an output on both expanders
    expander1.getPin(i).setPinMode(OUTPUT);
    expander2.getPin(i).setPinMode(OUTPUT);
    // initialize the pins to be HIGH
    expander1.getPin(i).setValue(1);
    expander2.getPin(i).setValue(1);
    }

    // setup the MCP23017 expanders
    expander1.setup();
    expander2.setup();

    // reset analog mux and RGB LED
    setMUX(0);
    setRGBLED(LED_WHITE);

}

//set analogMUX address
void setMUX(uint8_t address){
    for (uint8_t i = 0; i < 4; i++){
        //checks bit 0-3 of mux Address "address"
        //if bit set - setValue receives a positive value
        //otherwise sets setValue with 0
        //offset of 0 - expander 0x20 pins a0 a1 a2 a3
        expander1.getPin(i+0).setValue( address & (1 << i) ); 
    }
    expander1.write();
}

void setRGBLED(uint8_t address){
    for (uint8_t i = 0; i < 3; i++){
        //checks bit 0-2 of color ( R G B ) in "address"
        //offset of 4 - expander 0x20 pins a4 a5 a6
        expander1.getPin(i+4).setValue( address & (1 << i) ); 
    }
    expander1.write();
}

void FillingRelay(uint8_t address, bool state){
    //offset of 8 - expander 0x20 pins b0-b7
    expander1.getPin( address+8 ).setValue( state ); 
    expander1.write();
}

void StoringRelay(uint8_t address, bool state){
    //offset of 0 - expander 0x21 pins a0-a7
    expander2.getPin( address ).setValue( state ); 
    expander2.write();
}

void DrainingRelay(uint8_t address, bool state){
    //offset of 8 - expander 0x21 pins b0-b7
    expander2.getPin( address+8 ).setValue( state ); 
    expander2.write();
}

bool LockMUX(){
    // code goes here
    return true; // untill coded otherwise
}

void UnlockMUX(){
    // code goes here
}



















// Task 0 - Error reporting task

void SendSMS(String message);

class state_class{
public:
    state_class();

    bool return_state(uint8_t mask);
    bool mixing_state();
    bool storing_state();
    bool draining_state();
    bool stopped_state();

    uint16_t get_error();
    void set_error(uint16_t error);
    void unset_error(uint16_t error);
    bool any_new_errors();
    uint16_t last_error();
    void error_reported();

private:
    //bit field
    //000FMSEX
    // F = filling task on
    // M = mixing task on
    // S = storing task on
    // E = emptying task on
    // X = stopped status on
    uint8_t _state;

    // bit field
    //000CBA987654321
    //0-0-no errors
    //1-1-water line no pressure
    //2-2-
    //3-4-
    //4-8-
    //5-16-
    //6-32-
    //7-64-
    //8-128-
    //9-256-
    //A-512-
    //B-1024-
    //C-2048-
    //D-4096-
    //E-8192-
    //F-16384-
    uint16_t _error_state;
    uint16_t _lastError_state;

};


// create state_class instance
state_class mystate;

void errorReportTask(){
// while there is no new errors
while (!mystate.any_new_errors()){
    delay(1000);     //wait - endless loop untill error status changes
}

// goes here if there is an error to report
if (mystate.get_error() > 0 )
    // lock MUX
    OUT_PORT.println("reporting task is trying to lock MUX");
    while(!LockMUX()){
        delay(100); // waiting untill mux is unlocked by another task
    }

    // need to reimplement to send error description instead of error number
    SendSMS( "barrels error " + String(mystate.last_error()) );

    // unlock mux
    UnlockMUX();
    // set global ErrorState = LastErrorState

}

void set_error(uint16_t error){
    mystate.set_error(error);
}

// should I need to lock MUX before?
void modemInit(){
    setMUX(7); // modem is at port 7
    delay(10); // wait until expander + mux did their job
    // Serial2.begin(9600, SERIAL_8N1); // already done in main
    Serial2.println("AT"); //Once the handshake test is successful, it will back to OK
    Serial2.println("AT+CMGF=1"); // Configuring TEXT mode
    SendSMS("System Started");
}

// set MUX to SIM_MUX_ADDRESS
// send sms with error code description
void SendSMS(String message){
    setMUX(7); // modem is at port 7
    delay(10); // wait until expander + mux did their job
    Serial2.println("AT+CMGS=\"+972524373724\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
    Serial2.print( message ); //text content
    Serial2.write(26); // send ctrl+z end of message
    //Serial2.print((char)26); // if the above won't work - thy this one instead
}
// module needs 3.4V to 4.4V (Ideal 4.1V) @ 2amp

/*
  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  mySerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  mySerial.println("AT+CREG?"); //Check whether it has registered in the network
  ATI – Get the module name and revision
AT+COPS? – Check that you’re connected to the network, in this case BSNL
AT+COPS=? – Return the list of operators present in the network.
AT+CBC – will return the lipo battery state. The second number is the % full (in this case its 93%) and the third number is the actual voltage in mV (in this case, 3.877 V)
  */


state_class::state_class(){
    _state=0;
    _error_state=0;
}

bool state_class::return_state(uint8_t mask){
    //return (_state & ( 1 << position )) >> position
    //return (_state >> position) & 1 //right-shifting to position bit, and then extracting the first bit
    return _state & mask;
}

// returns error state
uint16_t state_class::get_error(){
    return _error_state;
}

void state_class::set_error(uint16_t error){
    _error_state |=  error;
}

void state_class::unset_error(uint16_t error){
    _error_state &= ~error;
}

// returns true if error_state changed
bool state_class::any_new_errors(){
    return _error_state == _lastError_state;
}

//returns the difference between current and last error states
// !! need to reimplement to return either the new error position
// so I dont have to use uint16_t for return
// or either if two errors should be reported at once - a while loop that solves them one by one
uint16_t state_class::last_error(){
    return _error_state ^ _lastError_state;
}

void state_class::error_reported(){
    _lastError_state=_error_state;
}























// SD Card and SPIFFS

// initiate SD Card and SPIFFS
// need to check if not getting out of scope
void initStorage(){
    while (!SD.begin()) {
        OUT_PORT.println(F("Error: Failed to initialize SD card"));
        delay(1000);
    }
    OUT_PORT.println(F("SD card OK"));
    if(!SPIFFS.begin(true)){
        OUT_PORT.println(F("Warning: Failed to initialize SPIFFS"));
        //return;
    }
    else {
        OUT_PORT.println(F("SPIFFS OK"));
    }
}


/*-------- Filesystem code ----------*/
// loading Structs from files
// filename, struct pointer, struct lenght "sizeof(myStruct)"
bool Load(const char* fname, byte* stru_p, uint16_t len){
    uint16_t count=0;
    OUT_PORT.printf("Loading %s\t", fname);
    file = SD.open(fname, "r"); 
        if (!file)
        OUT_PORT.print(F("unable to open file\r\n\r\n"));
        else
        for (; count<len; count++) 
            if ( file.available())
            *( stru_p + count ) = file.read();
    #ifdef DEBUG
    OUT_PORT.printf("%s\r\n%u out of %u Bytes read. filesize %satch.\r\n", count==len?"successfully":"failed", count, len, len==file.size()?"M":"Mism");
    #else
    OUT_PORT.println();
    #endif
    file.close();
    return count == len;
}

bool LoadSettings(){
    return false;   //return Load(CONFFILE, (byte*)&mySettings, sizeof(mySettings));
}

bool Save(const char* fname, byte* stru_p, uint16_t len){
    uint16_t count=0;
    OUT_PORT.printf("Saving %s\t", fname);
    file = SD.open(fname, "w"); // creates the file of not exist
    //file.setTimeCallback(timeCallback);
    if (!file)
        OUT_PORT.print(F("unable to open file\r\n\r\n"));
    else
        count = file.write(stru_p, len);
    #ifdef DEBUG
    OUT_PORT.printf("%s\r\n%u out of %u Bytes writen. filesize %satch.\r\n", count==len?"successfully":"failed", count, len, len==file.size()?"M":"Mism");
    #else
    OUT_PORT.println();
    #endif
    file.close();
    return count == len;
}

bool SaveSettings(){
    return false;   //return Save(CONFFILE, (byte*)&mySettings, sizeof(mySettings));
}








































// my data struct should go here

class barrels{
public:
    class barrel{
        friend class barrels;
    public:
        barrel();
        int get_error_state(); 
        void set_error_state(uint8_t error_mask);
        void unset_error_state(uint8_t error_mask);
        void reset_error_state(uint8_t error_mask);
        int measure_ultrasonic();
        void add_flow_volume();
        void substract_flow_volume();
        int get_flow_volume(); // in liters - data from flow 
        int get_sonic_volume(); // in liters - data from ultrasonic 
        int get_combined_volume(); // in liters - data from ultrasonic and flow combined
        int get_flow_percentage(); // 0 to 100 from flow 
        int get_sonic_percentage(); // 0 to 100 from ultrasonic 
        int get_combined_percentage(); // 0 to 100 from ultrasonic and flow combined

    protected:
        //bit field
        //76543210
        //0 - flush solenoid error
        //1 - store solenoid error
        //2 - drain solenoid error
        //3 - Ultrasonic sensor error
        //4 - 
        //5 - 
        //6 - disabled manually
        //7 - other error
        uint8_t _error_state;

        uint8_t _TotalVolume; // should be constant
        uint8_t _Coefficient;
        uint8_t _EmptyHeight;
        uint8_t _FullHeight;

        // important for mux selection
        uint8_t _SonicNumber;

        //data from flow sensor
        uint8_t _current_volume;

        // !! implement another counder for fresh water?

    };

private:
    //bit field
    //76543210
    //0 - 
    //1 - 
    //2 - 
    //3 - 
    //4 - 
    //5 - 
    //6 - 
    //7 - other error
    uint8_t _error_state;
    barrel _barrel[NUM_OF_BARRELS];

public:
    barrels();
    barrel &getBarrel(uint8_t barrelNumber);
}; // class barrels



// create class instances

// load JSON Object from storage

// copy JSON Object to data struct

// "Object Save" Task - serialize data to JSON object and save to SD

//barrel constructor
barrels::barrel::barrel(){
    OUT_PORT.println("barrel constructor");
}

//returns error state bit field
int barrels::barrel::get_error_state(){
    return _error_state;
}

//apply error state according to mask
void barrels::barrel::set_error_state(uint8_t error_mask){
    // "or" operator to the rescue
    _error_state |= error_mask;
}

//disapply error state according to mask
void barrels::barrel::unset_error_state(uint8_t error_mask){
    // "and not" operators to the rescue
    _error_state &= ~error_mask;

}

//reset error state according to mask
void barrels::barrel::reset_error_state(uint8_t error_mask){
    _error_state = 0;
}

//return raw value
int barrels::barrel::measure_ultrasonic(){
    // lock mux
    // set mux address to _SonicNumber
    // read sensor 5? times, get avearge
    // unlock mux
    return 0;  /// !!!! sensor ultra should be moved to this file!! + expander solenoids commands too
}

// increment _current_volume
void barrels::barrel::add_flow_volume(){

}

// decrement _current_volume
void barrels::barrel::substract_flow_volume(){

}

// in liters - data from flow sensor
int barrels::barrel::get_flow_volume(){
    // uses _current_volume
    return 0;
}

/*
barrels calculations:
(barrelsEmptyHeight - ultrasonicValue) * barrelsCoefficent = value in liters
(barrelsEmptyHeight - barrelsFullHeight) / ultrasonicValue = barrels percent 
*/



 // in liters - data from ultrasonic 
int barrels::barrel::get_sonic_volume(){
    return ( _EmptyHeight - measure_ultrasonic() ) * _Coefficient ;
}

// in liters - data from ultrasonic and flow combined
int barrels::barrel::get_combined_volume(){
    return ( get_flow_volume() + get_sonic_volume() ) / 2; // fifty-fifty - need to change?
}

 // 0 to 100 from flow 
int barrels::barrel::get_flow_percentage(){
    return 0;
}

 // 0 to 100 from ultrasonic 
int barrels::barrel::get_sonic_percentage(){
    return 0;
}

 // 0 to 100 from ultrasonic and flow combined
int barrels::barrel::get_combined_percentage(){
    return ( get_flow_percentage() + get_sonic_percentage() ) / 2; // fifty-fifty - need to change?
}


//barrels constructor
//assign every barrel its number
barrels::barrels(){
    OUT_PORT.println("barrels constructor");
    for (uint8_t x=0;x<NUM_OF_BARRELS;x++){
            // !!!! for test only - need to implement loading from JSON object
        _barrel[x]._error_state=0;
        _barrel[x]._current_volume=0;
        _barrel[x]._TotalVolume=0;
        _barrel[x]._Coefficient=0;
        _barrel[x]._EmptyHeight=0;
        _barrel[x]._FullHeight=0;        
        _barrel[x]._SonicNumber=x;
    }
}

barrels::barrel &barrels::getBarrel(uint8_t barrelNumber){
  if (barrelNumber < 0 || barrelNumber >= NUM_OF_BARRELS)
    barrelNumber = 0; // playing safe with dummys :-)

  return barrels::_barrel[barrelNumber];
}

extern barrels mybarrels;

void test1(){
    mybarrels.getBarrel(1).measure_ultrasonic();
}
































// flow Sensors Pin Declarations
// and Interrupt routines


void IRAM_ATTR FlowSensor1Interrupt();
void IRAM_ATTR FlowSensor2Interrupt();
void enableFlowInterrupts();

struct flow_sensor{
    uint8_t sensorPin;
    uint8_t conversion_multiplier;
    uint64_t counter;
} myFlowSensor[2];


//Flow sensor Interrupt counters
volatile int FlowSensor1Count = 0;
volatile int FlowSensor2Count = 0;

//for interrupts and xTasks
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//flow sensor 1 interrupt routine
void IRAM_ATTR FlowSensor1Interrupt() {
  portENTER_CRITICAL_ISR(&mux);
  FlowSensor1Count++;
  portEXIT_CRITICAL_ISR(&mux);
}
//flow sensor 2 interrupt routine
void IRAM_ATTR FlowSensor2Interrupt() {
  portENTER_CRITICAL_ISR(&mux);
  FlowSensor2Count++;
  portEXIT_CRITICAL_ISR(&mux);
}

// attach interrupts function
void enableFlowInterrupts(){
  pinMode(FLOW_1_PIN, INPUT_PULLUP);
  pinMode(FLOW_2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_1_PIN), FlowSensor1Interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_2_PIN), FlowSensor2Interrupt, RISING);
}

// assign FlowSensorCounts to object

// read FlowSensorCounts to object
// decrease FlowSensorCounts

// calculate flowCounts to liters
// use flowSensCalib as a multiplier

// calculate liters per hour


/*
Pulses per Liter: 450
Model: YF-S201

From the spec sheet:
16 Hz = 2 L/min = 120 L/hr 
32.5 Hz = 4 L/min = 240 L/hr
49.3 Hz = 6 L/min = 360 L/hr
65.5 Hz = 8 L/min = 480 L/hr
82 Hz = 10 L/min = 600 L/hr
etc.  Max 30 L/min
*/

/*
class flow_sensor{
public:
    flow_sensor(uint8_t sensorPin);
    // read only thru this function.
    uint8_t get_multiplier();

    // only from JSON data object
    void set_multiplier(int value);

private:
    uint8_t _sensorPin;
    uint8_t _conversion_multiplier;
};
*/


























// Pressure Sensors Pin Declarations
// and Analog reads
// task pressure sensors - stops pumps on overpressure


class pressure_sensor{
public:
    pressure_sensor(uint8_t sensorPin, uint16_t TooLowErr, uint16_t TooHighErr);
    uint16_t measure(); // returns pressure in psi?

private:
    uint8_t _sensorPin;
    uint8_t _multiplier;
    uint8_t _offset;
    uint8_t _max_pressure;
    uint8_t _min_pressure;
    uint16_t _TooLowErr;
    uint16_t _TooHighErr;
};


// pressure_sensor (sensor pin, too low error bit, too high error bit)
pressure_sensor::pressure_sensor(uint8_t sensorPin, uint16_t TooLowErr, uint16_t TooHighErr){
    _sensorPin=sensorPin;
    _TooLowErr = TooLowErr;
    _TooHighErr = TooHighErr;
    // next values shoud be set from JSON object data
    _multiplier=2;
    _offset=2;
    _max_pressure=30;
    _min_pressure=80;

    OUT_PORT.println("pressure_sensor constructor");
    OUT_PORT.println(sensorPin);

    // initialize analog pin for the sensor
    pinMode(sensorPin,INPUT);
    
}

// read sensor
uint16_t pressure_sensor::measure(){
    // convert analog value to psi
    uint16_t pressure = analogRead(_sensorPin) * _multiplier - _offset; //test - needs to be replaced with acrual formula
    if (pressure < _min_pressure) {
        // set underpressure error
        set_error(_TooLowErr);
    }
    if (pressure > _max_pressure) {
        // set overpressure error
        set_error(_TooHighErr);
    }

    return pressure;
}


// initialize analog pins for sensors
void initPressureSensors(){
    // !! need to reimplement to use error mask position instead the whole uint16_t mask
    pressure_sensor Ps1(PRESSUR_1_PIN,PS1TOOHIGH_ERROR,PS1TOOLOW_ERROR);
    pressure_sensor Ps2(PRESSUR_2_PIN,PS2TOOHIGH_ERROR,PS2TOOLOW_ERROR);
    // testing
    // OUT_PORT.println(Ps1.measure());
    // if error do something about it - rather not here but in the relevant task
}


// read sensor

// convert analog value to psi
// use pressSensMult and pressSensOffset






























// Ultrasonic sensors
// declare UART2 
// initiate UART2 - ultrasonic sensors MUX
void initUltrasonic(){
    
}

//while (Serial2.available())


// declare each barrel's ultrasonic sensor address in analog mux

// if mux not locked - read each sensor into object - will take some time???
// lock MUX
// loop thru all barrels using setMUX function (from expander)
// unlock MUX































// WebServer

//https://github.com/me-no-dev/ESPAsyncWebServer

/*

#define wifiTimeout 10 // in seconds

//global string for sending json to web client - this one will be alive thru all sessions
//String json = "[{\"time\":\"" + returnDateTime(now) + "\"";
String json = "[{\"time\":\"12:00\"";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

//temporary websocket code - will send json string every time a beowser sends something
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_DATA){ 
    for(int i=0; i < len; i++) {
      Serial.print((char) data[i]); // data received from browser - prints out to serial
    }
    json += ",\"flow\":\""+String(random(500))+"\""; // test sending to web
    json += "}]";
    client->text(json); //sends json global variable to the client
    json = "[{\"time\":\""+String(random(24))+":"+String(random(59))+"\""; //clean and re-use the global json string
    //json = String();
  }
  testPrintRTC(); // test - remove!!!
}



void setupServer(){

  // add websocket /ws on webserver
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Requested: " + request->url() );
    if(!SD.exists("/index2.html")){
      SD.end();
      Serial.println("trying to restart SD");
      if(!SD.begin(22)){
        Serial.println("unable to read form SD card");
        request->send(200, "text/html", "<html><body><center><h1>check SD Card please</h1></center></body></html>");
        }
      }
    request->send(SD, "/index2.html", String(), false);
  });

  // Start server
  server.begin();
} // void setupServer() end

*/


















/*-------- NTP code ----------*/
// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address, byte (&packetBuffer)[NTP_PACKET_SIZE]){
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  UDP.beginPacket(address, NTP_PORT);
  UDP.write(packetBuffer, NTP_PACKET_SIZE);
  UDP.endPacket();
}

time_t getNtpTime(){
  isTimeSync = true; // prevent retrigger untill done
  byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
  IPAddress ntpServerIP; // NTP server's ip address
  while (UDP.parsePacket() > 0) ; // discard any previously received packets
  WiFi.hostByName(NTP_HOSTNAME, ntpServerIP);  // get a random server from the pool
  #ifdef DEBUG
  OUT_PORT.print("NTP time request via ");
  OUT_PORT.print(ntpServerIP);
  #endif
  sendNTPpacket(ntpServerIP, packetBuffer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < NTP_TIMEOUT) {
    uint8_t size = UDP.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      OUT_PORT.println(" Received NTP Response");
      UDP.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      uint16_t DTS = 0;//mySettings.DTS?3600UL:0;
      uint16_t timeZone = 3; // temporary
      return secsSince1900 - 2208988800UL + DTS + timeZone * 3600UL; // added DTS
    }
  }
  OUT_PORT.println("No NTP Response :-(");
  isTimeSync = false; // if failed - set back to false
  return 0; // return 0 if unable to get the time
}



//RTC module
/*
RtcDS3231<TwoWire> Rtc(Wire);



void setupRTC (){
    Rtc.Begin();
    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    if (!Rtc.IsDateTimeValid()) {
        if (Rtc.LastError() != 0) // communication error
              Serial.println(Rtc.LastError());
        else // RTC lost confidence in the DateTime
            Rtc.SetDateTime(compiled);
    }
    if (!Rtc.GetIsRunning()) 
        Rtc.SetIsRunning(true);
    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled)
        Rtc.SetDateTime(compiled); // sets time to PC time if needed
    Rtc.Enable32kHzPin(false);
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 
}

//temporary test
void testPrintRTC() 
{
    //RtcDateTime now = Rtc.GetDateTime();
    //printDateTime(now);
    Serial.println(getDateTimeNow());
	RtcTemperature temp = Rtc.GetTemperature();
    Serial.print(temp.AsFloatDegC());
    Serial.println(" *C");
}


//void printDateTime(const RtcDateTime& dt)

// Returns Date-Time string
String getDateTimeNow(){
    RtcDateTime currentTime = Rtc.GetDateTime();    //get the time from the RTC
    char str[20];   //declare a string as an array of chars
    sprintf(str, "%d/%d/%d %d:%d:%d",     //%d allows to print an integer to the string
        currentTime.Year(),currentTime.Month(),currentTime.Day(),
        currentTime.Hour(),currentTime.Minute(),currentTime.Second());
    return (str);
}



alarm - needs an interrupt pin connected to RTC square wave / alarm pin (SQW) 
https://github.com/Makuna/Rtc/wiki/RtcDS3231-AlarmOne
https://techtutorialsx.com/2017/02/04/esp8266-ds3231-alarm-when-seconds-match/


*/





























// Task 1 - refilling and mixing task
void MixingTask();

class mixing_class {
    public:
        mixing_class();
        uint8_t mixer_timer; // in minutes
        void open_mixer_motor(bool on); // opens and closes
        bool is_filling_open();
        bool is_mixer_open();

    private:
        uint8_t PrefillAmmount; // initial 100 liters
        bool _filling_on; // filling state
        bool _mixer_on;

};


/*
class mixing_class {
    public:
        mixing_class();
        uint8_t mixer_timer; // in minutes
        void open_mixer_motor(bool on); // opens and closes
        bool is_filling_open();
        bool is_mixer_open();

    private:
        uint8_t PrefillAmmount; // initial 100 liters
        bool _filling_on; // filling state
        bool _mixer_on;

};
*/

void MixingTask(){
    //      RefillingTask
    // wait untill: no mixing task, no storing task
    // set filling status on
    // stopped status off?
    // check water level is 100Liters
    // if less than 100L
    // open mixer tap
    // fill until 100L
    // close mixer tap
    // set filling status off
    // init save task



    //      MixingTask
    // wait untill  no filling task  no storing task
    // set mixing status on
    // mixing status off? last timer = 0 ? default last timer
    // stopped status off?
    // init save task
    // open mixer tap
    // start mixer motor
    // loop - while barrel < 500 OR timer > 0
    // if barrel full (500)
        // close mixer tap
        // init save task
    // if timer ended (0)
        // stop mixer motor
        // init save task

    // transfer status on
    // mixing status off
    // init save task
    // init storing task
}


















// Task 2 - storing task
void StoringTask(){
    // wait untill  no filling task  no mixing task
    // set storing status on
    // stopped status off?
    // init save task
    // measure barrel mix
        // while barrel mix  not empty
            // set x to 1
            // measure barrel x
            // until Barrel x not full
                // assign flow counter  to barrel x
                // open barrel x tap
                // start pump1
                // wait until  barrel x full
                // init save task
            // goto next barrel set x = x+1
            // all barrels full?
                // stop pump
                // wait 1 minute
                // set x to 1
                // loop
        // if barrel mix empty
            // stop pump1
            // set filling status on 
            // set storing status off
            // init save task + exit task

}




















// Task 3 - draining task
class drainSystem
{
public:
    drainSystem(void);
    uint16_t get_drain_requirement();
    uint16_t get_drain_counter();
    void set_drain_requirement(uint16_t liters);
    void set_drain_counter(uint16_t liters);
    bool start_drain(uint16_t liters);
private:
    uint16_t _drain_requirement; //in liters - how much to drain
    uint16_t _drain_counter;    //in liters - how much is already drained

};



//constructor
drainSystem::drainSystem(void)
{
    OUT_PORT.println("drainSystem constructor");
    _drain_requirement = 0;
    _drain_counter = 0;
}

//in liters - how much to drain
uint16_t drainSystem::get_drain_requirement()
{
    return _drain_requirement;
}

//in liters - how much is already drained
uint16_t drainSystem::get_drain_counter()
{
    return _drain_counter;
}
//in liters - how much to drain
void drainSystem::set_drain_requirement(uint16_t liters)
{
    _drain_requirement = liters;
}

//in liters - how much is already drained
void drainSystem::set_drain_counter(uint16_t liters)
{
    _drain_counter = liters;
}

bool drainSystem::start_drain(uint16_t liters)
{
    // set requirement
    _drain_requirement = liters;

    //if not error
    return true;
}


void DrainingTask(){
    // create drain class instance
  drainSystem mydrain;
  extern barrels mybarrels;

    // set draining status on
//    mystatus.setState(DRAINIG_STATE);

    // run save task
//    mysave.save();

    // stopped status off?
//    mystatus.getState(STOPPED_STATE);

    uint16_t drainCounter = mydrain.get_drain_requirement();
    // while drainCounter > 0
    while (drainCounter>0){
        // set x to 5
        int x = 5;
        // measure barrel x
        // Barrel x not empty?
        if (mybarrels.getBarrel(x).measure_ultrasonic() > BARREL_LOW_LIMIT ){

        }
        
        // assign flow counter to barrel x
        // open barrel x tap
        // start pump2
        // wait until barrel x empty
        // stop pump2
        // close barrel x tap
        // init save task
        // goto next barrel set x = x-1
        // all storage barrels empty? 
            // wait 1 minute - loop
            // set x to 5
    }
    // drainCounter == 0
        // set draining status off
        // init save task
        // exit
}


























/* initial sequnence */
void setup() {

    // initiate UART0 - serial output
    OUT_PORT.begin(115200);

    // initialize UART2 - serial MUX
    Serial2.begin(9600, SERIAL_8N1);

    //WiFiManager Local intialization. Once its business is done, there is no need to keep it around
    AsyncWiFiManager wifiManager(&server,&dns);
    //wifiManager.resetSettings();
    wifiManager.autoConnect("AutoConnectAP"); // will stop here if no wifi connected
    OUT_PORT.printf("ESSID: %s\r\n", WiFi.SSID().c_str());

    //initiate expander ports - I2C wire
    expanderInit();

    // initiate SD Card - SPI bus + SPIFFS
    initStorage();

    // initialize uart2 SIM800L modem at MUX port 7?


    // setup NTP


    //start Web Server
        server.begin();
        ArduinoOTA.setHostname("barrels");
        ArduinoOTA.begin();

    // attach interrupts for flow sensors


    // read structs from sdcard

    // Create tasks

}

 





  
void loop() {
    // disable loop watchdog - working with tasks only?
    ArduinoOTA.handle();
}