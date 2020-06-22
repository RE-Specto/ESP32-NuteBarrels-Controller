/*
to implement:
*implement webserver
apply test code to check every part of the system:
    relays manual - on off
    flow sensor data - counter + flow/second
    pressure sensor data - live
    ultrasonic data - live
    rgb led
    start-stop
    sim800L - send sms
make filling mixing storing draining into single Task?
reimplement global save function:
    wait STOPPED_STATE, wait untill flow stopped, save 
pressure sensors - "stop pump on overpressure" Task
barrels - rewrite
ultrasonic - move to barrels?
deprecate RTC? replace with ntp + timeAlarms?
add everything to setup 
start/stop interrupts
    start sets canMix global boolean
        task or loop reads the bool - changes RGB immediately, but start mixing task only when no tasks running
    stop sets shouldStop global boolean
        task or loop reads the bool - changes RGB immediately, triggers stop and save
rgb led status (read from system status via error-reporting task?)
work on system hardware 
sendSMS should return actual error names



optional:
sendSMS - rewrite String to Char Array
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
#define MANUAL_STATE    0x20    // manual mode on
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

#define MUX_UNLOCKED 255 //valid shannels are 0-15

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
 // !! reimplement next version to be on per barrel limit and adjustable
#define BARREL_LOW_LIMIT 50

//Flow sensor Pin declarations
#define FLOW_1_PIN 25
#define FLOW_2_PIN 26
//#define FLOW_3_PIN 27

//Pressure sensor Pin declarations
#define PRESSUR_1_PIN 36
#define PRESSUR_2_PIN 34
//#define PRESSUR_3_PIN 35

// delay for SMS error report
#define REPORT_DELAY 5000

#define DEBUG

// Globals
File file;
WiFiUDP UDP;
AsyncWebServer server(80);
DNSServer dns;
bool isTimeSync = false;
bool isSaving = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // for interrupts and xTasks

//function declarations:
bool Load(const char* fname, byte* stru_p, uint16_t len);
bool Save(const char* fname, byte* stru_p, uint16_t len);
void SaveStructs();
void IRAM_ATTR FlowSensor1Interrupt();
void IRAM_ATTR FlowSensor2Interrupt();













// declare I/O Expander ports and constants

// MCP23017 with pin settings
MCP23017 expander1(0); // Base Address + 0: 0x20
MCP23017 expander2(1); // Base Address + 1: 0x21

class exp {
private:
    uint8_t _muxLock = MUX_UNLOCKED; // unlocked

public:
    void Init(){
        // initialize TwoWire communication
        Wire.begin();

        for (size_t i = 0; i < 16; i++) {
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

        // initial LED state
        setRGBLED(LED_WHITE);
    }

    //set analogMUX address
    void setMUX(uint8_t address){

        if ((_muxLock != address) && (_muxLock != MUX_UNLOCKED))
            OUT_PORT.printf("-MUX! previously locked to %u, %u is waiting\r\n", _muxLock, address);
        while ((_muxLock != address) && (_muxLock != MUX_UNLOCKED))
            Alarm.delay(1);

        OUT_PORT.printf("-MUX! is free. locking to %u\r\n", address);
        _muxLock = address;

        for (uint8_t i = 0; i < 4; i++){
            //checks bit 0-3 of mux Address "address"
            //if bit set - setValue receives a positive value
            //otherwise sets setValue with 0
            //offset of 0 - expander 0x20 pins a0 a1 a2 a3
            expander1.getPin(i+0).setValue( address & (1 << i) ); 
        }
        expander1.write();
    }

    uint8_t GetMUX(){
        return _muxLock;
    }

 
    // very important to run this every time you ended up business with setMUX
    void UnlockMUX(){
        if (_muxLock != MUX_UNLOCKED){
        OUT_PORT.printf("-MUX! Unlocking %u\r\n", _muxLock);
        _muxLock = MUX_UNLOCKED;
        }
        else
        OUT_PORT.println(F("-MUX! already unlocked!"));
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

} expanders; // initiated globally

















// set MUX to SIM_MUX_ADDRESS
// send sms with error code description
void SendSMS(String message){
    OUT_PORT.print("*sendsms*: ");
    OUT_PORT.println(message);
    expanders.setMUX(7); // modem is at port 7
    Alarm.delay(10); // wait until expander + mux did their job
    Serial2.println("AT+CMGS=\"+972524373724\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
    Serial2.print( message ); //text content
    Serial2.write(26); // send ctrl+z end of message
    //Serial2.print((char)26); // if the above won't work - thy this one instead
    expanders.UnlockMUX();
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

void modemInit(){
    OUT_PORT.println("modem init");
    expanders.setMUX(7); // modem is at port 7
    Alarm.delay(10); // wait until expander + mux did their job
    // Serial2.begin(9600, SERIAL_8N1); // already done in main
    Serial2.println("AT"); //Once the handshake test is successful, it will back to OK
    Serial2.println("AT+CMGF=1"); // Configuring TEXT mode
    expanders.UnlockMUX(); // Must unlock after every use!!
}

struct myST { 
    //bit field
    //00NFMSEX
    // N = maNual mode on - overrides stopped
    // F = Filling task on
    // M = Mixing task on
    // S = Storing task on
    // E = Emptying task on - overrides f,m,s
    // X = stopped status on
    uint8_t _state_now = 0;
    uint8_t _state_before = 0;

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
    uint16_t _error_now = 0;
    uint16_t _error_before = 0;
    };

class STClass{
private:
    myST myState;

public:
    bool LoadSD(){
        return Load("/SysState.bin", (byte*)&myState, sizeof(myState));
    }

    bool SaveSD(){
        return Save("/SysState.bin", (byte*)&myState, sizeof(myState));
    }

    uint8_t state_get(){
        return myState._state_now;
    }

    void state_set(uint8_t mask){
        myState._state_now |= mask;
    }

    void state_unset(uint16_t mask){
        myState._state_now &= ~mask;
    }
    // preserve prevoius state
    void state_save(){
        myState._state_before = myState._state_now;
    }

    // restore previous state
    void state_load(){
        myState._state_now = myState._state_before;
    }

    // returns true if state have "mask-bit" state on. ex: return_state(MIXING_STATE);
    bool state_check(uint8_t mask){
        //return (_state & ( 1 << position )) >> position
        //return (_state >> position) & 1 //right-shifting to position bit, and then extracting the first bit
        return myState._state_now & mask;
    }

    // returns error state
    uint16_t error_get(){
        return myState._error_now;
    }

    void error_set(uint16_t error){
        myState._error_now |=  error;
    }

    void error_unset(uint16_t error){
        myState._error_now &= ~error;
    }

    //returns the difference between current and last error states
    // !! need to reimplement to return either the new error position
    // so I dont have to use uint16_t for return
    // or either if two errors should be reported at once - a while loop that solves them one by one
    uint16_t error_getnew(){
        // bitwise - diff between "before" and "now", 
        // compare with "now" to extract new bits only
        return (myState._error_now ^ myState._error_before) & myState._error_now ; 
    }

    void error_reported(){
        myState._error_before=myState._error_now;
    }

} SystemState;

String system_error(uint16_t error){
    return "system error " + String(error);     // need to reimplement to send error description instead of error number
}

// Task 0 - Error reporting task
void errorReportTask(){
    // while there is no new errors
    while (!SystemState.error_getnew()){
        Alarm.delay(REPORT_DELAY);     //wait - endless loop untill error status changes
    }

    // goes here if there is an error to report
    uint16_t newerrors = SystemState.error_getnew();
    uint16_t counter = 1;
    while(newerrors){ // loops untill error is empty
        if(newerrors & counter){ // try each error state bit one by one
            Serial.printf("system error %u", counter);
            //SendSMS( system_error(counter) );
            newerrors-=counter; // substract what already reported
            counter *=2; // next bit
        }
    }
    // clear error after all being reported
    SystemState.error_reported();
}



























// SD Card and SPIFFS

// initiate SD Card and SPIFFS
// need to check if not getting out of scope
void initStorage(){
    while (!SD.begin()) {
        OUT_PORT.println(F("Error: Failed to initialize SD card"));
        Alarm.delay(1000);
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


struct myFS{
    volatile uint32_t counter = 0;
    uint16_t conversion_multiplier = 450;
    volatile uint32_t flow = 0;
    volatile uint32_t lastMilis = 0;
};

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

class FSClass {
private:
    myFS fsensor[2]; // two sensors
public:
    uint16_t FlowGet(uint8_t sens){
        return (fsensor[sens].flow * 1000 / fsensor[sens].conversion_multiplier); // in mililiters per second
    }

    void IRAM_ATTR CounterInc(uint8_t sens){
        fsensor[sens].counter++;
        fsensor[sens].flow = millis()-fsensor[sens].lastMilis ; // interval in miliseconds from the last call
        fsensor[sens].lastMilis = millis();
    }

    uint64_t CounterGet(uint8_t sens){
        return fsensor[sens].counter;
    }

    void CounterReset(uint_fast8_t sens){
        fsensor[sens].counter=0;
    }
    
    uint16_t MultGet(uint8_t sens){
        return fsensor[sens].conversion_multiplier;
    }

    void MultSet(uint8_t sens, uint16_t mult){
        fsensor[sens].conversion_multiplier = mult;
    }

    bool LoadSD(){
        return Load("/Flow.bin", (byte*)&fsensor, sizeof(fsensor));
    }

    bool SaveSD(){
        return Save("/Flow.bin", (byte*)&fsensor, sizeof(fsensor));
    }

    void begin(){ // attach interrupts
        pinMode(FLOW_1_PIN, INPUT_PULLUP);
        pinMode(FLOW_2_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(FLOW_1_PIN), FlowSensor1Interrupt, FALLING);
        attachInterrupt(digitalPinToInterrupt(FLOW_2_PIN), FlowSensor2Interrupt, FALLING);
    }

} flow;

//flow sensor 1 interrupt routine
void IRAM_ATTR FlowSensor1Interrupt() {
    portENTER_CRITICAL_ISR(&mux);
    flow.CounterInc(0);
    portEXIT_CRITICAL_ISR(&mux);
}
//flow sensor 2 interrupt routine
void IRAM_ATTR FlowSensor2Interrupt() {
    portENTER_CRITICAL_ISR(&mux);
    flow.CounterInc(1);
    portEXIT_CRITICAL_ISR(&mux);
}



















// Pressure Sensors Pin Declarations
// and Analog reads
// task pressure sensors - stops pumps on overpressure

struct myPS {
    uint8_t _sensorPin = 255; // defaults
    uint8_t _multiplier = 1;
    uint8_t _offset = 0;
    uint8_t _max_pressure = 255;
    uint8_t _min_pressure = 0;
    uint16_t _TooLowErr = 0;
    uint16_t _TooHighErr = 0;
};

// pressure sensors starts from 0
class pressure_sensor {
private:
    myPS psensor[2]; // two sensor
public:
    // init the sensor
    // !! need to reimplement to use error mask position instead the whole uint16_t mask
    void setSensor(uint8_t num, uint8_t sensorPin, uint16_t TooLowErr, uint16_t TooHighErr){
        psensor[num]._sensorPin = sensorPin;
        psensor[num]._TooLowErr = TooLowErr;
        psensor[num]._TooHighErr = TooHighErr;
        pinMode(sensorPin,INPUT); // initialize analog pin for the sensor
    }

    // separate functions to be called dynamically
    void setMultiplier(uint8_t num, uint8_t mult){
        psensor[num]._multiplier = mult;}
    void setOffset(uint8_t num, uint8_t offs){
        psensor[num]._offset = offs;}
    void setMax(uint8_t num, uint8_t max){
        psensor[num]._max_pressure = max;}
    void setMin(uint8_t num, uint8_t min){
        psensor[num]._min_pressure = min;}

    bool LoadSD(){
        return Load("/Pressure.bin", (byte*)&psensor, sizeof(psensor));
    }

    bool SaveSD(){
        return Save("/Pressure.bin", (byte*)&psensor, sizeof(psensor));
    }

    // read sensor - convert analog value to psi
uint16_t measure(uint8_t num){
    // convert analog value to psi
    uint16_t pressure = (analogRead(psensor[num]._sensorPin) * psensor[num]._multiplier - psensor[num]._offset) /1000; //test - needs to be replaced with acrual formula
    if (pressure < psensor[num]._min_pressure) {
        // set underpressure error
        SystemState.error_set(psensor[num]._TooLowErr);
    }
    if (pressure > psensor[num]._max_pressure) {
        // set overpressure error
        SystemState.error_set(psensor[num]._TooHighErr);
    }

    return pressure;
}

} pressure;



























struct st {
        uint16_t prefill_requirement = 100; // initial 100 liters
        uint16_t prefill_counter = 0;
        uint16_t afterfill_requirement = 400; // additional 400 liters
        uint16_t afterfill_counter = 0;
        uint16_t mix_requirement = 30; // minutes to mix
        uint16_t mix_counter = 0;
        // transfer requirement calculated dynamically by barrel ammount
        // transfer counter applied dynamically at transfer from source to destination
        uint16_t drain_requirement = 0; //in liters - how much to drain
        uint16_t drain_counter = 0;    //in liters - how much left to drain
        uint8_t filling_barrel = 0; // mixer barrel
        uint8_t mixin_barrel = 0;
        uint8_t storing_barrel = 1; // first barrel
        uint8_t draining_barrel = NUM_OF_BARRELS-1; // last barrel (-1 cause we start from zero)
} Transfers;

void Fill(uint16_t barrel, uint16_t requirement){
        // if barrel ammount less than requirement
        // open barrel filling tap
        // fill until requirement OR barrel_high_level
            // decrement requirement by flow counter
            // break if stopped but not manual
        // close mixer tap
}

// Filling task
void FillingTask(){
    while (true){ // endless loop Filling task
        // stopped status off?
        // check water level is prefill_requirement
            // filling(filling_barrel, prefill_requirement)
        // set stopped status on
        // set mixing status on
        // wait untill not stopped status
        // if water level is afterfill_requirement
            // filling(filling_barrel, afterfill_requirement)
        // set filling status off
        // init save
    } // endless loop Filling task
}

void Mix(uint16_t barrel, uint16_t requirement){
    // open barrel drain tap
    // open barrel store tap
    // open dIN tap
    // start pump
    // loop - while requirement > 0
        // decrement requirement every minute
        // break if stopped but not manual
    // if requirement reached 0
        // stop pump
        // close barrel drain tap
        // close dIN tap
        // close barrel store tap
        // init save 
}

// Mixing task
void MixingTask(){
    while(true){ // endless loop Mixing task
        // stopped status off?
            // Mixing(mixin_barrel, mix_requirement)
        // Storing status on
        // mixing status off
        // init save 
    } // endless loop Mixing task
}


void Store(uint16_t barrel, uint16_t target, uint16_t requirement){
    // open barrel drain tap
    // open target store tap
    // open dIN tap
    // start pump
    // loop - while requirement > 0 OR barrel not empty OR target not full
        // decrement requirement by flow counter
        // truncate flow counter from barrel
        // append flow counter to target
        // break if stopped but not manual
    // if requirement reached 0
        // stop pump
        // close barrel drain tap
        // close dIN tap
        // close target store tap
        // init save 
}

// Storing task
void StoringTask(){
    while (true){ // endless loop Storing task
        // stopped status off?
        // measure barrel 0
            // while barrel 0 not empty
                // measure barrel storing_barrel
                // if not full
                    // Storing(0, storing_barrel, mybarrels.getBarrel(0).measure_ultrasonic() ) // store all barrel 0 amount
                // goto next barrel set storing_barrel += 1
                // all barrels full?
                    // loop - let draining task to kick in when required
            // if barrel 0 empty
                // if drain_counter 
                    // draining status on
                    // keep storing status on so when dtaining finished it goes back here and do "else"
                // else
                    // filling status on
                    // storing status off
    } // endless loop Storing task
}

void Drain(uint16_t barrel, uint16_t requirement){
        // assign flow counter to barrel x
    // open barrel x drain tap + dOUT tap
    // start pump
    // loop until drain counter zero or barrel x empty
    while ( requirement && (mybarrels.getBarrel(barrel).measure_ultrasonic() > BARREL_LOW_LIMIT ) ){ // reimplement to use low_level
        // truncate flow counter from a barrel
        // decrement requirement by flow counter

        // check for STOPPED state change
        if (SystemState.state_check(STOPPED_STATE) && !SystemState.state_check(MANUAL_STATE)) // break if stopped but not manual
            break;  // breaks the measure loop, stops pump, not decrement the x
                    //stays in the while STOPPED above
        Alarm.delay(1000); 
    }
    // stop pump
    // close barrel x tap
    Alarm.delay(1000); // untill pressure released
    // close dOUT tap
    // init save
    SaveStructs(); 
}

// Draining task
void DrainingTask(){
    while (true){ // endless loop Draining task

        while (Transfers.drain_counter  && SystemState.state_check(DRAINIG_STATE)){ // have a need to drain
            while (SystemState.state_check(STOPPED_STATE)) // stay here if system stopped
                Alarm.delay(1000); // check every second


            // measure barrel x - Barrel x not empty?
            if (mybarrels.getBarrel(Transfers.draining_barrel).measure_ultrasonic() > BARREL_LOW_LIMIT ){  // reimplement to use low_level
                Drain(Transfers.draining_barrel, Transfers.drain_requirement);
            }


            else { // still in the draincounter > 0 loop -  barrel x is empty              
                if (Transfers.draining_barrel > 1) // if not the first barrel (i started from last to first)
                    Transfers.draining_barrel--; // goto next barrel
                else { // all storage barrels empty? 
                // unset DRAINING state temporarely so fms can kick in.
                SystemState.state_unset(DRAINIG_STATE);
                SystemState.state_load(); // get the last state back f? m? s? stopped or not..
                // init save 
                SaveStructs();
                // set x back to last barrel
                Transfers.draining_barrel = NUM_OF_BARRELS-1; //  -1 cause we start from zero
                }

            }        
        } //Transfers.drain_counter  && SystemState.state_check(DRAINIG_STATE)
        
        // i"m here because drainCounter reached 0 - or not DRAINIG_STATE
        // if still DRAINIG_STATE then unset DRAINIG_STATE and undo to last state before draining - back to fms
        if (SystemState.state_check(DRAINIG_STATE)) { 
            // set draining status off
            SystemState.state_unset(DRAINIG_STATE);
            // undo to last state before draining
            SystemState.state_load();
            // init save 
            SaveStructs();
        }
        // wait untill draining state set - goes here at system start cause DRAINIG_STATE not set
        if (!SystemState.state_check(DRAINIG_STATE))
            OUT_PORT.println(F("draining task entering wait-mode"));
        while ( !SystemState.state_check(DRAINIG_STATE))
            Alarm.delay(1000);



    } // endless loop Draining task
}


void fmsexTask(){ // Filling Mixing Storing Emptying
// fill
//Filling();
// send sms?
// wait for start - set STOPPED. 
// mix
//Mixing();
// store
//Storing();
// repeat
//Draining();

// if drain command - all others stops
        //remember last state (including stopped) 
        //remove STOPPED state - drain 
        
        //!continue fms tasks

        // what should happen if trying to drain more than available?
        // right now drain is looping endlessly
        // can I pause drain and continue after one fms cycle?

}
































// Ultrasonic sensors
// declare UART2 
// initiate UART2 - ultrasonic sensors MUX
void initUltrasonic(){
    
}

//while (Serial2.available())


// declare each barrel's ultrasonic sensor address in analog mux

// loop thru all barrels using setMUX function (from expander)
// unlock MUX































// WebServer

//https://github.com/me-no-dev/ESPAsyncWebServer

void setupServer(){


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


    server.on("/upload", HTTP_GET, [](AsyncWebServerRequest *request) {
    const char* serverIndex = "<form method='POST' action='/upload' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Upload'></form>";
    request->send(200, "text/html", serverIndex);
    });


    server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
        //request->send(200);
        //request->send(200, "text/plain", "OK");
        }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
            if(!index){
                Serial.printf("\r\nUpload Started: %s\r\n", filename.c_str());
                // open the file on first call and store the file handle in the request object
                request->_tempFile = SD.open("/"+filename, "w");
            }
            if(len) {
                Serial.printf("received chunk [from %u to %u]\r\n", index, len);
                // stream the incoming chunk to the opened file
                request->_tempFile.write(data,len);
            }       
            if(final){
                Serial.printf("\r\nUpload Ended: %s, %u Bytes\r\n", filename.c_str(), index+len);
                request->_tempFile.close();
                request->send(200, "text/plain", "File Uploaded !");
            }
        }
    );

/*
  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request){
    if(request->hasArg("file")){
      char buff[32]; // next implementation add leading "/"
      request->arg("file").toCharArray(buff, sizeof(buff), 0);
      if (fileSystem->exists(buff)){
        OUT_PORT.printf("\rDownloading file: %s\r\n", buff);
        File file = fileSystem->open(buff, "r");
        if (file){
          //request->sendHeader("filename", buff); // not working
          size_t sentsize = request->streamFile(file, "application/octet-stream");
          OUT_PORT.printf( "Sent %u out of %u Bytes\r\n",sentsize, file.size() );
        }
        else
          request->send(200, "text/plain", "error opening file");
        file.close();
      }
      else {// exist returned false
        request->send(200, "text/plain", "file not exist");
        OUT_PORT.printf("\rfile %s not exist\t~\r\n", buff);
      }
    }
    else {// no argumet "file" supplied
      request->send(200, "text/plain", "missing parameter \"file\"");
      OUT_PORT.print(F("\rDownload - missing parameter \"file\"\r\n"));
    }
  });

*/


  // Start server
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.begin();
} // void setupServer() end




















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























void LoadStructs(){
    if (SD.exists("/SysState.bin"))
        SystemState.LoadSD();
    else
        OUT_PORT.println("/SysState.bin not exist");
        
    if (SD.exists("/Pressure.bin"))
        pressure.LoadSD();
    else
        OUT_PORT.println("/Pressure.bin not exist");

    if (SD.exists("/Flow.bin"))
        flow.LoadSD();
    else
        OUT_PORT.println("/Flow.bin not exist");
}


void SaveStructs(){
    if (!isSaving){ // prevent concurrent saving
    isSaving = true;
    SystemState.SaveSD();
    //Barrels.SaveSD(); // important! need2implement

    // not required?
    //pressure.SaveSD();
    //flow.SaveSD();
    isSaving = false;
    }
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
    expanders.Init();

    // initiate SD Card - SPI bus + SPIFFS
    initStorage();


    // load structs from SD card
    LoadStructs();

    pressure.setSensor(0, PRESSUR_1_PIN, PS1TOOHIGH_ERROR, PS1TOOLOW_ERROR);
    pressure.setSensor(1, PRESSUR_2_PIN, PS2TOOHIGH_ERROR, PS2TOOLOW_ERROR);


    //bug? testing..
    expanders.UnlockMUX();

    // initialize uart2 SIM800L modem at MUX port 7?
    modemInit();

    // setup NTP


    //start Web Server
    setupServer();
    ArduinoOTA.setHostname("barrels");
    ArduinoOTA.begin();

    // attach interrupts for flow sensors


    // Create tasks
    /*
    xTaskCreatePinnedToCore(myTask, "loop1", 4096, (void *)1, 1, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(myTask, "loop2", 4096, (void *)2, 1, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(myTask, "loop3", 4096, (void *)3, 1, NULL, ARDUINO_RUNNING_CORE);

    void myTask(void *pvParameters) {
        int taskno = (int)pvParameters;
        int sleeptime;
        while (1) {
            sleeptime = (int)(esp_random()&0x0F);
            Serial.println(String("Task ")+String(taskno)+String(" ")+String(sleeptime));
            delay(500+sleeptime*100);
        }
    }

    void loop() {
    // nope, do nothing here
    vTaskDelay(portMAX_DELAY); // wait as much as posible ...
    }

    */


    SendSMS("System Started");
}

 





  
void loop() {
    // disable loop watchdog - working with tasks only?
    ArduinoOTA.handle();
}