//#include "main.h"
#include "PresureSensor.h"
#include "Expanders.h"
#include "SystemState.h"
// #include "Barrels.h"
// #include "FlowSensor.h"
// #include "PresureSensor.h"
// #include "WebServer.h"
#include "Filesystem.h"
#include "Modem.h"
// #include "NTPClient.h"
// #include "FMSD.h"
#include "globals.h"

/*-------- PressureSensor Begin ----------*/
// Pressure Sensors Pin Declarations
// and Analog reads
// task pressure sensors - stops pumps on overpressure

bool PSClass::LoadSD() { return Storage.Load("/Pressure.bin", (byte *)&iPsens, sizeof(iPsens)); }

bool PSClass::SaveSD() { return Storage.Save("/Pressure.bin", (byte *)&iPsens, sizeof(iPsens)); }

void PSClass::begin()
{
    setSensor(FRESHWATER, PRESSUR_1_PIN);
    setSensor(NUTRIENTS, PRESSUR_2_PIN);
}

// init the sensor
void PSClass::setSensor(byte sens, byte sensorPin)
{
    // sensors now start from 1 (but arrays from 0)
    sens--;
    iPsens[sens]._sensorPin = sensorPin;
    pinMode(sensorPin, INPUT); // initialize analog pin for the sensor
}

byte PSClass::Divider(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return iPsens[sens]._divider; 
}

void PSClass::DividerSet(byte sens, byte div) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    if (div)
    {
        LOG.printf("Changing pressensor %u divider, from:%u to:%u\r\n", sens+1, iPsens[sens]._divider, div);
        iPsens[sens]._divider = div; 
    }
    else
    {
        LOG.println("Uh-oh.. somebody just divided by zero O.o");
    }
}

int16_t PSClass::Offset(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return iPsens[sens]._offset; 
}

void PSClass::OffsetSet(byte sens, int16_t offs) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    iPsens[sens]._offset = offs; 
}

// gets maximal normal pressure point
byte PSClass::Max(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return iPsens[sens]._max_pressure; 
}

// sets maximal normal pressure point
void PSClass::MaxSet(byte sens, byte max) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    LOG.printf("Changing pressensor %u max pressure point, from:%u to:%u\r\n", sens+1, iPsens[sens]._max_pressure, max);
    iPsens[sens]._max_pressure = max; 
}

// gets minimal normal pressure point
byte PSClass::Min(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return iPsens[sens]._min_pressure; 
}

// sets minimal normal pressure point
void PSClass::MinSet(byte sens, byte min) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    LOG.printf("Changing pressensor %u min pressure point, from:%u to:%u\r\n", sens+1, iPsens[sens]._min_pressure, min);
    iPsens[sens]._min_pressure = min; 
}

// returns last value measured by pressure sensor (sens)
int16_t PSClass::LastValue(byte sens)
{
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return iPsens[sens]._last_pressure;
}

// read sensor (sens) - converts analog value to pressure
int16_t PSClass::measure(byte sens)
{
    /*
    short value: 409.5, short >= 46.5ma 3.3v
    20.3ma value: 253.8, 2.31v
    slight pressure: 53.9
    atmospheric pressure=4ma, 4.04ma value: 51.1-53.1, 0.58v
    slight vacuum: 43.5
    disconnect value: 0, v0 0ma
    */
    // sensors now start from 1 (but arrays from 0)
    sens--;
    sPressure *p = &iPsens[sens];
    // https://forum.arduino.cc/index.php?topic=571166.0
    analogRead(p->_sensorPin); // discard first value
    // convert analog value to pressure
    LOG.printf("measuring pSensor %u @pin%u calib:%u value:", sens + 1, p->_sensorPin, analogRead(p->_sensorPin)); // second value to serial mon
    uint16_t pres = (analogRead(p->_sensorPin) / p->_divider + p->_offset);                                    // third value goes into evaluation
    p->_last_pressure = pres; // storing value for reuse
    LOG.println(pres);

    //4  short circuit
    if (pres > 255)
    {
        // stop pump
        Expanders.Pump(false);
        vTaskDelay(100);
        Expanders.Protect(true);
        // set error
        if (p->_error_state != PRESSURE_SHORTCIRCUIT)
        {
            p->_error_state = PRESSURE_SHORTCIRCUIT;
            LOG.printf("[E] sensor %u short circuit\r\n", sens + 1);
            SendSMS("short circuit @pressure sensor", sens + 1);
        }
        return pres; // exits here
    }

    //3  disconnected
    if (pres < 20)
    {
        // stop pump
        Expanders.Pump(false);
        vTaskDelay(100);
        Expanders.Protect(true);
        // set error
        if (p->_error_state != PRESSURE_DISCONNECT)
        {
            p->_error_state = PRESSURE_DISCONNECT;
            LOG.printf("[E] sensor %u disconnected\r\n", sens + 1);
            SendSMS("lost connection @pressure sensor", sens + 1);
        }
        return pres; // exits here
    }

    //2  overpressure
    if (pres > p->_max_pressure && pres < 255)
    {
        // stop pump
        Expanders.Pump(false);
        vTaskDelay(100);
        Expanders.Protect(true);
        // set error
        if (p->_error_state != PRESSURE_OVERPRESSURE)
        {
            p->_error_state = PRESSURE_OVERPRESSURE;
            LOG.printf("[E] sensor %u overpressure\r\n", sens + 1);
            SendSMS("Overpressure @pressure sensor", sens + 1);
        }
        return pres; // exits here
    }

    //1  no pressure
    if (pres < p->_min_pressure)
        //if (p->_error_state == PRESSURE_NORMAL)
        p->_error_state = PRESSURE_NOPRESSURE;

    //0  normal pressure range is here
    if (pres > p->_min_pressure && pres < p->_max_pressure)
        //if (p->_error_state == PRESSURE_NOPRESSURE)
        p->_error_state = PRESSURE_NORMAL;

    // set system-wide pressure error if critical error
    if (p->_error_state > 1)
    {
        Expanders.setRGBLED(LED_RED);
        if (sens == 0)
            State.SetError(ERR_WATER_PRESSURE);
        else
            State.SetError(ERR_NUTRI_PRESSURE);
    }
    return pres;
}

// error handling
//0  normal pressure
//1  no pressure
//2  overpressure
//3  disconnected
//4  short circuit
byte PSClass::Errors(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return iPsens[sens]._error_state; 
}

// disable protect
// unset system error
// remeasure
void PSClass::Reset(byte sens)
{
    LOG.printf("removing pressure error @sensor:%u\r\n", sens);
    // sensors now start from 1 (but arrays from 0)
    sens--;
    Expanders.Protect(false);
    iPsens[sens]._error_state = 0;
    if (!sens) // sensor-0 freshwater
        State.UnsetError(ERR_WATER_PRESSURE);
    else // sensor-1 nutrients
        State.UnsetError(ERR_NUTRI_PRESSURE);
    // back to one - cause we calling function that also counts from one
    sens++;
    // remeasure to check sensor health
    if (Enabled(sens))
        measure(sens);
}

// returns sensor checks state
bool PSClass::Enabled(byte sens) 
{ 
    // sensors now start from 1 (but arrays from 0)
    sens--;
    return iPsens[sens]._enabled;
}

// sets sensor checks on
void PSClass::Enable(byte sens) 
{
    // sensors now start from 1 (but arrays from 0)
    sens--;
    iPsens[sens]._enabled = true; 
    LOG.printf("pressensor %u Enabled\r\n", sens+1);
}

// sets sensor checks off
void PSClass::Disable(byte sens)
{
    // sensors now start from 1 (but arrays from 0)
    sens--;
    iPsens[sens]._enabled = false; 
    LOG.printf("pressensor %u Disabled\r\n", sens+1);
}
/*-------- PressureSensor END ----------*/
PSClass Pressure;
