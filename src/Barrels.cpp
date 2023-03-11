//#include "main.h"
#include <Arduino.h>
#include "Barrels.h"
#include "Filesystem.h"
#include "FlowSensor.h"
#include "Expanders.h"
#include "SystemState.h"
#include "Modem.h"
#include "globals.h"

/*-------- Barrels Begin ----------*/
bool BarrClass::LoadSD() { return Filesys.Load("/Barrels.bin", (byte *)&iBarrel, sizeof(iBarrel)); }

bool BarrClass::SaveSD() { return Filesys.Save("/Barrels.bin", (byte *)&iBarrel, sizeof(iBarrel)); }

// returns barrel error state
byte BarrClass::Errors(byte barrel) { return iBarrel[barrel]._error_state; }

// checks for specific error by mask
bool BarrClass::ErrorCheck(byte barrel, byte mask) { return iBarrel[barrel]._error_state & mask; }

// sets barrel error by mask
void BarrClass::ErrorSet(byte barrel, byte mask)
{
    iBarrel[barrel]._error_state |= mask;
    LOG.printf("[E] Barr%u:e%u\r\n", barrel, mask);
    if (!SaveSD()) { LOG.println("[E] unable to Save ErrorSet"); } // just warn for now..
}

// unsets barrel error by mask
void BarrClass::ErrorUnset(byte barrel, byte mask)
{
    iBarrel[barrel]._error_state &= ~mask;
    if (!SaveSD()) { LOG.println("[E] unable to Save ErrorUnset"); } // just warn for now..
}

// removes error state from barrel
void BarrClass::Reset(byte barrel) 
{ 
    LOG.printf("Barrel %u error state reset, from:%u to:0\r\n", barrel, iBarrel[barrel]._error_state);
    iBarrel[barrel]._error_state = 0; 
    if (!SaveSD()) { LOG.println("[E] unable to Save Reset"); } // just warn for now..
}

// dangerous - override barrel error
void BarrClass::ErrorOverride(byte barrel, byte error) 
{ 
    LOG.printf("Barrel %u error state overridden, from:%u to:%u\r\n", barrel, iBarrel[barrel]._error_state, error);
    iBarrel[barrel]._error_state = error; 
    if (!SaveSD()) { LOG.println("[E] unable to Save ErrorOverride"); } // just warn for now..
}

// freshwater counted by flow for (barrel)
uint32_t BarrClass::FreshGet(byte barrel) { return iBarrel[barrel]._volume_freshwater; }

// nutrients counted by flow for (barrel)
uint32_t BarrClass::NutriGet(byte barrel) { return iBarrel[barrel]._volume_nutrients; }

// not recommended - use with caution
void BarrClass::FreshOverride(byte barrel, uint32_t volume)
{
    iBarrel[barrel]._volume_freshwater_last = volume;
    iBarrel[barrel]._volume_freshwater = volume;
}

// not recommended - use with caution
void BarrClass::NutriOverride(byte barrel, uint32_t volume)
{
    iBarrel[barrel]._volume_nutrients_last = volume;
    iBarrel[barrel]._volume_nutrients = volume;
}

// // decrease barrel by drained ammount
// void BarrClass::NutriLess(byte barrel, uint32_t liters)
// {
//     iBarrel[barrel]._volume_nutrients -= liters;
//     iBarrel[barrel]._volume_nutrients_last = iBarrel[barrel]._volume_nutrients;
// }

// add fresh flow couter to barrel, then Subtract it from flowsensor
// decrease ammo if manual
// decrease bypass req if filling pools not mixer
void BarrClass::FreshwaterFillCalc(byte barrel)
{
    sBarrel *b = &iBarrel[barrel];
    b->_volume_freshwater = b->_volume_freshwater_last; // safeguard
    uint32_t tempflow = Flow.Counted(FRESHWATER); // may be increased during calculation because flowsensor works on interrupts
    #ifdef DEBUG_FLOW
    if (tempflow)
    {
        LOG.printf("tempflow so far: %u\r\n", tempflow);
        tempflow /= Flow.Divider(FRESHWATER); // integral part, fractional part discarded.
        LOG.printf("tempflow in liters: %u\r\n", tempflow);
        b->_volume_freshwater += tempflow;
        State.DecreaseManual(tempflow); // checks if needed
        if (barrel > 0) // if not mixer
            State.DecreaseBypass(tempflow);
        tempflow *= Flow.Divider(FRESHWATER); // getting pulse count back - only the integral part
        LOG.printf("tempflow integral part: %u\r\n", tempflow);
        Flow.CounterSubtract(FRESHWATER, tempflow);              // counter 1 is freshwater
    }
    else
    {
        LOG.println("0 flow so far. skipping");
    }
    #else
    if (tempflow / Flow.Divider(FRESHWATER))
    {
        tempflow /= Flow.Divider(FRESHWATER); // integral part, fractional part discarded.
        b->_volume_freshwater += tempflow;
        State.DecreaseManual(tempflow); // checks if needed
        if (barrel > 0) // if not mixer
            State.DecreaseBypass(tempflow);
        tempflow *= Flow.Divider(FRESHWATER); // getting pulse count back - only the integral part
        Flow.CounterSubtract(FRESHWATER, tempflow);
    }
    #endif
    b->_volume_freshwater_last = b->_volume_freshwater; // after
}

// //https://sciencing.com/calculate-concentration-solution-different-concentrations-8680786.html
// // concentration of nutrient solution
// // ((concA/100 *LitersA) + (concB/100 *LitersB)) / LitersA + LitersB  * 100%
// // concB is allways 0 ( b is freshwater) since that is irrelevant, so:
// // (concA/100 *LitersA)  / LitersA + LitersB  * 100%
// // concA * LitersA  / LitersA + LitersB
// float BarrClass::ConcentrationTotal(byte barrel)
// {
//     sBarrel *b = &iBarrel[barrel];
//     if (!b->_volume_nutrients)
//         return 0; // no nutes to calc + prevent Divide By Zero! on dry barrels
//     else
//         return b->_concentraion * b->_volume_nutrients / (b->_volume_nutrients + b->_volume_freshwater); // mix
// }

// // calc new concentration:
// // counts all as nutrinets after calculation, at concentration X
// void BarrClass::ConcentrationRecalc(byte barrel)
// {
//     sBarrel *b = &iBarrel[barrel];
//     if (b->_volume_freshwater)
//     {
//         if (b->_concentraion_last != b->_concentraion)
//             b->_concentraion = b->_concentraion_last; // safeguard
//         b->_concentraion = ConcentrationTotal(barrel);
//         b->_volume_nutrients += b->_volume_freshwater;
//         b->_volume_freshwater = 0;
//         b->_concentraion_last = b->_concentraion;
//     }
//     else
//     {
//         LOG.println(F("[W] ConcentrationRecalc called but already calculated"));
//     }
// }

// // checks if current barrel freshwater/nutrients level is at/above target
// // uses barrel current level + flow counted level
// // does not apply or decrease flow count!
// bool BarrClass::isFillTargetReached(byte barrel, byte type, uint16_t target)
// {
//     sBarrel *b = &iBarrel[barrel];
//     uint16_t tempLiters = 0;
//     //check target barrel current state
//     if (type == FRESHWATER)
//         tempLiters += b->_volume_freshwater;
//     else
//         tempLiters += b->_volume_nutrients;
//     //add data from flowsensor
//     tempLiters += Flow.Counted(type) / Flow.Divider(type);
//     //check if target reached
//     return tempLiters >= target;
// }


// count mixed water as nutrients
void BarrClass::WaterToNutrients(byte barrel)
{
    sBarrel *b = &iBarrel[barrel];
    uint32_t tempflow = b->_volume_freshwater; // preserve so not changed by interrupts
    b->_volume_nutrients += tempflow;
    b->_volume_freshwater -= tempflow;
    b->_volume_nutrients_last = b->_volume_nutrients;
    b->_volume_freshwater_last = b->_volume_freshwater;
}

// Subtracting flowcount from source, adding to destination
// decrease ammo if manual
void BarrClass::NutrientsTransferCalc(byte from, byte to)
{
    sBarrel *a = &iBarrel[from];
    sBarrel *b = &iBarrel[to];

    // // recalculating concentration @destination
    // // run less often for concentration accuracy
    // // calc new concentration before transer so transfeting only nutrients @concentration
    // if (a->_volume_freshwater)
    //     ConcentrationRecalc(from);
    // if (b->_volume_freshwater)
    //     ConcentrationRecalc(to);
    // // both barrels now have only Nutrients @ concentration

    // safeguards
    a->_volume_nutrients = a->_volume_nutrients_last;
    b->_volume_nutrients = b->_volume_nutrients_last;

    // a->_concentraion = a->_concentraion_last;
    // b->_concentraion = b->_concentraion_last;

    uint32_t tempflow = Flow.Counted(NUTRIENTS); // may be increased during calculation because flowsensor works on interrupts
    if (tempflow / Flow.Divider(NUTRIENTS)) // prevents DivideByZero below
    {
        tempflow /= Flow.Divider(NUTRIENTS);         // integral part, fractional part discarded.
        LOG.printf("transfering %u liters from barrel %u to %u\r\n", tempflow, from, to);
        LOG.printf("source barrel %u before: %u, target barrel %u before: %u\r\n", from, a->_volume_nutrients, to, b->_volume_nutrients);
        // // barrel b concentration =  (concentrationA/100 *Aliters) + (concentrationB/100 *Bliters)   / (Aliters + Bliters)  * 100%
        // // recheck this implementation !!! should I calc before adding counter?
        // LOG.printf("concentration before: %f\r\n", b->_concentraion);
        // b->_concentraion = ((a->_concentraion / 100 * tempflow) + (b->_concentraion / 100 * b->_volume_nutrients)) / (tempflow + b->_volume_nutrients) * 100;
        // // recheck this implementation !!!
        // // is "_Concen /100 * tflow" same as "_Concen * tflow /100" considering uneven float point calculation?
        // // "Do not use float to represent whole numbers." http://www.cplusplus.com/forum/general/67783/

        if (a->_volume_nutrients < tempflow)
        {
            LOG.printf("[E] trying to decrease barrel %u nute level below zero", from);
            a->_volume_nutrients == 0;
        }
        else
        {
            a->_volume_nutrients -= tempflow;
        }
        if ((b->_volume_nutrients + tempflow) > UINT32_MAX)
        {
            LOG.printf("[E] trying to decrease barrel %u nute level above UINT32_MAX", to);
            b->_volume_nutrients == UINT32_MAX;
        }
        else
        {
            b->_volume_nutrients += tempflow;
        }
        State.DecreaseManual(tempflow); // checks if needed
        tempflow *= Flow.Divider(NUTRIENTS);     // getting pulse count back - only the integral part
        Flow.CounterSubtract(NUTRIENTS, tempflow); // counter 2 is nutrients
    }
    else
    {
        LOG.println("0 flow so far. skipping");
    }

    a->_volume_nutrients_last = a->_volume_nutrients;
    b->_volume_nutrients_last = b->_volume_nutrients;
    // a->_concentraion_last = a->_concentraion;
    // b->_concentraion_last = b->_concentraion;
    if (tempflow)
    {
        LOG.printf("source barrel %u after: %u, target barrel %u after: %u\r\n", from, a->_volume_nutrients, to, b->_volume_nutrients);
        // LOG.printf("concentration after: %f\r\n", b->_concentraion);
    }
}

uint32_t BarrClass::VolumeMax(byte barrel)
{
    return iBarrel[barrel]._volume_max;
}

uint16_t BarrClass::VolumeMin(byte barrel)
{
    return iBarrel[barrel]._volume_min;
}

void BarrClass::VolumeMaxSet(byte barrel, uint16_t volume)
{
    LOG.printf("Barrel %u changing max volume, from:%u to:%u\r\n", barrel, iBarrel[barrel]._volume_max, volume);
    iBarrel[barrel]._volume_max = volume;
    if (!SaveSD()) { LOG.println("[E] unable to Save VolumeMaxSet"); } // just warn for now..
}

void BarrClass::VolumeMinSet(byte barrel, uint16_t volume)
{
    LOG.printf("Barrel %u changing min volume, from:%u to:%u\r\n", barrel, iBarrel[barrel]._volume_min, volume);
    iBarrel[barrel]._volume_min = volume;
    if (!SaveSD()) { LOG.println("[E] unable to Save VolumeMinSet"); }  // just warn for now..
}


// contact the sensor via UART, measure, store max distance in mm
// receives:
// barrel number,
// measure x times - default 10, 
// max measuring time - default  1000 miliseconds,
// retries on failure - default 5
void BarrClass::SonicMeasure(byte barrel, byte measure, uint16_t timeLeft, byte retryLeft)
{ // the hedgehog :P
    #ifdef DEBUG_SONIC
    uint16_t temptimeout = timeLeft;
    byte tempretry = retryLeft;
    #endif
    sBarrel *b = &iBarrel[barrel];
    // collect "measure" successful measurements to calculate total
    // wait "timeLeft" ms total time for sonic data
    // try "retryLeft" times on no data or checksum error
    uint32_t distanceAvearge = 0;  // avearge of x measurements
    uint16_t distanceMin = 0xffff; // highest for 16bit uint
    uint16_t distanceMax = 0;      // to calculate error
    #ifdef DEBUG_SONIC
    LOG.printf("measuring barrel# %u\r\n", barrel);
    #endif
    Expanders.LockMUX(barrel);
    vTaskDelay(10);
    for (byte x = 0; x < measure && retryLeft && timeLeft;)
    {
        // send data so sonic will reply
        Serial2.write(0x55);
        // wait until some data is received
        while (!Serial2.available() && timeLeft)
        {
            vTaskDelay(1);
            if (timeLeft) // prevent integer overflow
                timeLeft--;
        };
        // discard data until begin of packet (0xFF)
        while (Serial2.read() != 0xFF && timeLeft)
        {
            vTaskDelay(1);
            if (timeLeft) // prevent integer overflow
                timeLeft--;
        };
        // wait for all data to be buffered
        while (Serial2.available() < 3 && timeLeft)
        {
            vTaskDelay(1);
            if (timeLeft) // prevent integer overflow
                timeLeft--;
        };
        // timed out waiting for 3 packats above
        if (!timeLeft)
        {
            // number of measurements so far (excluding the last "timed out" measurement)
            measure = x;
            break;
        }
        byte upper_data = Serial2.read();
        byte lower_data = Serial2.read();
        byte sum = Serial2.read();
        // Serial.printf("high %u low %u sum %u\r\n", upper_data, lower_data, sum); // for debug
        if (((0xFF + upper_data + lower_data) & 0xFF) == sum) // fix to match JSN-SR04T-2.0 checksum calculation
        {
            uint16_t distance = (upper_data << 8) | (lower_data & 0xFF);
            #ifdef DEBUG_SONIC
            LOG.printf("Sonic:%u Distance:%umm measurement:%u time left:%u retries left:%u\r\n", barrel, distance, x+1, timeLeft, retryLeft);
            #endif
            if (distance)
            {
                if (distance == 10555)
                {
                    measure = x; // number of measurements so far (excluding the last "out of range" measurement)
                    distanceAvearge = 0;
                    if (!ErrorCheck(barrel, BARREL_SONIC_OUTOFRANGE))
                        ErrorSet(barrel, BARREL_SONIC_OUTOFRANGE); 
                    break;
                }
                else
                {
                    distanceAvearge += distance;
                    if (distanceMin > distance)
                        distanceMin = distance;
                    if (distanceMax < distance)
                        distanceMax = distance;
                    x++; // success - decrement loop counter
                }
            }
            else
            {
                if (retryLeft) // prevent integer overflow
                    retryLeft--;
            }    
        }
        else
        {
            LOG.println("checksum error");
            if (retryLeft) // prevent integer overflow
                retryLeft--;
        }
        if (!retryLeft)
        {
            if(!ErrorCheck(barrel, BARREL_SONIC_CHECKSUM))
                ErrorSet(barrel, BARREL_SONIC_CHECKSUM);                    
        }
        else
            ErrorUnset(barrel, BARREL_SONIC_CHECKSUM);
    }                      // for loop end here
    Expanders.UnlockMUX(); // important!
    if (!timeLeft)
    {
        // moved over here to prevent MUX Deadlock
        if (!ErrorCheck(barrel, BARREL_SONIC_TIMEOUT))
        {
            ErrorSet(barrel, BARREL_SONIC_TIMEOUT);
            SendSMS("No signal @Ultrasonic", barrel);
        }
    }
    float deviation = 0;
    if (measure && distanceAvearge) // prevents DivideByZero below
    {                                                                           // taken more than 0 measurements, Avearge distance is not zero
        distanceAvearge /= measure;                                             // total divided by number of measurements taken
        deviation = (float)100 * ((distanceMax - distanceMin) / 2) / distanceAvearge; // calculate measurement ±error
        ErrorUnset(barrel, BARREL_SONIC_TIMEOUT);
        ErrorUnset(barrel, BARREL_SONIC_OUTOFRANGE);
        b->_sonic_last_value = distanceMax; //distanceAvearge; // set value to be used by other functions. will leave previous if measurement was bad.
    }
    if (b->_sonic_deviation < deviation)
        b->_sonic_deviation = deviation; // remember largest value
    if (deviation > SONIC_DEV)
    {
        if(!ErrorCheck(barrel, BARREL_SONIC_INACCURATE))
            ErrorSet(barrel, BARREL_SONIC_INACCURATE);
    }
    else
    {
        if(ErrorCheck(barrel, BARREL_SONIC_INACCURATE))
            ErrorUnset(barrel, BARREL_SONIC_INACCURATE);
    }
    #ifdef DEBUG_SONIC
    LOG.printf("finished\r\nsonic:%u, accepted:%u, value:%umm, time:%ums, retries:%u\r\nDistance min:%u, max:%u, diff:%u, deviation:±%.1f%%\r\n",
                    barrel,
                    measure,
                    distanceAvearge,
                    temptimeout - timeLeft,
                    tempretry - retryLeft,
                    distanceMin,
                    distanceMax,
                    distanceMax - distanceMin,
                    deviation
                    );
    #endif
    //return distanceAvearge;
} // end SonicMeasure

uint16_t BarrClass::SonicLastMM(byte barrel) { return iBarrel[barrel]._sonic_last_value; }
int16_t BarrClass::SonicLastMMfromEmpty(byte barrel) { return iBarrel[barrel]._barrel_height - iBarrel[barrel]._sonic_last_value; }
uint16_t BarrClass::SonicOffset(byte barrel) { return iBarrel[barrel]._barrel_height; }
uint32_t BarrClass::SonicMLinMMGet(byte barrel) { return iBarrel[barrel]._ml_in_mm; }

// empty barrel (full barrel length) in mm
void BarrClass::SonicOffsetSet(byte barrel, uint16_t offs) 
{ 
    LOG.printf("Barrel %u changing barrel length, from:%u to:%u\r\n", barrel, iBarrel[barrel]._barrel_height, offs);
    iBarrel[barrel]._barrel_height = offs; 
    if (!SaveSD()) { LOG.println("[E] unable to Save SonicOffsetSet"); } // just warn for now..
}

void BarrClass::SonicMLinMMSet(byte barrel, uint16_t coef) 
{ 
    LOG.printf("Barrel %u changing litrage to height coefficient, from:%u to:%u\r\n", barrel, iBarrel[barrel]._ml_in_mm, coef);
    iBarrel[barrel]._ml_in_mm = coef; 
    if (!SaveSD()) { LOG.println("[E] unable to Save SonicMLinMMSet"); } // just warn for now..
}

// sonic calculate liters of last measurement
uint32_t BarrClass::SonicCalcLiters(byte barrel)
{
    // _barrel_height = empty barrel (full barrel length) in mm
    // "full barrel length" - SonicMeasure = water level from empty in mm
    // length * _ml_in_mm / 1000mlINliter ) = current barrel volume in liters from sonic
    sBarrel *b = &iBarrel[barrel];
    //Serial.printf("barrel %u (offset %u - value %u) * MLinMM %u / 1000ml\r\n", barrel, b->_barrel_height, b->_sonic_last_value, b->_ml_in_mm);
    if (!b->_sonic_last_value)
        return 0;
    else
        return (b->_barrel_height - b->_sonic_last_value) * b->_ml_in_mm / 1000; // 1000ml in liter
}                                                                               // should I still return positive value for empty, but not dry barrel??

// calibrate point by filling 100L from dry
void BarrClass::Save100LitersMark(byte barrel)
{
    sBarrel *b = &iBarrel[barrel];
    if (b->_barrel_height - b->_sonic_last_value) // Prevent scary DivideByZero CPU Panic crash! :)
    {
        // mL in mm = mL / (mm whole length - mm now)
        b->_ml_in_mm = 100000 / (b->_barrel_height - b->_sonic_last_value);
        LOG.printf("100L calibration - Barrel:%u, %umL in one mm\r\n", barrel, b->_ml_in_mm);
        if (!SaveSD()) { LOG.println("[E] unable to Save Save100LitersMark"); } // just warn for now..
    }
    else
    {
        LOG.printf("Error! trying to set barrel:%u 100L point to same value as Dry Point\r\n", barrel);
    }
}

// barrel volume in percents, measured by sonic sensor
// no need to remeasure each call - needed real-time only at transfer - so updated anyway by transfer volume checkers below
// 100% * "current liters above min point" / "total liters above min point"
int8_t BarrClass::BarrelPercents(byte barrel)
{
    if (iBarrel[barrel]._volume_max == iBarrel[barrel]._volume_min) // prevents DivideByZero below
        return 0;
    return 100 * (SonicCalcLiters(barrel) - iBarrel[barrel]._volume_min) / (iBarrel[barrel]._volume_max - iBarrel[barrel]._volume_min);
    // exclude unusable percents below Min point
}

// total liters in all barrels excluding mixing barrel and barrels with errors
// should I remeasure all sonics before?
uint32_t BarrClass::SonicLitersTotal()
{
    int16_t result = 0;
    for (byte x = 1; x < NUM_OF_BARRELS; x++)
        if (!Errors(x))                 // if no errors at all
            result += SonicCalcLiters(x); // add this barrel content to sum
    return result;
}

// same as above but exclude unusable liters from below draining point.
// should I remeasure all sonics before?
uint32_t BarrClass::SonicLitersUsable()
{
    int16_t result = 0;
    for (byte x = 1; x < NUM_OF_BARRELS; x++)
        if (!Errors(x))                                            // if no errors at all
            result += (SonicCalcLiters(x) - iBarrel[x]._volume_min); // add this barrel content minux wasted liters to sum
    return result;
}

// totally empty (by ultrasonic)
bool BarrClass::isDry(byte barrel)
{
    #ifdef USE_FLOW_INSTEAD
    return (FreshGet(barrel) + NutriGet(barrel) + (Flow.Counted(NUTRIENTS) / Flow.Divider(NUTRIENTS))) <= 2;
    #else
    SonicMeasure(barrel);
    return SonicCalcLiters(barrel) < 2; // +1 spare as a safeguard
    #endif
}

// reached min level (by ultrasonic)
bool BarrClass::isEmpty(byte barrel)
{
    #ifdef USE_FLOW_INSTEAD
    return (FreshGet(barrel) + NutriGet(barrel) - (Flow.Counted(NUTRIENTS) / Flow.Divider(NUTRIENTS))) <= VolumeMin(barrel);
    // used mostly while draining now.. so its totoal litrage minus drained so far
    #else
    SonicMeasure(barrel);
    if (!Errors(barrel) && (SonicCalcLiters(barrel) <= iBarrel[barrel]._volume_min) )
        {
            LOG.printf("Barrel %u is empty\r\n", barrel);
        }
    return SonicCalcLiters(barrel) <= iBarrel[barrel]._volume_min;    
    #endif
}

// reached max level (by ultrasonic)
bool BarrClass::isFull(byte barrel)
{
    #ifdef USE_FLOW_INSTEAD
    return (FreshGet(barrel) + NutriGet(barrel) + (Flow.Counted(NUTRIENTS) / Flow.Divider(NUTRIENTS))) >= VolumeMax(barrel);
    // used mostly while filling/storing now.. so its totoal litrage plus added so far
    #else
    SonicMeasure(barrel);
    if (SonicCalcLiters(barrel) >= iBarrel[barrel]._volume_max)
        {
            LOG.printf("Barrel %u is full\r\n", barrel);
        }
    return SonicCalcLiters(barrel) >= iBarrel[barrel]._volume_max;
    #endif
}

// test all sonic sensors 
void BarrClass::TestSensors()
{
    for(byte x=0;x<NUM_OF_BARRELS;x++)
    {
        SonicMeasure(x, 1);
        //vTaskDelay(10); //without delay first, trying to reproduce mux overload bug
    }
}
/*-------- Barrels END ----------*/
BarrClass Barrels;