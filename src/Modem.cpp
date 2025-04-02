//#include "main.h"
#include "Modem.h"
#include "Expanders.h"
// #include "SystemState.h"
// #include "Barrels.h"
// #include "FlowSensor.h"
// #include "PresureSensor.h"
// #include "WebServer.h"
// #include "Filesystem.h"
// #include "Modem.h"
// #include "NTPClient.h"
// #include "FMSD.h"
#include "globals.h"


/*-------- Modem Begin ----------*/
// message... item number (optional)
void SendSMS(const char *message, byte item)
{ //byte item=0xFF moved to declaration BOF
    if (item != 0xFF)
    {
        LOG.printf("sms: %s %u\r\n", message, item);
    }
    else
    {
        LOG.printf("sms: %s\r\n", message);
    }

    Expanders.LockMUX(7);                          // modem is at port 7
    vTaskDelay(10);                                 // wait until expander + mux did their job
    Serial2.println("AT+CMGS=\"+100000000\""); // number including the country code to send sms alerts to
    // check AT+OK needed here!
    Serial2.print(message);                       //text content
    if (item != 0xFF)
        Serial2.print(item);
    Serial2.write(26); // send ctrl+z end of message
    //Serial2.print((char)26); // if the above won't work - thy this one instead
    Expanders.UnlockMUX();
    //Serial.println(); // add newline after sendsms for log readability
}

/*
module needs 3.4V to 4.4V (Ideal 4.1V) @ 2amp
mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
mySerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
mySerial.println("AT+CREG?"); //Check whether it has registered in the network
ATI – Get the module name and revision
AT+COPS? – Check that you’re connected to the network, in this case BSNL
AT+COPS=? – Return the list of operators present in the network.
AT+CBC – will return the lipo battery state. The second number is the % full (in this case its 93%) and the third number is the actual voltage in mV (in this case, 3.877 V)
*/

void modemInit()
{
    #ifdef DEBUG_MORE
    LOG.println("-modem init");
    #endif
    Expanders.LockMUX(7); // modem is at port 7
    vTaskDelay(10);     // wait until expander + mux did their job
    // Serial2.begin(9600, SERIAL_8N1); // already done in main
    Serial2.println("AT");        //Once the handshake test is successful, it will back to OK
    Serial2.println("AT+CMGF=1"); // Configuring TEXT mode
    Expanders.UnlockMUX();        // Must unlock after every use!!
}
/*-------- Modem END ----------*/
